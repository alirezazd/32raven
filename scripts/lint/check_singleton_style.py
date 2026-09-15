#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Check the shape every driver and service singleton is supposed to have.

Forty classes across the two firmwares share one construction: a static
GetInstance, a private Init the System calls, deleted copy members, and a
defaulted private constructor. Where one of them drifts it is nearly always an
oversight rather than a decision, and nothing catches it -- the code compiles
either way, and the consequence shows up as a second initialization on a live
peripheral or a silently copied singleton.

Rules:

  inline-getinstance
                A GetInstance body in the header. It emits the
                `static X instance;` initialization decision as a COMDAT in
                every including TU, and any TU that can see a
                constexpr-eligible constructor may constant-initialize it: one
                non-zero member drags the whole object, zero-filled buffers
                included, out of .bss into .data, and flash stores the zeros.
                That cost 17 KB. Class templates are exempt -- their body has
                nowhere else to go.
  singleton-copy
                Copy constructor and copy assignment not deleted. A private
                destructor already makes a copy fail, but at the point it is
                destroyed rather than the point it is made, which is a
                confusing place to learn it.
  system-member
                A driver or service held as a System member rather than
                reached through GetInstance. The other rules only look at a
                class that already declares one, so a component that never
                opted in was never checked at all: EscBootloader sat in
                Drivers for months with none of this shape and nothing said
                so. This rule asks the opposite question -- of every member
                System declares, is its type a driver or a service? -- and so
                cannot be evaded by leaving the pattern out. Control/ and
                Core/ objects stay members: they own no peripheral, and the
                two folders this covers are the ones meant to be uniform.
  singleton-init
                An Init reachable twice. Private is the usual answer, since
                only `friend class System` can then reach it. A public Init is
                allowed where the caller cannot be System -- the two that need
                an AppContext are constructed after System::Init -- but then
                the definition has to open with the guard, before anything
                else runs:

                    void Foo::Init(...) {
                      if (initialized_) {
                        Panic(ErrorCode::Stm32::kFooReinit);
                      }

                Recognising a mandated form rather than inferring intent from
                arbitrary code is deliberate: "guarded somewhere" cannot be
                checked, "guarded first" can.

Run:
  uv run --quiet --script scripts/lint/check_singleton_style.py
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

# Each entry needs a reason.
ALLOWED = {
    # Kicked from the panic loop when nothing else -- System included -- is
    # trusted, and Kick() must stay a single inlined register write. Trivial
    # class, no buffers, so the .data trap cannot bite it.
    "stm32/Drivers/watchdog.hpp": {"inline-getinstance"},
}

INLINE_BODY = re.compile(r"static\s+[\w:]+\s*&\s*GetInstance\(\)\s*\{")
DECLARES_GETINSTANCE = re.compile(r"\bGetInstance\s*\(")
CLASS_HEAD = re.compile(r"^\s*(class|struct)\s+(\w+)\s*(?:final\s*)?(?::|\{|$)")
ACCESS = re.compile(r"^\s*(public|private|protected)\s*:")
TEMPLATE_LINE = re.compile(r"^\s*template\s*<")
INIT_DECL = re.compile(r"^\s*(?:[\w:<>,\s*&]+\s)?Init\s*\(")
GUARD = re.compile(
    r"^\s*if\s*\(\s*initialized_\s*\)\s*\{\s*$\s*^\s*Panic\(", re.M
)


class ClassInfo:
    """One class body, with the facts the rules ask about."""

    def __init__(self, name: str, line: int, is_template: bool):
        self.name = name
        self.line = line
        self.is_template = is_template
        self.has_getinstance = False
        self.inline_getinstance = 0
        self.deletes_copy = False
        self.deletes_assign = False
        self.init_line = 0
        self.init_access = ""


def is_template(lines: list[str], head: int) -> bool:
    """Whether a template head precedes the class, over however many lines."""
    for j in range(head - 1, -1, -1):
        stripped = lines[j].strip()
        if not stripped or stripped.startswith("//"):
            continue
        if TEMPLATE_LINE.match(lines[j]):
            return True
        # A template head wraps without punctuation; anything that closes a
        # statement means the class stands on its own.
        if stripped.endswith((";", "{", "}")):
            return False
    return False


def parse_classes(lines: list[str]) -> list[ClassInfo]:
    """Every class body in the file, with members read at its top level."""
    out: list[ClassInfo] = []
    i, total = 0, len(lines)
    while i < total:
        head = CLASS_HEAD.match(lines[i])
        if not head:
            i += 1
            continue
        keyword, name = head.group(1), head.group(2)
        # A forward declaration or a variable of class type opens no body.
        brace = i
        while brace < total and "{" not in lines[brace]:
            if ";" in lines[brace]:
                brace = total
                break
            brace += 1
        if brace >= total:
            i += 1
            continue

        info = ClassInfo(name, i + 1, is_template(lines, i))
        access = "private" if keyword == "class" else "public"
        depth, k = 0, brace
        while k < total:
            text = lines[k]
            if depth == 1:
                found = ACCESS.match(text)
                if found:
                    access = found.group(1)
                else:
                    read_member(info, text, k, access, name)
            depth += text.count("{") - text.count("}")
            if depth <= 0 and k > brace:
                break
            k += 1
        out.append(info)
        i = k + 1
    return out


def read_member(
    info: ClassInfo, text: str, idx: int, access: str, cls: str
) -> None:
    if DECLARES_GETINSTANCE.search(text):
        info.has_getinstance = True
        if INLINE_BODY.search(text):
            info.inline_getinstance = idx + 1
    if re.search(rf"{cls}\s*\(\s*const\s+{cls}\s*&\s*\)\s*=\s*delete", text):
        info.deletes_copy = True
    if re.search(
        rf"operator=\s*\(\s*const\s+{cls}\s*&\s*\)\s*=\s*delete", text
    ):
        info.deletes_assign = True
    if not info.init_line and INIT_DECL.match(text):
        info.init_line = idx + 1
        info.init_access = access


def init_is_guarded(header: pathlib.Path, cls: str) -> bool:
    """Whether Foo::Init opens with the mandated re-init guard."""
    for source in (header.with_suffix(".cpp"), *header.parent.glob("*.cpp")):
        if not source.exists():
            continue
        text = source.read_text(encoding="utf-8", errors="replace")
        opened = re.search(rf"\b{cls}::Init\s*\([^)]*\)[^{{]*\{{", text)
        if not opened:
            continue
        return bool(GUARD.match(text[opened.end() :].lstrip("\n")))
    return False


def check(path: pathlib.Path, rel: str) -> list[str]:
    exempt = ALLOWED.get(rel, set())
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    findings: list[str] = []
    for info in parse_classes(lines):
        if not info.has_getinstance:
            continue
        if (
            info.inline_getinstance
            and not info.is_template
            and "inline-getinstance" not in exempt
        ):
            findings.append(
                f"{rel}:{info.inline_getinstance}: "
                f"[inline-getinstance] {info.name} defines GetInstance in the "
                f"header"
            )
        if (
            not (info.deletes_copy and info.deletes_assign)
            and "singleton-copy" not in exempt
        ):
            findings.append(
                f"{rel}:{info.line}: [singleton-copy] {info.name} does not "
                f"delete its copy members"
            )
        if (
            info.init_line
            and info.init_access != "private"
            and "singleton-init" not in exempt
            and not init_is_guarded(path, info.name)
        ):
            findings.append(
                f"{rel}:{info.init_line}: [singleton-init] {info.name}::Init "
                f"is {info.init_access} and opens with no re-init guard"
            )
    return findings


# Every System, and the folders it draws its components from. esp32/main is
# the application -- the state machine and its pages -- so it is not one.
SYSTEMS = ("stm32/Core/system.hpp", "esp32/services/system.hpp")
COMPONENT_DIRS = (
    "stm32/Drivers",
    "stm32/Services",
    "esp32/drivers",
    "esp32/services",
)
MEMBER = re.compile(r"^  ([A-Z][\w:]*) (\w+_)\s*;")
# The receiver of the Init call in an InitComponent case: a singleton by name,
# a System member, or a System accessor.
INIT_BY_SINGLETON = re.compile(r"\b(\w+)::GetInstance\(\)\s*\.Init\(")
INIT_BY_MEMBER = re.compile(r"^\s*(\w+_)\.Init\(")
INIT_BY_ACCESSOR = re.compile(r"(?:^|\.|>)\s*(\w+)\(\)\s*\.Init\(")
# An accessor body, which either hands out a singleton or hands out a member.
ACCESSOR = re.compile(
    r"^\s*[\w:]+\s*&\s*(\w+)\(\)\s*\{\s*return\s+([\w:]+)"
    r"(?:::GetInstance\(\))?;"
)

# A class in a component folder that is not a component. Each entry needs a
# reason: the alternative is inferring it, and a component that forgot the
# whole pattern looks exactly like a class that never needed it.
NOT_A_COMPONENT = {
    # Interfaces. The implementations are the components, and each of those
    # is brought up by name.
    "IMavlinkTransport": "the transport interface MAVLink is handed",
    "DisplayCanvas": "what a widget draws onto, owned by Ui",
    "HostLink": "the host protocol; its two transports are the components",
    # Owned by a component, constructed as part of it.
    "EeConfigStorage": "EE's record layout, held by the drivers that store one",
    "GyroCal": "a phase of SensorCalService, which owns it",
    "AccelCal": "a phase of SensorCalService, which owns it",
    "MagCal": "a phase of SensorCalService, which owns it",
    "MagFit": "MagCal's solver, held by it",
    "LevelCal": "a phase of SensorCalService, which owns it",
    "FcConfigCache": "the flight computer's config records, held by Mavlink",
    "MavlinkParamServer": "the parameter protocol, held by Mavlink",
}


def tree_headers() -> list[str]:
    """Every header in the tree, tracked or not.

    Of the tree, not the index: a header added but not yet committed is the
    one most likely to have missed the pattern, so it is the last one to
    leave unchecked.
    """
    headers: list[str] = []
    for extra in ((), ("--others", "--exclude-standard")):
        run = subprocess.run(
            ["git", "ls-files", *extra, "*.h", "*.hpp"],
            cwd=REPO,
            capture_output=True,
            text=True,
            check=True,
        )
        headers += run.stdout.split()
    return headers


def tree_classes(headers: list[str]) -> list[tuple[str, str, bool]]:
    """(class, header, declares a GetInstance) for every class in the tree.

    Read from the whole tree rather than from the files passed in: a hook
    hands over only what changed, and a component is no less a component for
    having been left alone this commit.
    """
    out: list[tuple[str, str, bool]] = []
    for rel in headers:
        if (
            not rel.endswith(".hpp")
            or rel.startswith("third_party")
            or rel in SYSTEMS
        ):
            continue
        text = (REPO / rel).read_text(encoding="utf-8", errors="replace")
        declared = {
            info.name for info in parse_classes(text.splitlines())
            if info.has_getinstance
        }
        for line in text.splitlines():
            # At namespace scope: an indented head is a nested type, which
            # belongs to the class around it rather than to the System.
            # CLASS_HEAD also wants a brace, a base clause or a wrapped head,
            # so a forward declaration is not mistaken for the definition.
            head = CLASS_HEAD.match(line)
            if head and head.group(1) == "class" and not line[0].isspace():
                out.append((head.group(2), rel, head.group(2) in declared))
    return out


def accessors(system_hpp: pathlib.Path) -> dict[str, str]:
    """A System accessor's name to what it hands out: a class, or a member."""
    out: dict[str, str] = {}
    for line in system_hpp.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines():
        found = ACCESSOR.match(line)
        if found:
            out[found.group(1)] = found.group(2).lstrip(":")
    return out


def brought_up(system_cpp: pathlib.Path, names: dict[str, str]) -> set[str]:
    """Every class System::InitComponent initializes, however it reaches it."""
    out: set[str] = set()
    for line in system_cpp.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines():
        if ".Init(" not in line:
            continue
        found = INIT_BY_SINGLETON.search(line)
        if found:
            out.add(found.group(1))
            continue
        if INIT_BY_MEMBER.match(line):
            continue  # the member rule below already has this one
        found = INIT_BY_ACCESSOR.search(line)
        if found:
            out.add(names.get(found.group(1), found.group(1)))
    return out


def system_rules(headers: list[str]) -> list[str]:
    """What System says its components are, against what they look like.

    A component is whatever InitComponent brings up, which is the one list
    that cannot drift: a class that quietly stopped being initialized stops
    being a component, and one that never adopted the pattern is still on it.
    """
    tree = tree_classes(headers)
    findings: list[str] = []

    for rel in SYSTEMS:
        hpp = REPO / rel
        if not hpp.exists():
            continue
        # One board at a time: both boards have a CommandHandler and an
        # FcLink, and one's GetInstance must not vouch for the other's.
        board = rel.split("/", 1)[0] + "/"
        classes: dict[str, str] = {}
        declared: dict[str, str] = {}
        for name, header, has_getinstance in tree:
            if not header.startswith(board):
                continue
            classes.setdefault(name, header)
            if has_getinstance:
                declared[name] = header
        components = {
            name: header
            for name, header in classes.items()
            if header.startswith(COMPONENT_DIRS)
        }
        for idx, line in enumerate(
            hpp.read_text(encoding="utf-8", errors="replace").splitlines()
        ):
            found = MEMBER.match(line)
            if found and found.group(1) in components:
                findings.append(
                    f"{rel}:{idx + 1}: [system-member] {found.group(2)} holds "
                    f"{found.group(1)}, declared in {classes[found.group(1)]}"
                )

        cpp = hpp.with_suffix(".cpp")
        if not cpp.exists():
            continue
        for name in sorted(brought_up(cpp, accessors(hpp))):
            if name in classes and name not in declared:
                findings.append(
                    f"{classes[name]}: [component-shape] {name} is brought up "
                    f"by {cpp.relative_to(REPO).as_posix()} and declares no "
                    f"GetInstance"
                )

        for name, header in sorted(components.items()):
            if name in declared or name in NOT_A_COMPONENT:
                continue
            findings.append(
                f"{header}: [component-shape] {name} is neither a singleton "
                f"nor listed in NOT_A_COMPONENT with a reason"
            )
    return findings


def main(argv: list[str]) -> int:
    headers = tree_headers()
    # CI runs with no arguments and expects a whole-tree walk.
    names = argv[1:] or [str(REPO / rel) for rel in headers]

    findings: list[str] = []
    for name in names:
        path = pathlib.Path(name)
        if path.suffix not in (".h", ".hpp") or "third_party" in path.parts:
            continue
        findings += check(path, path.resolve().relative_to(REPO).as_posix())

    findings += system_rules(headers)

    if findings:
        for finding in findings:
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nA driver or service is reached through GetInstance, never held\n"
            "as a System member, and they all share one shape. Declare\n"
            "`static X &GetInstance();` and define it in the .cpp, delete the\n"
            "copy members, and keep Init private so only the System reaches\n"
            "it -- or, where the caller cannot be System, open Init\n"
            "with `if (initialized_) { Panic(...Reinit); }`.",
            file=sys.stderr,
        )

    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
