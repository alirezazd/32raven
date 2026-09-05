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


# Every System, and the folders whose classes it may not own outright.
SYSTEMS = ("stm32/Core/system.hpp", "esp32/services/system.hpp")
COMPONENT_DIRS = (
    "stm32/Drivers",
    "stm32/Services",
    "esp32/drivers",
    "esp32/services",
    "esp32/main",
)
MEMBER = re.compile(r"^  ([A-Z][\w:]*) (\w+_)\s*;")


def component_classes() -> dict[str, str]:
    """Class name to the component header that declares it."""
    out = subprocess.run(
        ["git", "ls-files", "*.hpp"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    )
    found: dict[str, str] = {}
    for rel in out.stdout.splitlines():
        if not rel.startswith(COMPONENT_DIRS) or rel in SYSTEMS:
            continue
        text = (REPO / rel).read_text(encoding="utf-8", errors="replace")
        for line in text.splitlines():
            # CLASS_HEAD wants a brace, a base clause or a wrapped head, so a
            # forward declaration is not mistaken for the definition.
            head = CLASS_HEAD.match(line)
            if head and head.group(1) == "class":
                found.setdefault(head.group(2), rel)
    return found


def owned_components() -> list[str]:
    """Members of a System whose type is a driver or a service."""
    classes = component_classes()
    findings: list[str] = []
    for rel in SYSTEMS:
        path = REPO / rel
        if not path.exists():
            continue
        for idx, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines()
        ):
            found = MEMBER.match(line)
            if not found:
                continue
            owner = classes.get(found.group(1))
            if owner:
                findings.append(
                    f"{rel}:{idx + 1}: [system-member] {found.group(2)} holds "
                    f"{found.group(1)}, declared in {owner}"
                )
    return findings


def main(argv: list[str]) -> int:
    names = argv[1:]
    if not names:
        # CI runs with no arguments and expects a whole-tree walk.
        out = subprocess.run(
            ["git", "ls-files", "*.h", "*.hpp"],
            cwd=REPO,
            capture_output=True,
            text=True,
            check=True,
        )
        names = [str(REPO / line) for line in out.stdout.splitlines()]

    findings: list[str] = owned_components()
    for name in names:
        path = pathlib.Path(name)
        if path.suffix not in (".h", ".hpp") or "third_party" in path.parts:
            continue
        findings += check(path, path.resolve().relative_to(REPO).as_posix())

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
