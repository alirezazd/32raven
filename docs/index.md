# 32Raven Handbook

**32Raven** is a bare-metal flight-control stack split across two MCUs: an **STM32F407**
running the flight-critical loop and an **ESP32-C3** carrying the connectivity surface.
They share nothing but a versioned wire protocol.

This handbook is the practical layer — how to **build one**, wire it, flash it, bring it up,
and fly it. The repository README covers the architecture; these pages cover the hardware in
front of you.

[:material-book-open-variant: Start the build guide](build/index.md){ .md-button .md-button--primary }

!!! warning "Status: active development"

    Interfaces, configs, and pin assignments change quickly. Every pin and tunable quoted
    here is checked against `config/Kconfig` on each build (see below), but a page can still
    describe a stage that has moved on. Build from a tagged release if you want stability.

!!! danger "This aircraft can injure you"

    A 6S quadcopter with props on is a serious hazard. Nothing in this handbook is a
    substitute for your own judgement: props stay **off** the motors until the bench-test
    stage explicitly says otherwise, and the battery stays disconnected while you wire.

## Where to start

<div class="grid cards" markdown>

- **[Build the prototype](build/index.md)**

    Parts, wiring, power, and assembly — the from-scratch path. Start with
    [the wiring reference](build/wiring.md).

</div>

## How this handbook stays honest

Docs that quote pin numbers rot the instant a pin moves, and a stale pin number is not a
typo — it is a destroyed ESC. Three gates run on every push:

- **`mkdocs build --strict`** — broken internal links, orphan pages, dangling heading
  anchors, and absolute links (which break under the `/32raven/` Pages path) all fail.
- **Embedded code is transcluded, not copied.** Code blocks pull from the real source via
  [pymdownx.snippets][snip] anchors, so an embedded block cannot drift from the file it
  documents.
- **`scripts/lint/check_docs.py`** — catches what `--strict` cannot see: a transclusion
  anchor whose `[start:…]` marker was deleted (which silently embeds *nothing*), GitHub-style
  `> [!NOTE]` blocks that render broken in Material, and any `CONFIG_STM32_*` /
  `CONFIG_ESP32_*` symbol named here that no longer exists in `config/Kconfig`.

[snip]: https://facelessuser.github.io/pymdown-extensions/extensions/snippets/

## Building this handbook locally

```bash
make docs          # strict build into ./site, then the lint described above
make docs-serve    # live-reload preview at http://127.0.0.1:8000
```

The toolchain installs through **uv** as an opt-in dependency group — it is not part of the
firmware build, and `make docs` resolves it on demand.
