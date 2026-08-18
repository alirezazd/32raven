# Development setup

Everything is driven from the top-level `Makefile`, and every target runs in one of two
modes — inside a container that carries the whole toolchain, or directly against tools on
your machine. Pick one; the commands are identical either way, and you can switch later.

## Clone

The vendored dependencies are git submodules — ESP-IDF among them — so a clone is not
complete without them:

```bash
git clone --recursive https://github.com/alirezazd/32raven.git
```

Already cloned without the flag? `git submodule update --init --recursive` catches up.

## Pick a build mode

=== "Docker (recommended)"

    The only host requirement is a working container engine — ARM GCC, CMake, Ninja, uv and
    the ESP-IDF host prerequisites all live in the build image. Install Docker per the
    [official docs](https://docs.docker.com/engine/install/), add yourself to the `docker`
    group, and log out and back in. Podman works too — the Makefile falls back to it
    automatically when `docker` is not on the `PATH`.

    ```bash
    make enable-docker
    ```

    The first run builds the image. The mode is sticky — it persists in `.build-mode`
    across shells until `make disable-docker` — and a single build can step outside it:
    `USE_DOCKER=0 make stm32`.

    !!! tip

        **VSCode users can skip the toggle entirely.** With the Dev Containers extension,
        opening the repository in a container reuses the same `Dockerfile`, initialises the
        submodules, installs the ESP-IDF tools on first open, and adds the CMake and clangd
        extensions.

=== "Host toolchain"

    - **CMake** 3.22+
    - **ARM GCC** (`arm-none-eabi-*`) for the STM32
    - **Python 3** and **[uv](https://docs.astral.sh/uv/getting-started/installation/)** —
      no manual package installs. Every first-party script declares its own dependencies
      inline ([PEP 723](https://peps.python.org/pep-0723/)) and the build invokes them via
      `uv run --script`, so uv resolves and caches them on first use.
    - **ripgrep** and **clang-format** for `make format-cpp`
    - The ESP-IDF host prerequisites
      ([Espressif's list](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/get-started/linux-macos-setup.html#install-prerequisites))
      for ESP32 builds
    - **Serial-port access** for flashing and monitoring — add yourself to the `dialout`
      group once with `sudo usermod -aG dialout $USER`, then log out and back in. Without
      it, opening the port fails in a way that looks exactly like the board not being
      plugged in.

## Install the ESP-IDF tools

ESP-IDF itself is pinned as a submodule at `third_party/esp-idf` (v5.5.3), so there is
nothing to download — but its toolchain is a one-time install, needed only for ESP32
builds:

```bash
make idf-install
```

In Docker mode the tools land in `.docker/home/.espressif` (gitignored); on the host, in
`~/.espressif`. Either way they persist across builds.

## Local overrides — `user_config.cmake`

The serial port and baud rate resolve from the USB descriptor, so neither normally needs
setting. To pin them anyway — or to point the build at an out-of-tree ESP-IDF or ARM
toolchain — copy the example and uncomment what you need:

```bash
cp user_config.cmake.example user_config.cmake
```

The file is gitignored and read by the build when present.

## VSCode workspace settings (optional)

```bash
make setup-vscode
```

Applies recommended settings: submodule scanning off (Source Control performance with
ESP-IDF checked out), file-watcher and search exclusions for vendored code, and
`clangd.arguments` so clangd queries both the `arm-none-eabi` and `riscv32-esp-elf` GCC
drivers for system headers — without which `<cstdint>` and friends do not resolve across
the two targets. It works in both build modes, and nothing it writes is committed — you
control what applies to your workspace.

Next: [configure and build](building.md).
