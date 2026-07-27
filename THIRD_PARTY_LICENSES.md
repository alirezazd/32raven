# Third-Party Licenses

32raven bundles or depends on the third-party components listed below. These
components are **not** covered by 32raven's own licence (see [LICENSE](./LICENSE)
and [COPYRIGHT](./COPYRIGHT)); each retains its upstream licence and copyright.

Every entry was verified against the submodule / vendored directory actually
present in this repository (submodule remotes from `.gitmodules`; licence text
from the `LICENSE`/`COPYING` files shipped in each component).

| Component | Path | Upstream | License |
|-----------|------|----------|---------|
| MAVLink c_library_v2 | `third_party/mavlink/` | https://github.com/mavlink/c_library_v2 | MIT |
| ESP-IDF | `third_party/esp-idf/` | https://github.com/espressif/esp-idf | Apache-2.0 |
| Adafruit-GFX-Library | `third_party/Adafruit-GFX-Library/` | https://github.com/adafruit/Adafruit-GFX-Library | BSD-3-Clause |
| Eigen | `third_party/eigen/` | https://gitlab.com/libeigen/eigen | MPL-2.0 (with minor parts under Apache-2.0, BSD, and MINPACK terms) |
| nanoprintf | `third_party/nanoprintf/` | https://github.com/charlesnicholson/nanoprintf | Unlicense OR 0BSD (public domain) |
| STM32 open pin data | `third_party/stm32_open_pin_data/` | https://github.com/STMicroelectronics/STM32_open_pin_data | BSD-3-Clause (© STMicroelectronics) |
| CMSIS + STM32F4 device headers | `stm32/lib/CMSIS_F4xx/` | Arm CMSIS / STMicroelectronics | Apache-2.0 |

## Notes

- **MAVLink** is vendored as pre-generated C headers with no `LICENSE` file in
  the tree; the upstream `c_library_v2` repository releases these headers under
  the MIT License.
- **STM32 open pin data** is distributed by ST under the **BSD 3-Clause**
  license (see `third_party/stm32_open_pin_data/LICENSE`), not a bespoke ST
  license.
- **Eigen** is predominantly MPL-2.0. A small number of files carry Apache-2.0,
  BSD, or MINPACK licenses; see `third_party/eigen/COPYING.*` and
  `third_party/eigen/LICENSES/`.
- **`stm32/lib/CMSIS_F4xx/`** contains Arm CMSIS core headers and ST device
  headers/startup code. The vendored `LICENSE.txt` (Apache-2.0) and
  `LICENSE-STMicroelectronics.txt` (defaults to Apache-2.0) apply.
- **`stm32/lib/control/`** and **`stm32/lib/math/`** are first-party 32raven
  code. They live under `stm32/lib/` for build-layout reasons and are
  intentionally left without SPDX headers per repository policy (nothing under
  `stm32/lib/` is modified by the licensing pass); they are nonetheless covered
  by the 32raven copyright and dual-licensing terms in [COPYRIGHT](./COPYRIGHT).
- `esp32/third_party/` is currently an empty placeholder directory.

Submodule commit pins are recorded in the repository's git index; run
`git submodule status` for the exact revisions in use.
