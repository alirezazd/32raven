# Vendored: STM32 open pin data (subset)

Source: <https://github.com/STMicroelectronics/STM32_open_pin_data>, MIT-licensed by ST.
Vendored as plain files (not a submodule) because we only need a few KB out of a 2000-file repo.

## Files

- `mcu/STM32F407V(E-G)Tx.xml` — package-level pin list (which pins exist on this package).
- `mcu/IP/GPIO-STM32F417_gpio_v1_0_Modes.xml` — `(pin, signal) → AF` mapping.
  The F407 MCU XML references this F417 GPIO IP version (same silicon for GPIOs).

## How to refresh

Re-fetch from the upstream repo if ST publishes corrections:

```bash
cd third_party/stm32_open_pin_data
python3 - <<'PY'
import urllib.request, urllib.parse, os
BASE = "https://raw.githubusercontent.com/STMicroelectronics/STM32_open_pin_data/master"
for f in [
    "mcu/STM32F407V(E-G)Tx.xml",
    "mcu/IP/GPIO-STM32F417_gpio_v1_0_Modes.xml",
]:
    urllib.request.urlretrieve(f"{BASE}/{urllib.parse.quote(f)}", f)
    print(f"{f}: {os.path.getsize(f)} bytes")
PY
```

## Used by

- [scripts/pin_constraints.py](../../scripts/pin_constraints.py) — Python query API over this data.
- [scripts/check_pinmap.py](../../scripts/check_pinmap.py) — validates [stm32/Core/Inc/board.hpp](../../stm32/Core/Inc/board.hpp).
