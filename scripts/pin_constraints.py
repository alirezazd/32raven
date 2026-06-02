"""STM32 silicon-level pin/signal/AF constraint database.

Loads the vendored open-pin-data XML for one MCU + GPIO IP version, exposes
a small query API that downstream tooling (validators, Kconfig generators)
can use.

Currently wired for STM32F407V(E-G)Tx. To add a second MCU later, parameterize
the file paths and add a key. The query API stays the same.

Usage:
    >>> db = PinConstraints.load_default()
    >>> db.is_valid_signal_on_pin("PB13", "SPI2_SCK")
    True
    >>> db.af_for("PB13", "SPI2_SCK")
    'GPIO_AF5_SPI2'
    >>> db.pins_for_signal("SPI2_SCK")
    [PinAf(pin='PB10', af='GPIO_AF5_SPI2'),
     PinAf(pin='PB13', af='GPIO_AF5_SPI2'),
     PinAf(pin='PI1',  af='GPIO_AF5_SPI2')]
"""

from __future__ import annotations

import pathlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DATA_ROOT = REPO_ROOT / "third_party" / "stm32_open_pin_data"
MCU_FILE = DATA_ROOT / "mcu" / "STM32F407V(E-G)Tx.xml"
GPIO_IP_FILE = DATA_ROOT / "mcu" / "IP" / "GPIO-STM32F417_gpio_v1_0_Modes.xml"


@dataclass(frozen=True)
class PinAf:
    pin: str  # e.g. "PB13"
    af: str   # e.g. "GPIO_AF5_SPI2"


def _strip_ns(tag: str) -> str:
    """ElementTree returns Clark notation '{ns}tag'; we only care about the local name."""
    return tag.split("}", 1)[-1] if "}" in tag else tag


class PinConstraints:
    """In-memory view of the F407 pin → signal → AF mapping.

    Construction parses the two XMLs once; lookups are O(1) afterwards.
    """

    def __init__(self, package_pins: set[str], pin_signal_af: dict[tuple[str, str], str]):
        self._package_pins = package_pins
        self._pin_signal_af = pin_signal_af

    @classmethod
    def load_default(cls) -> "PinConstraints":
        return cls.load(MCU_FILE, GPIO_IP_FILE)

    @classmethod
    def load(cls, mcu_xml: pathlib.Path, gpio_ip_xml: pathlib.Path) -> "PinConstraints":
        package_pins = cls._parse_package_pins(mcu_xml)
        pin_signal_af = cls._parse_pin_signal_af(gpio_ip_xml, package_pins)
        return cls(package_pins=package_pins, pin_signal_af=pin_signal_af)

    @staticmethod
    def _parse_package_pins(mcu_xml: pathlib.Path) -> set[str]:
        """Pins available on this package (e.g. F407V LQFP100 has 82 GPIOs)."""
        pins: set[str] = set()
        root = ET.parse(mcu_xml).getroot()
        for el in root.iter():
            if _strip_ns(el.tag) != "Pin":
                continue
            if el.get("Type") != "I/O":
                continue
            # Names like "PA0-WKUP", "PB7" — keep just the port+pin token.
            name = (el.get("Name") or "").split("-")[0].split("/")[0].strip()
            if name.startswith("P") and len(name) >= 3 and name[2:].isdigit():
                pins.add(name)
        return pins

    @staticmethod
    def _parse_pin_signal_af(
        gpio_ip_xml: pathlib.Path,
        keep_pins: set[str],
    ) -> dict[tuple[str, str], str]:
        """Map (pin, signal) -> 'GPIO_AFxx_PERIPH' macro name.

        Only retains entries for pins that exist on the configured package.
        """
        out: dict[tuple[str, str], str] = {}
        root = ET.parse(gpio_ip_xml).getroot()
        for gp in root.iter():
            if _strip_ns(gp.tag) != "GPIO_Pin":
                continue
            pin = gp.get("Name") or ""
            if pin not in keep_pins:
                continue
            for ps in gp:
                if _strip_ns(ps.tag) != "PinSignal":
                    continue
                signal = ps.get("Name") or ""
                if not signal:
                    continue
                # Find <SpecificParameter Name="GPIO_AF"><PossibleValue>GPIO_AFn_PERIPH</PossibleValue>
                for sp in ps:
                    if _strip_ns(sp.tag) != "SpecificParameter":
                        continue
                    if sp.get("Name") != "GPIO_AF":
                        continue
                    for pv in sp:
                        if _strip_ns(pv.tag) == "PossibleValue" and (pv.text or "").strip():
                            out[(pin, signal)] = pv.text.strip()
                            break
                    break
        return out

    # ---- query API -------------------------------------------------------

    @property
    def package_pins(self) -> set[str]:
        return self._package_pins

    def is_valid_pin(self, pin: str) -> bool:
        return pin in self._package_pins

    def is_valid_signal_on_pin(self, pin: str, signal: str) -> bool:
        return (pin, signal) in self._pin_signal_af

    def af_for(self, pin: str, signal: str) -> str | None:
        """Return the AF macro (e.g. 'GPIO_AF5_SPI2') or None if invalid."""
        return self._pin_signal_af.get((pin, signal))

    def afs_for_pin(self, pin: str) -> set[str]:
        """Every AF macro valid on `pin` across all its signals — e.g. to list
        the legal alternates when a (pin, AF) combo is rejected."""
        return {af for (p, _), af in self._pin_signal_af.items() if p == pin}

    def pins_for_signal(self, signal: str) -> list[PinAf]:
        return sorted(
            (PinAf(pin=pin, af=af)
             for (pin, sig), af in self._pin_signal_af.items()
             if sig == signal),
            key=lambda x: x.pin,
        )

    def signals_for_pin(self, pin: str) -> list[PinAf]:
        return sorted(
            (PinAf(pin=sig, af=af)  # `pin` field repurposed to carry the signal name
             for (p, sig), af in self._pin_signal_af.items()
             if p == pin),
            key=lambda x: x.pin,
        )

    def all_signals(self) -> set[str]:
        return {sig for (_, sig) in self._pin_signal_af}


if __name__ == "__main__":
    # Lightweight smoke check: print stats so a dev can sanity-eyeball the load.
    db = PinConstraints.load_default()
    print(f"Package pins:    {len(db.package_pins)}")
    # Debug smoke-test in this module's own __main__ — reaching into its own
    # class's internals here is fine. pylint: disable=protected-access
    print(f"(pin,sig)→AF:    {len(db._pin_signal_af)}")  # pylint: disable=protected-access
    print(f"Distinct signals:{len(db.all_signals())}")
    for sig in ("SPI2_SCK", "USART1_TX", "TIM1_CH1", "CAN1_RX"):
        choices = db.pins_for_signal(sig)
        print(f"  {sig:14s} -> {[c.pin for c in choices] or 'NOT FOUND'}")
