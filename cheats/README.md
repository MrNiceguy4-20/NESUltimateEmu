# NES cheat database

NESUltimateEmu reads RetroArch/libretro-style `.cht` files from `cheats/nes`.
The **Cheats** button appears in the main UI after a game is loaded. The emulator
first looks for an exact ROM basename and then falls back to normalized title
matching (for example, `Super C (USA).nes` -> `Super C (USA) (Game Genie).cht`).

Supported code forms in this checkpoint:

- NES Game Genie: 6-character address/value codes.
- NES Game Genie: 8-character address/value/compare codes.
- Multiple codes in one cheat separated with `+`.
- Simple raw CPU byte overrides: `AAAA:VV` (for example `0025:63`).

The upstream libretro database is licensed CC-BY-SA-4.0. Its NES cheat files are
not bundled in this source checkpoint; use `tools/import_libretro_nes_cheats.py`
to copy compatible entries from a local libretro-database checkout. Keeping the
third-party dataset separate avoids silently changing the license of the emulator
source and makes database updates independent of emulator releases.

Upstream: https://github.com/libretro/libretro-database
License: CC-BY-SA-4.0
