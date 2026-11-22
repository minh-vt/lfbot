#!/bin/sh
# Regenerate the schematic and the custom footprints from design.py /
# gen_fp.py, then run ERC. The boards are not touched: fab/base.kicad_pcb is the
# layout source; after a pin->net change, update the board's pads (KiCad:
# Update PCB from Schematic) and check with DRC --schematic-parity.
set -e
G=$(cd "$(dirname "$0")" && pwd); B=$(dirname "$G"); S=$(mktemp -d)
cd "$G"
python3 gen_sch.py "$B/fab/base.kicad_sch"
# Home-etch variant (home/): same parts and nets, its own project.
python3 gen_sch.py "$B/home/home.kicad_sch"
# Single-sided variant (single/): same parts; pins changed per VARIANT_PINS.
python3 gen_sch.py "$B/single/single.kicad_sch"
python3 gen_fp.py "$B/Base.pretty"
kicad-cli sch erc --format json -o "$S/erc.json" "$B/fab/base.kicad_sch" | grep Found
kicad-cli sch erc --format json -o "$S/erc_home.json" "$B/home/home.kicad_sch" | grep Found
kicad-cli sch erc --format json -o "$S/erc_single.json" "$B/single/single.kicad_sch" | grep Found
rm -rf "$G/__pycache__"
