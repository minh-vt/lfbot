#!/bin/sh
# Regenerate the schematic and the custom footprints from design.py /
# gen_fp.py, then run ERC. The board is not touched: base.kicad_pcb is the
# layout source; after a pin->net change, update the board's pads (KiCad:
# Update PCB from Schematic) and check with DRC --schematic-parity.
set -e
G=$(cd "$(dirname "$0")" && pwd); B=$(dirname "$G"); S=$(mktemp -d)
cd "$G"
python3 gen_sch.py "$B/base.kicad_sch"
python3 gen_fp.py "$B/Base.pretty"
kicad-cli sch erc --format json -o "$S/erc.json" "$B/base.kicad_sch" | grep Found
rm -rf "$G/__pycache__"
