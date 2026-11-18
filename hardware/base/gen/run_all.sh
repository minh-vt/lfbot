#!/bin/sh
# Rebuild hardware/base from design.py: schematic -> netlist -> footprints -> placed PCB -> DRC.
# Routing is a separate step: route.sh (KiCadRoutingTools).
# Overwrites base.kicad_sch / base.kicad_pcb / Base.pretty: do not run after hand edits in KiCad.
set -e
# base.kicad_pcb now holds the KiCadRoutingTools placement (seed 3 + hand
# nudges) and its route; design.py still has the earlier v0.5 poses. Running
# this would throw that away, so it needs an explicit FORCE=1.
[ "${FORCE:-0}" = 1 ] || { echo "refusing: base.kicad_pcb is the KRT placement; FORCE=1 to regenerate from design.py"; exit 2; }
G=$(cd "$(dirname "$0")" && pwd); B=$(dirname "$G"); S=$(mktemp -d)
cd "$G"
python3 gen_sch.py $B/base.kicad_sch
kicad-cli sch erc --format json -o $S/erc.json $B/base.kicad_sch | grep Found
kicad-cli sch export netlist --format kicadsexpr -o $S/base.net $B/base.kicad_sch >/dev/null
python3 gen_fp.py $B/Base.pretty
python3 gen_pcb.py $B $S/base.net 2>&1 | grep -v Debug
kicad-cli pcb drc --schematic-parity --format json -o $S/drc.json $B/base.kicad_pcb | grep Found
python3 -c "import json,collections,sys;d=json.load(open(sys.argv[1]));print(collections.Counter((x['severity'],x['type']) for x in d['violations']))" $S/drc.json
echo "reports in $S"
