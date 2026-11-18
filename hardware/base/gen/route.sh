#!/bin/sh
# Route hardware/base with KiCadRoutingTools (KiCad PCM plugin
# com.github.drandyhaas.kicadroutingtools). Overwrites base.kicad_pcb with the
# routed board; the placed-only input is kept in the log folder.
#
# Layer strategy (chosen by measuring four strategies on the placed board, see
# REVIEW.md): strip old copper; GND poured on both layers first; power and
# motor nets routed with B.Cu preferred; every other net with F.Cu preferred,
# the power copper kept as routed; stitching vias every 16 mm between the two
# pours. Vias kept off SMD pads (hand soldering); power widths per net class;
# size/clearance never below the board's own rules and project rules
# untouched, so KiCad DRC (zone refill) grades the board's own rules.
set -e
G=$(cd "$(dirname "$0")" && pwd); B=$(dirname "$G")
KRT=${KRT:-$HOME/.local/share/kicad/10.0/3rdparty/plugins/com_github_drandyhaas_kicadroutingtools}
W=$(mktemp -d)
COMMON="--same-net-pad-clearance 0.2 --fab-tier standard --escalation board --no-fix-drc-settings"
PWR='/VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3 /GND'
PWW='0.8 0.8 0.6 0.8 0.8 0.5 0.5 0.5'
cd "$KRT"
python3 -X utf8 py_router/copy_board.py "$B/base.kicad_pcb" "$W/in.kicad_pcb" >/dev/null
python3 "$G/strip_copper.py" "$W/in.kicad_pcb" "$W/in.kicad_pcb"
python3 -X utf8 py_router/route_planes.py "$W/in.kicad_pcb" "$W/poured.kicad_pcb" \
    --nets /GND /GND --plane-layers F.Cu B.Cu > "$W/planes.log" 2>&1
# shellcheck disable=SC2086
python3 -X utf8 py_router/route.py "$W/poured.kicad_pcb" "$W/power.kicad_pcb" \
    --nets '/VBAT' '/VBAT_F' '/VBAT_SW' '/MOT_?_A' '/MOT_?_B' '/+5V*' '/+3V3' $COMMON \
    --layers F.Cu B.Cu --layer-costs 10.0 1.0 \
    --power-nets $PWR --power-nets-widths $PWW > "$W/route_power.log" 2>&1
# shellcheck disable=SC2086
python3 -X utf8 py_router/route.py "$W/power.kicad_pcb" "$W/routed.kicad_pcb" --nets '*' \
    $COMMON --keep-input-copper --layers F.Cu B.Cu --layer-costs 1.0 10.0 \
    --power-nets $PWR --power-nets-widths $PWW > "$W/route.log" 2>&1
grep -h -E 'Single-ended:|Multi-point:' "$W/route_power.log" "$W/route.log"
python3 -X utf8 py_router/route_planes.py "$W/routed.kicad_pcb" "$W/stitched.kicad_pcb" \
    --nets /GND /GND --plane-layers F.Cu B.Cu --skip-existing-zones \
    --stitch-vias --stitch-pitch 16 --same-net-pad-clearance 0.2 > "$W/stitch.log" 2>&1
grep -E 'Stitch vias placed' "$W/stitch.log"
python3 -X utf8 py_router/copy_board.py "$W/stitched.kicad_pcb" "$B/base.kicad_pcb" >/dev/null
cd "$B"
kicad-cli pcb drc --refill-zones --save-board --schematic-parity --format json \
    -o "$W/drc.json" "$B/base.kicad_pcb" | grep Found
python3 -c "import json,collections,sys;d=json.load(open(sys.argv[1]));print(collections.Counter((x['severity'],x['type']) for x in d['violations']))" "$W/drc.json"
echo "logs in $W"
