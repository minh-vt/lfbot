#!/bin/sh
# Build home.kicad_pcb, the home-etch variant of the base board, from the
# placement of ../fab/base.kicad_pcb: strip its copper, move the divider
# resistors to the bottom and keep the top layer off pins that cannot be
# soldered from the top (prepare.py), then route with KiCadRoutingTools
# under home.kicad_pro (0.3 mm tracks, 0.25 mm clearance, 1.4/0.7 mm vias
# for wire-through vias), tidy, centre track ends on vias, place labels and
# run DRC.
#
# Overwrites home.kicad_pcb: hand fixes made after this script (see README)
# are lost. The previous board is kept in the log folder.
set -e
set -f  # net patterns such as "*" must reach the router, not the shell
G=$(cd "$(dirname "$0")" && pwd); H=$(dirname "$G"); BASE=$(dirname "$H")
KRT=${KRT:-$HOME/.local/share/kicad/10.0/3rdparty/plugins/com_github_drandyhaas_kicadroutingtools}
W=$(mktemp -d)
COMMON="--same-net-pad-clearance 0.3 --fab-tier standard --escalation board --no-fix-drc-settings"
PWR='/VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3 /GND'
PWW='0.8 0.8 0.6 0.8 0.8 0.5 0.5 0.5'
[ -f "$H/home.kicad_pcb" ] && cp "$H/home.kicad_pcb" "$W/previous.kicad_pcb"
python3 "$BASE/gen/strip_copper.py" "$BASE/fab/base.kicad_pcb" "$W/in.kicad_pcb"
python3 "$G/prepare.py" "$W/in.kicad_pcb"
cp "$H/home.kicad_pro" "$W/in.kicad_pro"
cd "$KRT/py_router"
python3 -X utf8 route_planes.py "$W/in.kicad_pcb" "$W/p0.kicad_pcb" \
    --nets /GND /GND --plane-layers F.Cu B.Cu > "$W/planes.log" 2>&1
# Routing passes, in order: "COST_F:COST_B NET ...". Earlier passes get the
# straight paths; override with PASSES="...;..." to try another order.
# Power first on the bottom, then the three ToF lines so they keep the
# straight path through the J3 pin gaps, then the rest (chosen by comparing
# orders with route_review.py of KiCadRoutingTools; see README).
PASSES=${PASSES:-"10:1 /VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3;1:1 /I2C_SDA /I2C_SCL /TOF_XSHUT;1:1 *"}
i=0
echo "$PASSES" | tr ';' '\n' > "$W/passes.txt"
while read -r costs nets; do
  cp "$H/home.kicad_pro" "$W/p$i.kicad_pro"
  # shellcheck disable=SC2086
  python3 -X utf8 route.py "$W/p$i.kicad_pcb" "$W/p$((i+1)).kicad_pcb" --nets $nets \
      $COMMON --keep-input-copper --layers F.Cu B.Cu --layer-costs ${costs%:*} ${costs#*:} \
      --power-nets $PWR --power-nets-widths $PWW > "$W/pass$((i+1)).log" 2>&1
  grep -h 'Single-ended:' "$W/pass$((i+1)).log" | tail -1
  i=$((i+1))
done < "$W/passes.txt"
cp "$W/p$i.kicad_pcb" "$W/routed.kicad_pcb"
cp "$W/routed.kicad_pcb" "$H/home.kicad_pcb"
python3 "$BASE/gen/tidy_tracks.py" "$H/home.kicad_pcb" 2>&1 | grep 'runs kept' || true
python3 "$G/center_vias.py" "$H/home.kicad_pcb" 2>&1 | grep -E 'dropped|moved' || true
rm -rf "$BASE/gen/__pycache__"
# bottom-side labels of the parts prepare.py moved there
(cd "$KRT/py_placer" && python3 -X utf8 beautify_labels.py "$H/home.kicad_pcb" "$W/labeled.kicad_pcb" \
    --refs C2 R1 R2 > "$W/labels.log" 2>&1)
cp "$W/labeled.kicad_pcb" "$H/home.kicad_pcb"
kicad-cli pcb drc --refill-zones --save-board --schematic-parity --format json \
    -o "$W/drc.json" "$H/home.kicad_pcb" | grep Found
echo "logs and the previous board in $W"
