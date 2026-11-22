#!/bin/sh
# Build single.kicad_pcb, the single-sided variant of the base board, from
# the placement of ../fab/base.kicad_pcb. Only the bottom copper is etched;
# the top layer stands for jumpers (see prepare.py). Steps: strip copper,
# prepare.py, the hand-planned bus (guide.py), GND poured on the bottom
# only, route the rest with KiCadRoutingTools with the top layer made
# expensive so it is used only where the bottom has
# no way through, tidy, centre track ends on vias, turn short jumpers into
# 0 ohm parts and join wire chains (jumpers.py), join cut-off GND pour
# (gnd_bridges.py), drop copper that carries nothing (prune.py), place
# labels and run DRC.
#
# Overwrites single.kicad_pcb; the previous board is kept in the log folder.
set -e
set -f  # net patterns such as "*" must reach the router, not the shell
G=$(cd "$(dirname "$0")" && pwd); H=$(dirname "$G"); BASE=$(dirname "$H")
KRT=${KRT:-$HOME/.local/share/kicad/10.0/3rdparty/plugins/com_github_drandyhaas_kicadroutingtools}
W=$(mktemp -d)
COMMON="--same-net-pad-clearance 0.3 --fab-tier standard --escalation board --no-fix-drc-settings"
PWR='/VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3 /GND'
PWW='0.8 0.8 0.6 0.8 0.8 0.5 0.5 0.5'
[ -f "$H/single.kicad_pcb" ] && cp "$H/single.kicad_pcb" "$W/previous.kicad_pcb"
python3 "$BASE/gen/strip_copper.py" "$BASE/fab/base.kicad_pcb" "$W/in.kicad_pcb"
cp "$H/single.kicad_pro" "$W/in.kicad_pro"  # this board's rules (via size) for the steps below
python3 "$G/prepare.py" "$W/in.kicad_pcb"
python3 "$G/guide.py" "$W/in.kicad_pcb"
cd "$KRT/py_router"
python3 -X utf8 route_planes.py "$W/in.kicad_pcb" "$W/p0.kicad_pcb" \
    --nets /GND --plane-layers B.Cu > "$W/planes.log" 2>&1
# Passes in order, "COST_F:COST_B NET ...": power, the IR channels, then
# the rest (the bus is already laid by guide.py). GND is left out of every
# pass ('!/GND'): the bottom pour carries it, and KRT's in-run pour taps would
# otherwise lay GND copper on the top, where no pin may take it.
P='/VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3'
PASSES="30:1 $P !/GND;30:1 /ADC_IR* /IR_* !/GND;30:1 * !/GND"
i=0
echo "$PASSES" | tr ';' '\n' > "$W/passes.txt"
while read -r costs nets; do
  cp "$H/single.kicad_pro" "$W/p$i.kicad_pro"
  # shellcheck disable=SC2086
  python3 -X utf8 route.py "$W/p$i.kicad_pcb" "$W/p$((i+1)).kicad_pcb" --nets $nets \
      $COMMON --keep-input-copper --layers F.Cu B.Cu --layer-costs ${costs%:*} ${costs#*:} \
      --power-nets $PWR --power-nets-widths $PWW > "$W/pass$((i+1)).log" 2>&1
  grep -h 'Single-ended:' "$W/pass$((i+1)).log" | tail -1
  i=$((i+1))
done < "$W/passes.txt"
cp "$W/p$i.kicad_pcb" "$H/single.kicad_pcb"
python3 "$BASE/gen/tidy_tracks.py" "$H/single.kicad_pcb" 2>&1 | grep 'runs kept' || true
python3 "$BASE/gen/center_vias.py" "$H/single.kicad_pcb" 2>&1 | grep -E 'dropped|moved' || true
python3 "$G/jumpers.py" "$H/single.kicad_pcb" 2>&1 | grep -E 'jumper|kept' || true
python3 "$G/gnd_bridges.py" "$H/single.kicad_pcb" 2>&1 | grep -E 'GND pieces' || true
KRT="$KRT" python3 "$G/prune.py" "$H/single.kicad_pcb" 2>&1 | grep -E 'carrying nothing' || true
rm -rf "$BASE/gen/__pycache__" "$G/__pycache__"
(cd "$KRT/py_placer" && python3 -X utf8 beautify_labels.py "$H/single.kicad_pcb" "$W/labeled.kicad_pcb" \
    --refs C2 R1 R2 H2 'JP*' > "$W/labels.log" 2>&1)
cp "$W/labeled.kicad_pcb" "$H/single.kicad_pcb"
kicad-cli pcb drc --refill-zones --save-board --schematic-parity --format json \
    -o "$W/drc.json" "$H/single.kicad_pcb" | grep Found
echo "logs and the previous board in $W"
