#!/bin/sh
# Rip up and re-route some nets of base.kicad_pcb with KiCadRoutingTools
# (KiCad PCM plugin com.github.drandyhaas.kicadroutingtools); every other
# track, via and the GND pours stay as they are.
#
# usage: route.sh "COST_F:COST_B NET ..." ["COST_F:COST_B NET ..." ...]
#   One argument per routing pass, routed in the order given; NET may be a
#   pattern ("/ADC_IR*"). COST_F:COST_B weights the two layers (1:10 keeps a
#   pass on F.Cu). Copper of every listed net is stripped first.
#
# Example, the front zone as routed for v0.7: the ToF lines first so they
# take the gaps between the J3 pins, then 3V3/5V, then the IR channels:
#   route.sh "1:1 /I2C_SDA /I2C_SCL /TOF_XSHUT" "10:1 /+3V3 /+5V" \
#            "1:10 /ADC_IR* /IR_*"
#
# The router never goes below the board's rules and the project file is left
# alone; vias stay off SMD pads (hand soldering). Afterwards tidy_tracks.py
# cleans the geometry and KiCad DRC grades the result. The previous board is
# kept in the log folder printed at the end.
set -e
[ $# -gt 0 ] || { sed -n '2,13p' "$0"; exit 2; }
G=$(cd "$(dirname "$0")" && pwd); B=$(dirname "$G")
KRT=${KRT:-$HOME/.local/share/kicad/10.0/3rdparty/plugins/com_github_drandyhaas_kicadroutingtools}
W=$(mktemp -d)
COMMON="--same-net-pad-clearance 0.2 --fab-tier standard --escalation board --no-fix-drc-settings --keep-input-copper"
PWR='/VBAT /VBAT_F /VBAT_SW /MOT_?_A /MOT_?_B /+5V* /+3V3 /GND'
PWW='0.8 0.8 0.6 0.8 0.8 0.5 0.5 0.5'
cp "$B/base.kicad_pcb" "$W/before.kicad_pcb"
ALL=""
for pass in "$@"; do ALL="$ALL ${pass#* }"; done
# shellcheck disable=SC2086
python3 "$G/strip_copper.py" "$B/base.kicad_pcb" "$W/p0.kicad_pcb" $ALL
i=0
for pass in "$@"; do
  costs=${pass%% *}; nets=${pass#* }
  cp "$B/base.kicad_pro" "$W/p$i.kicad_pro"   # KRT reads rules from the matching .kicad_pro
  # shellcheck disable=SC2086
  (cd "$KRT/py_router" && python3 -X utf8 route.py "$W/p$i.kicad_pcb" "$W/p$((i+1)).kicad_pcb" \
      --nets $nets $COMMON --layers F.Cu B.Cu --layer-costs ${costs%:*} ${costs#*:} \
      --power-nets $PWR --power-nets-widths $PWW > "$W/pass$((i+1)).log" 2>&1)
  grep -h 'Single-ended:' "$W/pass$((i+1)).log" | tail -1
  i=$((i+1))
done
cp "$W/p$i.kicad_pcb" "$B/base.kicad_pcb"
python3 "$G/tidy_tracks.py" "$B/base.kicad_pcb" 2>&1 | grep 'runs kept'
rm -rf "$G/__pycache__"
kicad-cli pcb drc --refill-zones --save-board --schematic-parity --format json \
    -o "$W/drc.json" "$B/base.kicad_pcb" | grep Found
echo "logs and the previous board in $W"
