#!/bin/sh
# Export what is needed to make single.kicad_pcb at home into
# release/<version>: the 1:1 bottom-copper film (positive and negative), a
# 1:1 map of the wire jumpers on the top, the top silkscreen for toner
# transfer, assembly guides (also the paper fit check for the modules),
# drill files, a STEP model for mechanical CAD and a Gerber zip for a CNC
# milling shop. Every 1:1 page has a 100 mm bar to check the printer scale.
#
# usage: export.sh [version]   (default v0.7)
set -e
G=$(cd "$(dirname "$0")" && pwd); H=$(dirname "$G"); BASE=$(dirname "$H")
V=${1:-v0.7}; R="$H/release/$V"; B="$H/single.kicad_pcb"
PL="python3 $BASE/gen/plot_1to1.py"
rm -rf "$R/phim" "$R/khoan" "$R"/so_do_*.pdf "$R"/*.zip "$R"/*.step
mkdir -p "$R/phim" "$R/khoan"
# Bottom film goes printed side against the copper, so it is not mirrored.
$PL "$B" "$R/phim/dong_mat_duoi.pdf" B.Cu,Edge.Cuts --black-and-white --drill-shape-opt 1
$PL "$B" "$R/phim/dong_mat_duoi_AM-BAN.pdf" B.Cu,Edge.Cuts --black-and-white --drill-shape-opt 1 -n
$PL "$B" "$R/phim/chu_mat_tren_ui_lat_guong.pdf" F.Silkscreen,Edge.Cuts \
    --black-and-white --subtract-soldermask --mirror
# Top copper here is only wire jumpers: a map for laying them, never etched.
$PL "$B" "$R/so_do_day_nhay_mat_tren.pdf" F.Cu,F.Fab,Edge.Cuts --black-and-white --sp
$PL "$B" "$R/so_do_lap_mat_tren.pdf" F.Fab,F.Silkscreen,Edge.Cuts --black-and-white --sp
$PL "$B" "$R/so_do_lap_mat_duoi.pdf" B.Fab,B.Silkscreen,B.Cu,Edge.Cuts \
    --black-and-white --sp --mirror
kicad-cli pcb export step --subst-models -f -o "$R/LF-Bot_base-1side_${V}.step" "$B" >/dev/null 2>&1
kicad-cli pcb export drill --format excellon --excellon-units mm \
    --excellon-zeros-format decimal --excellon-oval-format alternate --excellon-separate-th \
    --generate-map --map-format pdf -o "$R/khoan/" "$B" >/dev/null
T=$(mktemp -d)
kicad-cli pcb export gerbers -l B.Cu,B.Mask,B.Silkscreen,F.Silkscreen,Edge.Cuts \
    --subtract-soldermask -o "$T/" "$B" >/dev/null
cp "$R"/khoan/*.drl "$T/"
python3 - "$T" "$R/LF-Bot_base-1side_${V}_gerber.zip" <<'EOF'
import os, sys, zipfile
src, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for name in sorted(os.listdir(src)):
        z.write(os.path.join(src, name), arcname=name)
EOF
rm -rf "$T" "$BASE/gen/__pycache__"
ls -R "$R"
