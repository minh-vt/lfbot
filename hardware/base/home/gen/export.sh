#!/bin/sh
# Export everything needed to make home.kicad_pcb at home into
# release/<version>: 1:1 etch films (positive and negative), the top
# silkscreen for toner transfer, assembly guides, drill files and a Gerber
# zip for a CNC milling shop. Every 1:1 page has a 100 mm bar to check the
# printer scale.
#
# usage: export.sh [version]   (default v0.7)
set -e
G=$(cd "$(dirname "$0")" && pwd); H=$(dirname "$G"); BASE=$(dirname "$H")
V=${1:-v0.7}; R="$H/release/$V"; B="$H/home.kicad_pcb"
PL="python3 $BASE/gen/plot_1to1.py"
rm -rf "$R/phim" "$R/khoan" "$R"/so_do_lap_*.pdf "$R"/*.zip
mkdir -p "$R/phim" "$R/khoan"
FILM="--black-and-white --drill-shape-opt 1"
# Top film is mirrored and the bottom one is not: both then go printed side
# against the copper.
$PL "$B" "$R/phim/1_dong_mat_tren_lat_guong.pdf" F.Cu,Edge.Cuts $FILM --mirror
$PL "$B" "$R/phim/2_dong_mat_duoi.pdf" B.Cu,Edge.Cuts $FILM
$PL "$B" "$R/phim/1_dong_mat_tren_lat_guong_AM-BAN.pdf" F.Cu,Edge.Cuts $FILM --mirror -n
$PL "$B" "$R/phim/2_dong_mat_duoi_AM-BAN.pdf" B.Cu,Edge.Cuts $FILM -n
$PL "$B" "$R/phim/3_chu_mat_tren_ui_lat_guong.pdf" F.Silkscreen,Edge.Cuts \
    --black-and-white --subtract-soldermask --mirror
$PL "$B" "$R/so_do_lap_mat_tren.pdf" F.Fab,F.Silkscreen,Edge.Cuts --black-and-white --sp
$PL "$B" "$R/so_do_lap_mat_duoi.pdf" B.Fab,B.Silkscreen,B.Cu,Edge.Cuts \
    --black-and-white --sp --mirror
kicad-cli pcb export drill --format excellon --excellon-units mm \
    --excellon-zeros-format decimal --excellon-oval-format alternate --excellon-separate-th \
    --generate-map --map-format pdf -o "$R/khoan/" "$B" >/dev/null
T=$(mktemp -d)
kicad-cli pcb export gerbers -l F.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,Edge.Cuts \
    --subtract-soldermask -o "$T/" "$B" >/dev/null
cp "$R"/khoan/*.drl "$T/"
python3 - "$T" "$R/LF-Bot_base-home_${V}_gerber.zip" <<'EOF'
import os, sys, zipfile
src, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for name in sorted(os.listdir(src)):
        z.write(os.path.join(src, name), arcname=name)
EOF
rm -rf "$T" "$BASE/gen/__pycache__"
ls -R "$R"
