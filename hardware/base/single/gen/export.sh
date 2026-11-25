#!/bin/sh
# Export what is needed to make single.kicad_pcb at home into
# release/<version>: the 1:1 bottom-copper film (positive and negative), a
# 1:1 map of the wire jumpers on the top, the top silkscreen for toner
# transfer, assembly guides (also the paper fit check for the modules),
# drill files, a STEP model for mechanical CAD and a Gerber zip for a CNC
# milling shop. Every 1:1 page has a 100 mm bar to check the printer scale.
# Each film also comes 4 to an A4 page (*_4ban.pdf) for test prints and
# spares; those pages carry a second bar up the side.
#
# usage: export.sh [version]   (default v0.7)
set -e
G=$(cd "$(dirname "$0")" && pwd); H=$(dirname "$G"); BASE=$(dirname "$H")
V=${1:-v0.7}; R="$H/release/$V"; B="$H/single.kicad_pcb"
PL="python3 $BASE/gen/plot_1to1.py"
rm -rf "$R/phim" "$R/khoan" "$R"/so_do_*.pdf "$R"/*.zip "$R"/*.step
mkdir -p "$R/phim" "$R/khoan"
# Bottom film goes printed side against the copper, so it is not mirrored.
films() {  # films SUFFIX [plot_1to1 options]
  S=$1; shift
  $PL "$B" "$R/phim/dong_mat_duoi$S.pdf" B.Cu,Edge.Cuts "$@" --black-and-white --drill-shape-opt 1
  $PL "$B" "$R/phim/dong_mat_duoi_AM-BAN$S.pdf" B.Cu,Edge.Cuts "$@" --black-and-white \
      --drill-shape-opt 1 -n
  $PL "$B" "$R/phim/chu_mat_tren_ui_lat_guong$S.pdf" F.Silkscreen,Edge.Cuts "$@" \
      --black-and-white --subtract-soldermask --mirror
}
films ""
films _4ban --copies 2x2
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
# KiCad names the copper "L2, Bot" and the drills "1,2": the note tells the
# shop it is one copper layer on the bottom, every hole unplated.
python3 - "$T" "$R/LF-Bot_base-1side_${V}_gerber.zip" "$B" "$V" <<'EOF'
import collections, json, os, sys, zipfile
import pcbnew
src, out, board_path, version = sys.argv[1:5]
board = pcbnew.LoadBoard(board_path)
mm = pcbnew.ToMM
box = board.GetBoardEdgesBoundingBox()
rules = json.load(open(board_path[:-len("kicad_pcb")] + "kicad_pro"))["board"]["design_settings"]["rules"]
slots, ring = collections.Counter(), 99.0
for fp in board.GetFootprints():
    for pad in fp.Pads():
        d = pad.GetDrillSize()
        if d.x != d.y:
            slots[(round(mm(min(d.x, d.y)), 1), round(mm(max(d.x, d.y)), 1))] += 1
        elif d.x and pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            s = pad.GetSize(pcbnew.B_Cu)
            ring = min(ring, mm(min(s.x, s.y) - d.x) / 2)
slot_text = "; ".join(f"{n} khe {w:g} x {l:g} mm" for (w, l), n in sorted(slots.items()))
note = f"""LF-Bot base 1 mat {version}: ghi chu dat xuong / fabrication notes

- 1 lop dong, chi o MAT DUOI (single-B_Cu.gbl). Single-sided: copper on the
  BOTTOM only. Linh kien xuyen lo cam tu mat tren, han o mat duoi.
- Phip FR-4 1,6 mm, dong 1 oz. FR-4 1.6 mm, 1 oz copper.
- Kich thuoc {mm(box.GetWidth()):.1f} x {mm(box.GetHeight()):.1f} mm (single-Edge_Cuts.gm1).
- Phu xanh (solder mask) chi mat duoi; chu (silkscreen) ca mat tren va mat duoi.
- Moi lo deu KHONG ma (non-plated), ke ca file single-PTH.drl.
- Khe oval trong single-NPTH.drl (G85): {slot_text}.
- Duong nho nhat {rules['min_track_width']} mm, khe ho nho nhat {rules['min_clearance']} mm,
  lo nho nhat {rules['min_through_hole_diameter']} mm, vanh dong mong nhat {ring:.2f} mm,
  dong cach mep board {rules['min_copper_edge_clearance']} mm.
"""
open(os.path.join(src, "GHI_CHU_DAT_XUONG.txt"), "w").write(note)
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for name in sorted(os.listdir(src)):
        z.write(os.path.join(src, name), arcname=name)
EOF
rm -rf "$T" "$BASE/gen/__pycache__"
ls -R "$R"
