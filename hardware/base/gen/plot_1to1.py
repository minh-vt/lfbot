"""Plot a board 1:1 onto the middle of its page, with a 100 mm scale bar.

kicad-cli plots at scale 1 from the page corner, and these boards sit around
the axle at (0, 0), so most of the board falls off the page. This copies the
board, shifts the copy to the page centre, draws a 100 mm bar with its label
on Cmts.User below the board, and plots the given layers plus that bar.
Measure the bar on the print: if it is not 100 mm, the printer scaled it.

usage: plot_1to1.py BOARD.kicad_pcb OUT.pdf LAYER,LAYER,... [kicad-cli pdf options]
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

import pcbnew

BAR = pcbnew.FromMM(100)


SIZES_MM = {"A4": (297, 210), "A3": (420, 297), "A5": (210, 148)}


def page_size(path):
    """Landscape unless the file says portrait (the Python binding hides it)."""
    m = re.search(r'\(paper "([^"]+)"( portrait)?\)', open(path).read())
    w, h = SIZES_MM.get(m.group(1) if m else "A4", SIZES_MM["A4"])
    return (h, w) if m and m.group(2) else (w, h)


def main(board_path, out, layers, extra):
    src = os.path.dirname(os.path.abspath(board_path))
    name = os.path.splitext(os.path.basename(board_path))[0]
    work = tempfile.mkdtemp()
    for ext in (".kicad_pro", ".kicad_sch"):
        if os.path.exists(os.path.join(src, name + ext)):
            shutil.copy(os.path.join(src, name + ext), work)
    table = open(os.path.join(src, "fp-lib-table")).read().replace("${KIPRJMOD}", src)
    open(os.path.join(work, "fp-lib-table"), "w").write(table)

    board = pcbnew.LoadBoard(board_path)
    box = board.GetBoardEdgesBoundingBox()
    page_w, page_h = (pcbnew.FromMM(v) for v in page_size(board_path))
    shift = pcbnew.VECTOR2I(int(page_w / 2 - box.GetCenter().x),
                            int(page_h / 2 - box.GetCenter().y))
    items = (list(board.GetFootprints()) + list(board.GetTracks()) +
             list(board.GetDrawings()) + list(board.Zones()))
    for item in items:
        item.Move(shift)

    y = box.GetBottom() + shift.y + pcbnew.FromMM(8)
    x0 = int(page_w / 2 - BAR / 2)
    for a, b in (((x0, y), (x0 + BAR, y)), ((x0, y - pcbnew.FromMM(2)), (x0, y + pcbnew.FromMM(2))),
                 ((x0 + BAR, y - pcbnew.FromMM(2)), (x0 + BAR, y + pcbnew.FromMM(2)))):
        line = pcbnew.PCB_SHAPE(board)
        line.SetShape(pcbnew.SHAPE_T_SEGMENT)
        line.SetStart(pcbnew.VECTOR2I(*a))
        line.SetEnd(pcbnew.VECTOR2I(*b))
        line.SetLayer(pcbnew.Cmts_User)
        line.SetWidth(pcbnew.FromMM(0.3))
        board.Add(line)
    label = pcbnew.PCB_TEXT(board)
    label.SetText("100 mm")
    label.SetLayer(pcbnew.Cmts_User)
    label.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(2), pcbnew.FromMM(2)))
    label.SetTextThickness(pcbnew.FromMM(0.3))
    label.SetPosition(pcbnew.VECTOR2I(int(page_w / 2), int(y + pcbnew.FromMM(4))))
    if "--mirror" in extra or "-m" in extra:
        label.SetMirrored(True)  # reads correctly once the plot mirrors it
    board.Add(label)

    moved = os.path.join(work, name + ".kicad_pcb")
    pcbnew.SaveBoard(moved, board)
    subprocess.run(["kicad-cli", "pcb", "export", "pdf", "--mode-single", "--scale", "1",
                    "-l", layers + ",User.Comments", "-o", os.path.abspath(out), *extra, moved],
                   check=True, capture_output=True)
    shutil.rmtree(work)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
