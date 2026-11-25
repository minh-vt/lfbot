"""Plot a board 1:1 onto the middle of its page, with 100 mm scale bars.

kicad-cli plots at scale 1 from the page corner, and these boards sit around
the axle at (0, 0), so most of the board falls off the page. This copies the
board, shifts the copy to the page centre, draws a 100 mm bar with its label
on Cmts.User below the board, and plots the given layers plus that bar.
Measure the bar on the print: if it is not 100 mm, the printer scaled it.

--copies COLSxROWS lays that many copies side by side on one page (test
prints, spare films). The page turns portrait when the grid fits only that
way, and a second bar runs up the side: a printer can scale the two
directions differently.

usage: plot_1to1.py BOARD.kicad_pcb OUT.pdf LAYER,LAYER,... [--copies CxR]
                    [kicad-cli pdf options]
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

import pcbnew

MM = pcbnew.FromMM
BAR = MM(100)
GAP = MM(6)       # between copies: room to cut them apart
MARGIN = MM(10)   # printers leave about 5 mm blank at each edge
SIZES_MM = {"A4": (297, 210), "A3": (420, 297), "A5": (210, 148)}


def page_size(path):
    """Landscape unless the file says portrait (the Python binding hides it)."""
    m = re.search(r'\(paper "([^"]+)"( portrait)?\)', open(path).read())
    w, h = SIZES_MM.get(m.group(1) if m else "A4", SIZES_MM["A4"])
    return (h, w) if m and m.group(2) else (w, h)


def duplicate(item):
    try:
        return item.Duplicate(False).Cast()
    except TypeError:  # tracks and drawings take no argument
        return item.Duplicate().Cast()


def bar(board, start, vertical, mirrored):
    """100 mm bar from `start` (right or up) with end ticks and a label."""
    x, y = start
    tick = MM(2)
    if vertical:
        lines = (((x, y), (x, y - BAR)), ((x - tick, y), (x + tick, y)),
                 ((x - tick, y - BAR), (x + tick, y - BAR)))
        at, angle = (x + MM(4), y - BAR // 2), 90
    else:
        lines = (((x, y), (x + BAR, y)), ((x, y - tick), (x, y + tick)),
                 ((x + BAR, y - tick), (x + BAR, y + tick)))
        at, angle = (x + BAR // 2, y + MM(4)), 0
    for a, b in lines:
        line = pcbnew.PCB_SHAPE(board)
        line.SetShape(pcbnew.SHAPE_T_SEGMENT)
        line.SetStart(pcbnew.VECTOR2I(*a))
        line.SetEnd(pcbnew.VECTOR2I(*b))
        line.SetLayer(pcbnew.Cmts_User)
        line.SetWidth(MM(0.3))
        board.Add(line)
    label = pcbnew.PCB_TEXT(board)
    label.SetText("100 mm")
    label.SetLayer(pcbnew.Cmts_User)
    label.SetTextSize(pcbnew.VECTOR2I(MM(2), MM(2)))
    label.SetTextThickness(MM(0.3))
    label.SetTextAngleDegrees(angle)
    label.SetPosition(pcbnew.VECTOR2I(*at))
    label.SetMirrored(mirrored)  # reads correctly once the plot mirrors it
    board.Add(label)


def main(board_path, out, layers, extra):
    cols = rows = 1
    if "--copies" in extra:
        i = extra.index("--copies")
        cols, rows = (int(v) for v in extra[i + 1].lower().split("x"))
        extra = extra[:i] + extra[i + 2:]
    mirrored = "--mirror" in extra or "-m" in extra

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
    pitch_x, pitch_y = box.GetWidth() + GAP, box.GetHeight() + GAP
    grid_w, grid_h = cols * pitch_x - GAP, rows * pitch_y - GAP
    page_w, page_h = (MM(v) for v in page_size(board_path))
    portrait = False
    if cols * rows > 1:
        # room for the bars: one below the grid, one beside it
        need_w, need_h = grid_w + 2 * MARGIN + MM(16), grid_h + 2 * MARGIN + MM(16)
        if need_w > page_w or need_h > page_h:
            page_w, page_h = min(page_w, page_h), max(page_w, page_h)
            portrait = True
        if need_w > page_w or need_h > page_h:
            sys.exit(f"{cols}x{rows} copies do not fit on the page")

    left = page_w // 2 - grid_w // 2
    top = page_h // 2 - grid_h // 2
    items = (list(board.GetFootprints()) + list(board.GetTracks()) +
             list(board.GetDrawings()) + list(board.Zones()))
    copies = {(0, 0): items}
    for r in range(rows):
        for c in range(cols):
            if (r, c) != (0, 0):
                copies[(r, c)] = [duplicate(i) for i in items]  # before anything moves
    for (r, c), group in copies.items():
        shift = pcbnew.VECTOR2I(left + c * pitch_x - box.GetLeft(),
                                top + r * pitch_y - box.GetTop())
        for item in group:
            if (r, c) != (0, 0):
                board.Add(item)
            item.Move(shift)

    bar(board, (page_w // 2 - BAR // 2, top + grid_h + MM(8)), False, mirrored)
    if cols * rows > 1:
        bar(board, (left + grid_w + MM(6), page_h // 2 + BAR // 2), True, mirrored)

    moved = os.path.join(work, name + ".kicad_pcb")
    pcbnew.SaveBoard(moved, board)
    if portrait:
        text = open(moved).read()
        open(moved, "w").write(re.sub(r'\(paper "([^"]+)"\)', r'(paper "\1" portrait)', text, count=1))
    subprocess.run(["kicad-cli", "pcb", "export", "pdf", "--mode-single", "--scale", "1",
                    "-l", layers + ",User.Comments", "-o", os.path.abspath(out), *extra, moved],
                   check=True, capture_output=True)
    shutil.rmtree(work)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
