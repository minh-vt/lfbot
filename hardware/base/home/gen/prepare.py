"""Prepare the base placement for home etching.

1. Move the divider resistors and the battery-sense parts to the bottom, the
   side that is etched and soldered anyway: the IR header and the ESP32 pins
   then reach them without vias. D1 stays on top; on the bottom it would sit
   inside the caster.
2. Forbid top-side copper at pins that cannot be soldered from the top.
3. Keep vias out from under bodies that sit flat on the board: a via there
   cannot be soldered on top once the part is fitted (button, JST, header,
   buck).
4. Mark the board text as the home variant and move the resistor row
   labels to the bottom with the resistors.

A home-etched board has no plated holes, so a pin joins top copper only if it
is soldered on top. Parts whose body sits flat on the board (sockets, JST,
buttons, the flat IMU, the buck on short legs) leave no room for that, so
each of their pins gets a small F.Cu rule area: no tracks, no vias, no pour.
The router then has to reach those pins on B.Cu.

usage: prepare.py BOARD.kicad_pcb
"""
import sys

import pcbnew

# ref -> pad numbers that are blocked (None: every plated pad). The DRV
# wire pads (5-8) stay open: their wires are soldered on top anyway.
BLOCKED = {
    "U1": None, "U2": {"1", "2", "3", "4"}, "U3": {"1", "2", "3", "4"},
    "U4": None, "J3": None, "J4": None, "J5": None, "J6": None, "J7": None,
    "J8": None, "J9": None, "J10": None, "SW1": None, "SW2": None,
}
TO_BOTTOM = [f"R{i}" for i in (*range(10, 18), *range(20, 28))] + ["R1", "R2", "C2"]
MARGIN = pcbnew.FromMM(0.3)
NAME = "no-top-solder"
# Flat on the board; the ESP32 and the DRVs stand on sockets, so vias under
# them are soldered before the module is plugged in. The caster is left out:
# keeping vias from under its 24 mm body, mid-board, made the router fail or
# detour; the few vias there are filed flat (README). The IMU is left out
# too: it stands on its header spacer, and its GND pin may need a via next to
# it to reach the pour.
NO_VIA_UNDER = ["SW1", "SW2", "J3", "J4", "J5", "J7", "J8", "J9", "J10", "U4"]


def keep_vias_out(board):
    """Rule area with the footprint's courtyard outline: no vias."""
    for ref in NO_VIA_UNDER:
        fp = board.FindFootprintByReference(ref)
        area = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
        if area.OutlineCount() == 0:
            continue
        zone = pcbnew.ZONE(board)
        zone.SetIsRuleArea(True)
        zone.SetLayer(pcbnew.F_Cu)
        zone.SetDoNotAllowVias(True)
        zone.SetDoNotAllowTracks(False)
        zone.SetDoNotAllowZoneFills(False)
        zone.SetDoNotAllowPads(False)
        zone.SetDoNotAllowFootprints(False)
        zone.SetZoneName("no-via-under-" + ref)
        zone.Outline().AddOutline(area.Outline(0))
        board.Add(zone)
    print("bodies kept free of vias:", len(NO_VIA_UNDER))


def main(path):
    board = pcbnew.LoadBoard(path)
    for ref in TO_BOTTOM:
        fp = board.FindFootprintByReference(ref)
        if not fp.IsFlipped():
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    print("SMD parts moved to the bottom:", len(TO_BOTTOM))
    for item in board.GetDrawings():
        if item.GetClass() == "PCB_TEXT" and item.GetText().startswith("LF-Bot base"):
            item.SetText(item.GetText().replace("LF-Bot base ", "LF-Bot base-home "))
        elif item.GetClass() == "PCB_TEXT" and item.GetText() in ("10k", "47k"):
            item.SetLayer(pcbnew.B_SilkS)  # row labels follow the resistors
            item.SetMirrored(True)
    count = 0
    for ref, numbers in BLOCKED.items():
        for pad in board.FindFootprintByReference(ref).Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            if numbers is not None and pad.GetNumber() not in numbers:
                continue
            box = pad.GetBoundingBox()
            zone = pcbnew.ZONE(board)
            zone.SetIsRuleArea(True)
            zone.SetLayer(pcbnew.F_Cu)
            zone.SetDoNotAllowTracks(True)
            zone.SetDoNotAllowVias(True)
            zone.SetDoNotAllowZoneFills(True)
            zone.SetDoNotAllowPads(False)
            zone.SetDoNotAllowFootprints(False)
            zone.SetZoneName(NAME)
            outline = zone.Outline()
            outline.NewOutline()
            for x, y in ((box.GetLeft() - MARGIN, box.GetTop() - MARGIN),
                         (box.GetRight() + MARGIN, box.GetTop() - MARGIN),
                         (box.GetRight() + MARGIN, box.GetBottom() + MARGIN),
                         (box.GetLeft() - MARGIN, box.GetBottom() + MARGIN)):
                outline.Append(int(x), int(y))
            board.Add(zone)
            count += 1
    keep_vias_out(board)
    pcbnew.SaveBoard(path, board)
    print("pins kept off the top layer:", count)


if __name__ == "__main__":
    main(sys.argv[1])
