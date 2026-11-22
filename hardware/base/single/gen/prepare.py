"""Prepare the base placement for a single-sided board with jumpers.

Only the bottom copper is etched. The top layer in this project stands for
wire jumpers: a top track between two vias is a wire laid on the component
side through two holes, soldered underneath. So:

1. The divider resistors and the battery-sense parts go to the bottom, like
   the fab version keeps on top. D1 has to go too (the top has no copper).
2. No top copper at any pad: a jumper ends in its own hole, never in a pin.
3. No jumper wire or hole under a body that sits flat on the board.
4. The board text names the variant; the resistor row labels follow the
   resistors to the bottom assembly drawing.
5. Parts whose place is free move where their copper is shortest (MOVES):
   J4 and J10 onto the hand-planned bus (guide.py), D1 under the MP1584
   module, whose bottom side is free (only the module's pins come through),
   between its output pin and the ESP32 5V pin straight above, and the
   battery divider closes up around its ADC pin. The pins this
   variant changes take their nets
   from design.VARIANT_PINS, so the board matches single.kicad_sch.
6. Pins a part already joins inside are marked as joined (KiCad jumper pads),
   so each pair works as a free bridge: the two pins of each side of a 6 mm
   push button, the ESP32 DevKit's GND pins on both headers and its two 3V3
   pins, the two legs of each MP1584 terminal (both go through the one
   module pad) and the MP1584's input and output GND.

usage: prepare.py BOARD.kicad_pcb
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gen"))
from design import VARIANT_PINS  # noqa: E402

# Etched and soldered side: the IR dividers, battery sense parts and D1.
TO_BOTTOM = [f"R{i}" for i in (*range(10, 18), *range(20, 28))] + ["R1", "R2", "C2"]
# Bodies flat on the board (buttons, JST, headers, buck): a jumper hole under
# one cannot be soldered once the part is fitted. The ESP32 and the DRVs stand
# on sockets; the caster's few holes are filed flat (README).
NO_VIA_UNDER = ["SW1", "SW2", "J3", "J4", "J5", "J7", "J8", "J9", "J10", "U4"]
MARGIN = pcbnew.FromMM(0.4)
# ref: (footprint origin x, y in mm, angle), set after the flip to the
# bottom. J4 goes to the front strip beside J3 so IR_CTRL reaches it through
# the free channel of the divider block; J10 sits on the bus where it runs up
# the right side, SDA and SCL under pins 4 and 5. D1 (origin at its centre)
# stands in the column of the ESP32 5V pin (U1.21, x -26.92), cathode toward
# U1.21, anode 45 degrees from the nearer MP1584 output leg (U4.3). In the
# battery divider the two VBAT_SENSE ends of R1 and R2 face each other either
# side of U1.41 (the ADC pin straight below), R1's VBAT_SW end toward where
# VBAT_SW comes in, and C2, parallel to R2, sits pin for pin above it.
MOVES = {"J4": (-11.43, -88.7, 0), "J10": (18.8, -68.0, 90), "D1": (-26.92, -42.32, -90),
         "R1": (13.95, -38.61, 0), "R2": (18.53, -38.61, 0), "C2": (18.57, -41.2, 0)}
SAME_NUMBER_JOINED = ["SW1", "SW2", "U4"]
JOINED_INSIDE = {"U1": (("22", "23", "24", "44"), ("1", "2")), "U4": (("2", "4"),)}


def rule_area(board, name, outline_points=None, outline=None, tracks=True):
    zone = pcbnew.ZONE(board)
    zone.SetIsRuleArea(True)
    zone.SetLayer(pcbnew.F_Cu)
    zone.SetDoNotAllowTracks(tracks)
    zone.SetDoNotAllowVias(True)
    zone.SetDoNotAllowZoneFills(True)
    zone.SetDoNotAllowPads(False)
    zone.SetDoNotAllowFootprints(False)
    zone.SetZoneName(name)
    if outline is not None:
        zone.Outline().AddOutline(outline)
    else:
        zone.Outline().NewOutline()
        for x, y in outline_points:
            zone.Outline().Append(int(x), int(y))
    board.Add(zone)


def set_variant_pins(board):
    """Nets from design.VARIANT_PINS: a renamed net, or an open pin named the
    way KiCad names an unconnected connector pin."""
    nets = board.GetNetsByName()
    changed = []
    for ref, pins in VARIANT_PINS["single"].items():
        for pad in board.FindFootprintByReference(ref).Pads():
            num = int(pad.GetNumber())
            if num not in pins:
                continue
            if pins[num] is None:
                net = pcbnew.NETINFO_ITEM(board, f"unconnected-({ref}-Pin_{num}-Pad{num})")
                board.Add(net)
            else:
                net = nets["/" + pins[num]]
            pad.SetNet(net)
            changed.append(f"{ref}.{num}")
    return changed


def join_inside(path):
    """Write the jumper pad groups; pcbnew's Python API cannot set them."""
    text = open(path).read()
    blocks = text.split("\n\t(footprint ")
    for i, block in enumerate(blocks):
        for ref, pads in JOINED_INSIDE.items():
            if f'(property "Reference" "{ref}"' in block and "jumper_pad_groups" not in block:
                flag = next(f"\t\t(duplicate_pad_numbers_are_jumpers {v})\n"
                            for v in ("no", "yes")
                            if f"(duplicate_pad_numbers_are_jumpers {v})" in block)
                groups = " ".join("(" + " ".join(f'"{p}"' for p in g) + ")" for g in pads)
                blocks[i] = block.replace(flag, flag + f"\t\t(jumper_pad_groups {groups})\n", 1)
    open(path, "w").write("\n\t(footprint ".join(blocks))


def main(path):
    board = pcbnew.LoadBoard(path)
    for ref in SAME_NUMBER_JOINED:
        board.FindFootprintByReference(ref).SetDuplicatePadNumbersAreJumpers(True)
    changed = set_variant_pins(board)
    for ref in TO_BOTTOM + ["D1"]:
        fp = board.FindFootprintByReference(ref)
        if not fp.IsFlipped():
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    for ref, (x, y, angle) in MOVES.items():
        fp = board.FindFootprintByReference(ref)
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(angle)
        fp.BuildCourtyardCaches()
    pads = 0
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if not pad.IsOnLayer(pcbnew.F_Cu):
                continue
            box = pad.GetBoundingBox()
            rule_area(board, "no-top-copper", (
                (box.GetLeft() - MARGIN, box.GetTop() - MARGIN),
                (box.GetRight() + MARGIN, box.GetTop() - MARGIN),
                (box.GetRight() + MARGIN, box.GetBottom() + MARGIN),
                (box.GetLeft() - MARGIN, box.GetBottom() + MARGIN)))
            pads += 1
    for ref in NO_VIA_UNDER:
        fp = board.FindFootprintByReference(ref)
        area = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
        if area.OutlineCount():
            rule_area(board, "no-jumper-under-" + ref, outline=area.Outline(0))
    for item in board.GetDrawings():
        if item.GetClass() != "PCB_TEXT":
            continue
        if item.GetText().startswith("LF-Bot base "):
            item.SetText(item.GetText().replace("LF-Bot base ", "LF-Bot base-1side "))
        elif item.GetText() in ("10k", "47k"):
            # bottom silkscreen is never printed at home: the row labels go on
            # the assembly drawing, where jumpers next to them do not matter
            item.SetLayer(pcbnew.B_Fab)
            item.SetMirrored(True)
    pcbnew.SaveBoard(path, board)
    join_inside(path)
    print("SMD parts moved to the bottom:", len(TO_BOTTOM) + 1, "| moved:", list(MOVES),
          "| pins changed:", changed)
    print("pads kept free of top copper:", pads, "| bodies kept free of jumpers:", len(NO_VIA_UNDER))


if __name__ == "__main__":
    main(sys.argv[1])
