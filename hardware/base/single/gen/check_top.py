"""Checks for a board made without plated holes (single.kicad_pcb).

1. Top joints: plated-through pads that a top track (a wire jumper) reaches.
   Each would need a solder joint on the top side; a pin in BLOCKED must have
   none.
2. Jumper holes under parts: under a part that sits flat on top the wire has
   no room; under a part on the bottom (caster, IR bar) the solder joint of
   the hole is in its way and has to be filed flat.
3. Drilling margin: copper ring left around each hole. A hand drill wanders,
   so a thin ring tears off easily.

usage: check_top.py BOARD.kicad_pcb
"""
import collections
import sys

import pcbnew

# ref -> pad numbers no top copper may reach (None: every plated pad). The DRV
# wire pads (5-8) are left out: their wires are soldered on top anyway.
BLOCKED = {
    "U1": None, "U2": {"1", "2", "3", "4"}, "U3": {"1", "2", "3", "4"},
    "U4": None, "J3": None, "J4": None, "J5": None, "J6": None, "J7": None,
    "J8": None, "J9": None, "J10": None, "SW1": None, "SW2": None,
}

# Bottom parts whose body sits on the board: copper bumps under them matter.
BOTTOM_BODIES = ("H2", "H3")


def courtyard(fp):
    layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    return fp.GetCourtyard(layer)


def main(path):
    board = pcbnew.LoadBoard(path)
    mm = pcbnew.ToMM
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]

    top, bad = [], []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH or not pad.GetNetname():
                continue
            shape = pad.GetEffectivePolygon(pcbnew.F_Cu, pcbnew.ERROR_INSIDE)
            if any(t.GetLayer() == pcbnew.F_Cu and t.GetNetCode() == pad.GetNetCode() and
                   (shape.Collide(t.GetStart(), t.GetWidth() // 2) or
                    shape.Collide(t.GetEnd(), t.GetWidth() // 2)) for t in tracks):
                ref = fp.GetReference()
                name = f"{ref}.{pad.GetNumber()} {pad.GetNetname().lstrip('/')}"
                top.append(name)
                blocked = BLOCKED.get(ref, ())
                if ref in BLOCKED and (blocked is None or pad.GetNumber() in blocked):
                    bad.append(name)
    print(f"top joints ({len(top)}): {', '.join(top)}")
    print(f"  on pins that cannot be soldered on top: {bad or 'none'}")

    under = collections.defaultdict(list)
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if not ((ref in BLOCKED and not fp.IsFlipped()) or ref in BOTTOM_BODIES):
            continue
        area = courtyard(fp)
        if area.OutlineCount() == 0:
            continue
        for v in vias:
            if area.Contains(v.GetPosition()):
                under[ref].append(tuple(round(c, 1) for c in mm(v.GetPosition())))
    print(f"vias under part bodies ({sum(len(v) for v in under.values())} of {len(vias)}):")
    for ref, spots in sorted(under.items()):
        print(f"  {ref}: {len(spots)} {spots}")

    rings = collections.Counter()
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            size, drill = pad.GetSize(pcbnew.F_Cu), pad.GetDrillSize()
            ring = min(size.x - drill.x, size.y - drill.y) / 2
            rings[(round(mm(ring), 2), fp.GetFPID().GetLibItemName().wx_str()[:30])] += 1
    for v in vias:
        rings[(round(mm(v.GetWidth(pcbnew.F_Cu) - v.GetDrillValue()) / 2, 2), "via")] += 1
    print("thinnest copper rings around holes (mm, part, count):")
    for (ring, part), n in sorted(rings.items())[:5]:
        print(f"  {ring:.2f}  {part}  {n}")


if __name__ == "__main__":
    main(sys.argv[1])
