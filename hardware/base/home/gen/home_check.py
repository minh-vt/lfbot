"""Checks that matter only when the board is made without plated holes.

1. Top joints: plated-through pads that carry top copper. Each needs a
   solder joint on the top side; a blocked pin (prepare.BLOCKED) must have none.
2. Vias under parts: a via needs solder on both sides. Under a part that sits
   flat on top (socket, JST, button, IMU, buck) the top joint is covered once
   the part is fitted; under a part on the bottom (caster, IR bar) the joint
   is in its way.
3. Drilling margin: copper ring left around each hole. A hand drill wanders,
   so a thin ring tears off easily.

usage: home_check.py BOARD.kicad_pcb
"""
import collections
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prepare import BLOCKED  # noqa: E402

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
    pours = [z for z in board.Zones() if not z.GetIsRuleArea() and z.IsOnLayer(pcbnew.F_Cu)]

    top, bad = [], []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH or not pad.GetNetname():
                continue
            shape = pad.GetEffectivePolygon(pcbnew.F_Cu, pcbnew.ERROR_INSIDE)
            outline = shape.Outline(0)
            by_track = any(t.GetLayer() == pcbnew.F_Cu and t.GetNetCode() == pad.GetNetCode() and
                           (shape.Collide(t.GetStart(), t.GetWidth() // 2) or
                            shape.Collide(t.GetEnd(), t.GetWidth() // 2)) for t in tracks)
            by_pour = any(z.GetNetCode() == pad.GetNetCode() and any(
                z.GetFilledPolysList(pcbnew.F_Cu).Collide(outline.CPoint(i), pcbnew.FromMM(0.05))
                for i in range(outline.PointCount())) for z in pours)
            if by_track or by_pour:
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
