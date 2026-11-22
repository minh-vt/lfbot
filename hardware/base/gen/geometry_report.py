"""Track geometry report: how clean the copper is, in counts.

Board-wide bend angles (45, 90, other), off-angle segments, pieces shorter
than 0.2 mm and straight runs cut into several segments. A 90 degree corner
inside a pad is not counted: the pad hides it. Per-net detour and vias come
from KiCadRoutingTools' route_review.py, which also shows congested areas;
this only adds what it does not measure.

usage: geometry_report.py BOARD.kicad_pcb
"""
import collections
import math
import sys

import pcbnew

TOL = 200  # nm


def main(path):
    board = pcbnew.LoadBoard(path)
    mm = pcbnew.ToMM
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    via_at = {(v.GetPosition().x, v.GetPosition().y) for v in vias}
    pads = [p for f in board.GetFootprints() for p in f.Pads()]

    node = collections.defaultdict(list)
    for t in tracks:
        for e in (t.GetStart(), t.GetEnd()):
            node[(e.x, e.y, t.GetLayer(), t.GetNetCode())].append(t)
    kinds = collections.Counter()
    for (x, y, layer, _), ts in node.items():
        if len(ts) != 2 or (x, y) in via_at:
            continue
        here = pcbnew.VECTOR2I(x, y)
        dirs = []
        for t in ts:
            a, c = (t.GetStart(), t.GetEnd()) if t.GetStart() == here else (t.GetEnd(), t.GetStart())
            dirs.append((c.x - a.x, c.y - a.y))
        (ux, uy), (vx, vy) = dirs
        nu, nv = math.hypot(ux, uy), math.hypot(vx, vy)
        if not nu or not nv:
            continue
        bend = 180 - math.degrees(math.acos(max(-1, min(1, (ux * vx + uy * vy) / (nu * nv)))))
        if bend < 0.5:
            kinds["straight run cut in two"] += 1
            continue
        in_pad = any(p.GetNetCode() == ts[0].GetNetCode() and p.IsOnLayer(layer) and p.HitTest(here)
                     for p in pads)
        if abs(bend - 45) < 0.5:
            kinds["45 degree bend"] += 1
        elif abs(bend - 90) < 0.5:
            kinds["90 degree corner (in a pad)" if in_pad else "90 degree corner"] += 1
        else:
            kinds["other bend angle"] += 1
    for t in tracks:
        dx, dy = t.GetEnd().x - t.GetStart().x, t.GetEnd().y - t.GetStart().y
        if not (abs(dx) <= TOL or abs(dy) <= TOL or abs(abs(dx) - abs(dy)) <= TOL):
            kinds["off-angle segment"] += 1
        if t.GetLength() < pcbnew.FromMM(0.2):
            kinds["piece under 0.2 mm"] += 1

    print(f"{path}")
    print(f"  segments {len(tracks)}, vias {len(vias)}")
    for k, v in sorted(kinds.items()):
        print(f"  {k}: {v}")

if __name__ == "__main__":
    main(sys.argv[1])
