"""Join GND pour pieces that tracks have cut off from the rest.

GND is poured on the bottom only and never routed, so a piece of pour that
tracks enclose (with GND pads in it) is left without a connection. For each
cut-off piece this looks for the shortest gap to another GND piece and
bridges it: a 0 ohm part when the gap is short enough for its pads to land
in the pour on both sides (0805, then 1206), otherwise a wire jumper (two
holes, a top track). The pour then flows into the new pads or holes. Each
bridge is kept only if KiCad DRC shows no new violation and fewer GND
pieces unconnected.

usage: gnd_bridges.py BOARD.kicad_pcb   (needs BOARD.kicad_pro beside it)
"""
import json
import math
import os
import subprocess
import sys
import tempfile

import pcbnew
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jumpers import GRAVE, LIB  # noqa: E402

GND = "GND"
MM = pcbnew.FromMM
# footprint, value, pad pitch, pad length along the part (mm)
PARTS = [("Jumper_0R_0805", "0R 0805", 2.0, 1.2), ("Jumper_0R_1206", "0R 1206", 3.1, 1.3)]
WIRE_MAX = 15.0   # longest wire jumper tried, mm
SAMPLE = 0.5      # spacing of the points tried along a piece's edge, mm
TRIES = 40        # closest point pairs tried per piece


def check(path):
    """(violation keys, number of unconnected GND items)."""
    out = tempfile.mktemp(suffix=".json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json",
                    "-o", out, path], check=True, capture_output=True)
    data = json.load(open(out))
    os.remove(out)
    keys = {(v["type"], tuple(sorted(i.get("uuid", "") for i in v["items"])))
            for v in data["violations"]
            if not v["type"].startswith(("silk_", "text_", "track_not_centered", "lib_footprint"))}
    gnd = sum(1 for v in data["unconnected_items"]
              if all(f"[{GND}]" in i["description"] for i in v["items"]))
    others = {tuple(sorted(i["description"] for i in v["items"]))
              for v in data["unconnected_items"]
              if not all(f"[{GND}]" in i["description"] for i in v["items"])}
    return keys, gnd, others


def pieces(board):
    """GND pour pieces as shapely polygons (mm), largest first."""
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    polys = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != GND:
            continue
        fill = z.GetFilledPolysList(pcbnew.B_Cu)
        for i in range(fill.OutlineCount()):
            ring = fill.Outline(i)
            pts = [(pcbnew.ToMM(ring.CPoint(k).x), pcbnew.ToMM(ring.CPoint(k).y))
                   for k in range(ring.PointCount())]
            holes = []
            for h in range(fill.HoleCount(i)):
                hr = fill.Hole(i, h)
                holes.append([(pcbnew.ToMM(hr.CPoint(k).x), pcbnew.ToMM(hr.CPoint(k).y))
                              for k in range(hr.PointCount())])
            polys.append(Polygon(pts, holes).buffer(0))
    return sorted(polys, key=lambda p: -p.area)


def candidates(piece, others):
    """Closest point pairs from this piece's edge to any other piece."""
    pairs = []
    edge = piece.exterior
    n = max(1, int(edge.length / SAMPLE))
    for k in range(n):
        p = edge.interpolate(k * SAMPLE)
        for j, other in enumerate(others):
            q = nearest_points(p, other)[1]
            d = p.distance(q)
            if 0.05 < d <= WIRE_MAX:
                pairs.append((d, (p.x, p.y), (q.x, q.y), j))
    pairs.sort()
    picked, seen = [], set()
    for d, p, q, j in pairs:
        key = (round(p[0]), round(p[1]), j)
        if key not in seen:
            seen.add(key)
            picked.append((d, p, q, j))
        if len(picked) >= TRIES:
            break
    return picked


def part_bridge(board, p, q, n):
    """0 ohm parts whose pads straddle the gap p-q, smallest first."""
    d = math.dist(p, q)
    ux, uy = (q[0] - p[0]) / d, (q[1] - p[1]) / d
    angle = math.degrees(math.atan2(-uy, ux))
    out = []
    for name, value, pitch, length in PARTS:
        if pitch < d + length:
            continue
        cx, cy = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        fp = pcbnew.FootprintLoad(LIB, name)
        fp.SetReference(f"JP{n}")
        fp.SetValue(value)
        board.Add(fp)
        pos = pcbnew.VECTOR2I(MM(cx), MM(cy))
        fp.SetPosition(pos)
        fp.Flip(pos, pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        fp.SetOrientationDegrees(angle)
        board.Remove(fp)
        for pad in fp.Pads():
            pad.SetNet(board.GetNetsByName()[GND])
        out.append([fp])
    return out


def wire_bridge(board, p, q, a, b):
    """Two holes inside the pieces either side of the gap and a top track."""
    d = math.dist(p, q)
    ux, uy = (q[0] - p[0]) / d, (q[1] - p[1]) / d
    ds = board.GetDesignSettings()
    r = pcbnew.ToMM(ds.GetCurrentViaSize()) / 2
    for inset in (r + 0.3, r + 0.8, r + 1.5):
        h1 = (p[0] - ux * inset, p[1] - uy * inset)
        h2 = (q[0] + ux * inset, q[1] + uy * inset)
        if a.buffer(-r).contains(Point(h1)) and b.buffer(-r).contains(Point(h2)):
            break
    else:
        return []
    net = board.GetNetsByName()[GND]
    items = []
    for x, y in (h1, h2):
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        via.SetWidth(ds.GetCurrentViaSize())
        via.SetDrill(ds.GetCurrentViaDrill())
        via.SetNet(net)
        items.append(via)
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(pcbnew.VECTOR2I(MM(h1[0]), MM(h1[1])))
    t.SetEnd(pcbnew.VECTOR2I(MM(h2[0]), MM(h2[1])))
    t.SetWidth(MM(0.3))
    t.SetLayer(pcbnew.F_Cu)
    t.SetNet(net)
    items.append(t)
    return [items]


def main(path):
    board = pcbnew.LoadBoard(path)
    keys, gnd, others = check(path)
    n = 1 + max([int(f.GetReference()[2:]) for f in board.GetFootprints()
                 if f.GetReference().startswith("JP") and f.GetReference()[2:].isdigit()] or [0])
    added = {"0 ohm": 0, "wire": 0}
    while gnd:
        polys = pieces(board)
        done = False
        # small pieces first: they are the cut-off ones
        for a in sorted(polys, key=lambda p: p.area):
            rest = [p for p in polys if p is not a]
            for d, p, q, j in candidates(a, rest):
                options = part_bridge(board, p, q, n) if d <= 1.8 else []
                options += wire_bridge(board, p, q, a, rest[j])
                for items in options:
                    for it in items:
                        board.Add(it)
                    pcbnew.SaveBoard(path, board)
                    new_keys, new_gnd, new_others = check(path)
                    if new_gnd < gnd and not (new_keys - keys) and not (new_others - others):
                        keys, gnd, others = new_keys, new_gnd, new_others
                        added["0 ohm" if len(items) == 1 else "wire"] += 1
                        n += len(items) == 1
                        done = True
                        break
                    for it in items:
                        board.Remove(it)
                    GRAVE.extend(items)
                    pcbnew.SaveBoard(path, board)
                if done:
                    break
            if done:
                break
        if not done:
            break
    print("GND pieces joined:", added, "| GND pieces still apart:", gnd)


if __name__ == "__main__":
    main(sys.argv[1])
