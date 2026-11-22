"""Remove copper that carries nothing: a track piece whose removal keeps every
pad of its net connected and leaves no loose end.

Two kinds are tried:
- what KiCadRoutingTools' check_weird calls removable (the model the
  router's own cleanup removes by): pieces the steps after the router (via
  centring, jumpers, GND bridges) leave, such as a loop closed by a 0 ohm
  part;
- a track straight from one pad to another of the same part. The router
  does not know which pins a part joins inside (prepare.py marks them as
  KiCad jumper pads), so it links them in copper too; KiCad does know.
One piece at a time, since removing one can make another needed. Each
removal is kept only if KiCad DRC shows no new violation and no new open
connection.

usage: prune.py BOARD.kicad_pcb   (needs BOARD.kicad_pro beside it; KRT
names the KiCadRoutingTools folder when it is not the KiCad plugin one)
"""
import os
import re
import sys

import pcbnew

KRT = os.environ.get("KRT", os.path.expanduser(
    "~/.local/share/kicad/10.0/3rdparty/plugins/com_github_drandyhaas_kicadroutingtools"))
sys.path.insert(0, os.path.join(KRT, "py_router"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gen"))
from center_vias import GRAVE, drc  # noqa: E402
from check_weird import check_weird  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

ENDS = re.compile(r"\(([-\d.]+), ([-\d.]+)\)-\(([-\d.]+), ([-\d.]+)\)")
NEAR = 1000  # nm
IGNORE = ("silk_", "text_")  # labels are placed after this step


def removable(path):
    """(net, layer, end, end) of each piece check_weird would remove."""
    found, _ = check_weird(parse_kicad_pcb(path), tolerance=0)
    out = []
    for f in found:
        if f["category"] == "removable-segment":
            x0, y0, x1, y1 = map(float, ENDS.search(f["detail"]).groups())
            out.append((f["net"], f["layer"], (x0, y0), (x1, y1)))
    return out


def track_at(board, net, layer, a, b):
    ends = {tuple(pcbnew.FromMM(v) for v in a), tuple(pcbnew.FromMM(v) for v in b)}

    def near(p, q):
        return abs(p[0] - q[0]) <= NEAR and abs(p[1] - q[1]) <= NEAR

    for t in board.GetTracks():
        if (t.GetClass() != "PCB_TRACK" or t.IsLocked() or t.GetNetname() != net
                or t.GetLayerName() != layer):
            continue
        s, e = (t.GetStart().x, t.GetStart().y), (t.GetEnd().x, t.GetEnd().y)
        if all(any(near(p, q) for q in ends) for p in (s, e)):
            return t
    return None


def pad_links(board):
    """(net, layer, end, end) of each track from a pad to another pad of
    the same part."""
    pads = [p for f in board.GetFootprints() for p in f.Pads() if p.GetNetCode()]
    out = []
    for t in board.GetTracks():
        if t.GetClass() != "PCB_TRACK" or t.IsLocked():
            continue
        hit = [[p for p in pads if p.GetNetCode() == t.GetNetCode() and
                p.IsOnLayer(t.GetLayer()) and p.HitTest(e)] for e in (t.GetStart(), t.GetEnd())]
        # SWIG hands out a new proxy per call: compare by reference and uuid
        if any(a.GetParentFootprint().GetReference() == b.GetParentFootprint().GetReference()
               and a.m_Uuid.AsString() != b.m_Uuid.AsString()
               for a in hit[0] for b in hit[1]):
            out.append((t.GetNetname(), t.GetLayerName(), pcbnew.ToMM(t.GetStart()),
                        pcbnew.ToMM(t.GetEnd())))
    return out


def main(path):
    board = pcbnew.LoadBoard(path)
    baseline = drc(path, IGNORE)
    tried, dropped = set(), []
    while True:
        todo = [r for r in removable(path) + pad_links(board) if r not in tried]
        if not todo:
            break
        piece = todo[0]
        tried.add(piece)
        t = track_at(board, *piece)
        if t is None:
            continue
        board.Remove(t)
        pcbnew.SaveBoard(path, board)
        if drc(path, IGNORE) - baseline:
            board.Add(t)
            pcbnew.SaveBoard(path, board)
            continue
        GRAVE.append(t)
        dropped.append(f"{piece[0]} {pcbnew.ToMM(t.GetLength()):.2f} mm")
    print(f"pieces carrying nothing removed: {len(dropped)}", dropped)


if __name__ == "__main__":
    main(sys.argv[1])
