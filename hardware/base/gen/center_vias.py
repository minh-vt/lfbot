"""Finish a routed board made without plated holes: no copper twice, no
floating pour, every track end on a via centre.

1. Drops exact duplicate segments (the router can lay one twice) and sets
   every copper pour to drop islands that connect to nothing.
2. For a via whose tracks end inside its copper but off its centre, moves the
   via onto that end point (with the tracks that sat on the old centre). If
   that breaks a rule, keeps the via and adds a short stub from the end to the
   centre instead.

Each change is kept only if KiCad DRC shows no new violation.

usage: center_vias.py BOARD.kicad_pcb   (needs BOARD.kicad_pro beside it)
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile

import pcbnew

GRAVE = []  # removed items stay referenced: pcbnew frees them twice otherwise


def drc(path, ignore=("silk_", "text_", "track_not_centered")):
    """Violation keys, less the kinds in `ignore` (silkscreen is placed later;
    an off-centre end is what this step fixes). An open connection is keyed
    by its nets: KiCad reports it with whichever items it picks as its ends,
    and those change when unrelated copper moves."""
    out = tempfile.mktemp(suffix=".json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json",
                    "-o", out, path], check=True, capture_output=True)
    data = json.load(open(out))
    os.remove(out)
    keys = set()
    for v in data["violations"] + data["unconnected_items"]:
        kind = "unconnected_items" if v in data["unconnected_items"] else v["type"]
        if kind.startswith(ignore):
            continue
        if kind == "unconnected_items":
            nets = {m for i in v["items"] for m in re.findall(r"\[([^\]]+)\]", i["description"])}
            keys.add((kind, tuple(sorted(nets))))
        else:
            keys.add((kind, tuple(sorted(i.get("uuid", "") for i in v["items"]))))
    return keys


def drop_duplicates(board):
    seen, n = set(), 0
    for t in [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]:
        ends = tuple(sorted(((t.GetStart().x, t.GetStart().y), (t.GetEnd().x, t.GetEnd().y))))
        k = (t.GetLayer(), t.GetNetCode(), t.GetWidth(), ends)
        if k in seen:
            board.Remove(t)
            GRAVE.append(t)
            n += 1
        seen.add(k)
    return n


def main(path):
    board = pcbnew.LoadBoard(path)
    print("duplicate segments dropped:", drop_duplicates(board))
    for zone in board.Zones():
        if not zone.GetIsRuleArea():
            zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    pcbnew.SaveBoard(path, board)
    baseline = drc(path)
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    moved = stubbed = 0
    for via in [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]:
        c, r = via.GetPosition(), via.GetWidth(pcbnew.F_Cu) / 2
        on_centre, off = [], []
        for t in tracks:
            if t.GetNetCode() != via.GetNetCode():
                continue
            for get, put in ((t.GetStart, t.SetStart), (t.GetEnd, t.SetEnd)):
                e = get()
                if e == c:
                    on_centre.append(put)
                elif math.dist((e.x, e.y), (c.x, c.y)) <= r + 1000:  # 1 um: ends on the rim count
                    off.append((t, pcbnew.VECTOR2I(e)))
        if len({(p.x, p.y) for _, p in off}) != 1:
            continue
        end = off[0][1]
        via.SetPosition(end)
        for put in on_centre:
            put(end)
        pcbnew.SaveBoard(path, board)
        if not drc(path) - baseline:
            moved += 1
            continue
        via.SetPosition(c)
        for put in on_centre:
            put(c)
        t = off[0][0]
        if any(o.GetLayer() == t.GetLayer() and o.GetNetCode() == t.GetNetCode() and
               {(o.GetStart().x, o.GetStart().y), (o.GetEnd().x, o.GetEnd().y)} ==
               {(end.x, end.y), (c.x, c.y)} for o in tracks):
            continue  # that stub is already there
        stub = pcbnew.PCB_TRACK(board)
        stub.SetStart(end)
        stub.SetEnd(c)
        stub.SetWidth(t.GetWidth())
        stub.SetLayer(t.GetLayer())
        stub.SetNet(t.GetNet())
        board.Add(stub)
        pcbnew.SaveBoard(path, board)
        if not drc(path) - baseline:
            stubbed += 1
            continue
        board.Remove(stub)
        GRAVE.append(stub)
        pcbnew.SaveBoard(path, board)
    print("vias moved onto track ends:", moved, "stubs added:", stubbed)
    print("duplicate segments dropped:", drop_duplicates(board))
    pcbnew.SaveBoard(path, board)


if __name__ == "__main__":
    main(sys.argv[1])
