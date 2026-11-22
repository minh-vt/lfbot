"""Turn short top-side jumpers into 0 ohm 0805 or 1206 parts on the bottom.

On the single-sided board a top track between two vias is a wire jumper
through two holes. Steps, each kept only if KiCad DRC shows no new violation:

1. A wire whose two holes sit where the pads of
   an 0805 (0.8-3.2 mm apart) or a 1206 (1.8-4.4 mm apart) can cover them
   becomes a 0 ohm resistor soldered on the copper side: no holes, no
   wire. The part slides along the wire in 0.1 mm steps, because the track
   it bridges must sit in the gap between the pads, which the router does
   not centre. The 0805 is tried first (smaller), then the 1206 (more room
   under it). Repeated until nothing changes: a swap can free room for a
   neighbour.
2. Two wires joined only by a short bottom piece (the router dropped to the
   bottom to let another top wire cross) become one wire: two holes fewer.
   Then step 1 again, for room the joined pieces freed.
3. A 0 ohm part that no bottom track and no GND pour passes under becomes a
   plain bottom track.

Both steps are greedy and each can take room the other needed, so the board
is built both ways (1 then 2, and 2 then 1) and the one with the fewest
holes is kept, then the fewest bridges, then the shortest longest wire.
Prints what is left as wire jumpers.

usage: jumpers.py BOARD.kicad_pcb   (needs BOARD.kicad_pro beside it)
"""
import collections
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

import pcbnew

LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Base.pretty")
# footprint, value, via spacing it can take (pads reach the via positions)
SIZES = [("Jumper_0R_0805", "0R 0805", pcbnew.FromMM(0.8), pcbnew.FromMM(3.2)),
         ("Jumper_0R_1206", "0R 1206", pcbnew.FromMM(1.8), pcbnew.FromMM(4.4))]
GRAVE = []  # removed items stay referenced: pcbnew frees them twice otherwise


def drc(path):
    """New-violation keys. Silkscreen is placed later and library checks need
    the project's library table, which a scratch copy lacks: both ignored. An open
    connection is keyed by its nets: KiCad reports it with whichever items it
    picks as its ends, and those change when unrelated copper moves."""
    out = tempfile.mktemp(suffix=".json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json",
                    "-o", out, path], check=True, capture_output=True)
    data = json.load(open(out))
    os.remove(out)
    keys = set()
    for v in data["violations"] + data["unconnected_items"]:
        kind = "unconnected_items" if v in data["unconnected_items"] else v["type"]
        if kind.startswith(("silk_", "text_", "track_not_centered", "lib_footprint")):
            continue
        if kind == "unconnected_items":
            nets = {m for i in v["items"] for m in re.findall(r"\[([^\]]+)\]", i["description"])}
            keys.add((kind, tuple(sorted(nets))))
        else:
            keys.add((kind, tuple(sorted(i.get("uuid", "") for i in v["items"]))))
    return keys


def top_runs(board):
    """Connected groups of top tracks, with the vias at their ends."""
    top = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetLayer() == pcbnew.F_Cu]
    vias = {(v.GetPosition().x, v.GetPosition().y): v
            for v in board.GetTracks() if v.GetClass() == "PCB_VIA"}
    at = collections.defaultdict(list)
    for t in top:
        for e in (t.GetStart(), t.GetEnd()):
            at[(e.x, e.y)].append(t)
    seen, runs = set(), []
    for t in top:
        if id(t) in seen:
            continue
        group, todo = [], [t]
        while todo:
            s = todo.pop()
            if id(s) in seen:
                continue
            seen.add(id(s))
            group.append(s)
            for e in (s.GetStart(), s.GetEnd()):
                todo.extend(at[(e.x, e.y)])
        ends = [k for k in at if len([s for s in at[k] if s in group]) == 1]
        runs.append({"tracks": group, "ends": [vias.get(k) for k in ends]})
    return runs


def try_change(board, path, baseline, add, remove):
    """Apply a change; keep it and return the new baseline if DRC shows nothing
    new, else undo it and return None."""
    for item in remove:
        board.Remove(item)
    for item in add:
        board.Add(item)
    pcbnew.SaveBoard(path, board)
    if drc(path) - baseline:
        for item in add:
            board.Remove(item)
        GRAVE.extend(add)
        for item in remove:
            board.Add(item)
        pcbnew.SaveBoard(path, board)
        return None
    GRAVE.extend(remove)
    return drc(path)


def merge_chains(board, path, baseline):
    """Join wire, bottom piece, wire into one wire, following the same path."""
    merged = 0
    vias = {(v.GetPosition().x, v.GetPosition().y): v
            for v in board.GetTracks() if v.GetClass() == "PCB_VIA"}
    bottom = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetLayer() == pcbnew.B_Cu]
    at = collections.defaultdict(list)
    for t in bottom:
        for e in (t.GetStart(), t.GetEnd()):
            at[(e.x, e.y)].append(t)
    seen = set()
    for t in bottom:
        if id(t) in seen:
            continue
        piece, todo = [], [t]
        while todo:  # follow the piece through plain bends only
            s = todo.pop()
            if id(s) in seen:
                continue
            seen.add(id(s))
            piece.append(s)
            for e in (s.GetStart(), s.GetEnd()):
                if (e.x, e.y) not in vias and len(at[(e.x, e.y)]) == 2:
                    todo.extend(at[(e.x, e.y)])
        ends = collections.Counter((e.x, e.y) for s in piece for e in (s.GetStart(), s.GetEnd()))
        ends = [k for k, n in ends.items() if n == 1]
        if len(ends) != 2 or any(k not in vias or len(at[k]) != 1 for k in ends):
            continue
        net = piece[0].GetNetCode()
        if any(p.GetNetCode() == net and any(p.HitTest(pcbnew.VECTOR2I(*k)) for k in ends)
               for fp in board.GetFootprints() for p in fp.Pads()):
            continue
        top = []
        for s in piece:
            c = pcbnew.PCB_TRACK(board)
            c.SetStart(s.GetStart())
            c.SetEnd(s.GetEnd())
            c.SetWidth(min(s.GetWidth(), pcbnew.FromMM(0.3)))
            c.SetLayer(pcbnew.F_Cu)
            c.SetNet(s.GetNet())
            top.append(c)
        new = try_change(board, path, baseline, top, piece + [vias[k] for k in ends])
        if new is not None:
            baseline = new
            merged += 1
    return baseline, merged


def other_copper(board, net):
    """Bottom copper of other nets (tracks, vias, pads) as shapes, for a quick
    clearance test before the slow DRC run."""
    shapes = [t.GetEffectiveShape(pcbnew.B_Cu) for t in board.GetTracks()
              if t.GetNetCode() != net and t.IsOnLayer(pcbnew.B_Cu)]
    shapes += [p.GetEffectiveShape(pcbnew.B_Cu) for fp in board.GetFootprints()
               for p in fp.Pads() if p.GetNetCode() != net and p.IsOnLayer(pcbnew.B_Cu)]
    return shapes


def pad_stubs(board, fp, like):
    """Short tracks from each bottom track end that touches a pad of fp to the
    pad centre, so the joint is the full track width, not the sliver where a
    track cap overlaps the pad edge. One stub per end point."""
    stubs, done = [], set()
    for p in fp.Pads():
        centre = p.GetPosition()
        for t in board.GetTracks():
            if t.GetClass() != "PCB_TRACK" or t.GetLayer() != pcbnew.B_Cu \
                    or t.GetNetCode() != like.GetNetCode():
                continue
            for e in (t.GetStart(), t.GetEnd()):
                if e != centre and (e.x, e.y) not in done and p.HitTest(e, t.GetWidth() // 2):
                    done.add((e.x, e.y))
                    stub = pcbnew.PCB_TRACK(board)
                    stub.SetStart(e)
                    stub.SetEnd(centre)
                    stub.SetWidth(min(t.GetWidth(), pcbnew.FromMM(0.3)))
                    stub.SetLayer(pcbnew.B_Cu)
                    stub.SetNet(like.GetNet())
                    stubs.append(stub)
    return stubs


def as_resistor(board, path, baseline, run, n):
    """Swap one wire for an 0805 or 1206 slid along the line between its
    holes; the new baseline and the value used, or None."""
    tracks, ends = run["tracks"], run["ends"]
    if len(ends) != 2 or None in ends:
        return None
    a, c = ends[0].GetPosition(), ends[1].GetPosition()
    span = math.dist((a.x, a.y), (c.x, c.y))
    if span < 1:
        return None
    ux, uy = (c.x - a.x) / span, (c.y - a.y) / span
    angle = math.degrees(math.atan2(-(c.y - a.y), c.x - a.x))
    clearance = board.GetDesignSettings().m_MinClearance
    others = other_copper(board, tracks[0].GetNetCode())  # before any part is added
    for name, value, lo, hi in SIZES:
        if not lo <= span <= hi:
            continue
        half_pitch, half_pad = (pcbnew.FromMM(1.0), pcbnew.FromMM(0.6)) if "0805" in name \
            else (pcbnew.FromMM(1.55), pcbnew.FromMM(0.65))
        # offsets along the wire that keep both holes on a pad
        steps = sorted(range(-12, 13), key=abs)
        for k in steps:
            d = pcbnew.FromMM(0.1 * k)
            if not all(half_pitch - half_pad <= abs(span / 2 + sgn * d) <= half_pitch + half_pad
                       for sgn in (1, -1)):
                continue
            fp = pcbnew.FootprintLoad(LIB, name)
            fp.SetReference(f"JP{n}")
            fp.SetValue(value)
            mid = pcbnew.VECTOR2I(int((a.x + c.x) / 2 + d * ux), int((a.y + c.y) / 2 + d * uy))
            board.Add(fp)  # Flip needs the board (layer set)
            fp.SetPosition(mid)
            fp.Flip(mid, pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetOrientationDegrees(angle)
            if not all(any(p.HitTest(e.GetPosition()) for p in fp.Pads()) for e in ends):
                fp.SetOrientationDegrees(-angle)  # flipped parts turn the other way
            ok = all(any(p.HitTest(e.GetPosition()) for p in fp.Pads()) for e in ends)
            if ok:
                ok = not any(p.GetEffectiveShape(pcbnew.B_Cu).Collide(o, clearance)
                             for p in fp.Pads() for o in others)
            board.Remove(fp)
            if not ok:
                GRAVE.append(fp)
                continue
            for p in fp.Pads():
                p.SetNet(tracks[0].GetNet())
            new = try_change(board, path, baseline, [fp] + pad_stubs(board, fp, tracks[0]),
                             tracks + ends)
            if new is not None:
                return new, value
    return None


def build(path, resistors_first):
    """One way of turning wires into parts; returns the counts."""
    board = pcbnew.LoadBoard(path)
    baseline = drc(path)
    swapped = collections.Counter()
    n = 1 + max([int(f.GetReference()[2:]) for f in board.GetFootprints()
                 if re.fullmatch(r"JP\d+", f.GetReference())] or [0])

    def resistors(baseline, n):
        changed = True
        while changed:
            changed = False
            for run in top_runs(board):
                done = as_resistor(board, path, baseline, run, n)
                if done:
                    baseline, value = done
                    swapped[value] += 1
                    n += 1
                    changed = True
        return baseline, n

    if resistors_first:
        baseline, n = resistors(baseline, n)
    baseline, merged = merge_chains(board, path, baseline)
    if merged or not resistors_first:
        baseline, n = resistors(baseline, n)
    # A 0 ohm part that no bottom track passes under may bridge nothing, or
    # only let the GND pour through (DRC then reports the pour split): try a
    # plain bottom track between its pad positions instead.
    dropped = 0
    for fp in [f for f in board.GetFootprints() if f.GetReference().startswith("JP")]:
        pads = list(fp.Pads())
        a, c = pads[0].GetPosition(), pads[1].GetPosition()
        net = pads[0].GetNet()
        under = pcbnew.SEG(a, c)
        if any(t.GetClass() == "PCB_TRACK" and t.GetLayer() == pcbnew.B_Cu and
               t.GetNetCode() != net.GetNetCode() and
               pcbnew.SEG(t.GetStart(), t.GetEnd()).Intersects(under)
               for t in board.GetTracks()):
            continue
        width = max([t.GetWidth() for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"
                     and t.GetNetCode() == net.GetNetCode()] or [pcbnew.FromMM(0.3)])
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(a)
        track.SetEnd(c)
        track.SetWidth(min(width, pcbnew.FromMM(0.5)))
        track.SetLayer(pcbnew.B_Cu)
        track.SetNet(net)
        board.Remove(fp)
        board.Add(track)
        pcbnew.SaveBoard(path, board)
        if drc(path) - baseline:
            board.Remove(track)
            GRAVE.append(track)
            board.Add(fp)
        else:
            GRAVE.append(fp)
            dropped += 1
            baseline = drc(path)
        pcbnew.SaveBoard(path, board)
    left = top_runs(board)
    lengths = sorted(round(pcbnew.ToMM(sum(t.GetLength() for t in r["tracks"])), 1) for r in left)
    parts = sum(1 for f in board.GetFootprints() if f.GetReference().startswith("JP"))
    score = (2 * len(left), len(left) + parts, lengths[-1] if lengths else 0)
    report = (f"wire chains joined: {merged} | 0 ohm jumpers: {dict(swapped)} | "
              f"replaced by a plain track: {dropped}\n"
              f"wire jumpers left: {len(left)} lengths mm: {lengths}")
    return score, report


def main(path):
    pro = os.path.splitext(path)[0] + ".kicad_pro"
    tries = []
    with tempfile.TemporaryDirectory() as tmp:
        for first in (True, False):
            work = os.path.join(tmp, f"try{int(first)}.kicad_pcb")
            shutil.copy(path, work)
            shutil.copy(pro, os.path.splitext(work)[0] + ".kicad_pro")
            score, report = build(work, first)
            tries.append((score, first, work, report))
        score, first, work, report = min(tries)
        shutil.copy(work, path)
    print("kept:", "0 ohm parts first" if first else "wire chains first",
          f"| holes {score[0]}, bridges {score[1]}, longest wire {score[2]} mm")
    print(report)


if __name__ == "__main__":
    main(sys.argv[1])
