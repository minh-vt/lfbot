"""Tidy routed copper in place, gated by KiCad DRC.

First snaps track ends lying inside a via onto its centre and extends free
ends that only graze a pad into it. Then splits each net's tracks into runs between anchors (pad, via, branch, width
change, free end) and rewrites a run only when that makes it cleaner:
collinear pieces merged, jogs shorter than JOG folded away, off-angle
segments made 0/45/90 degrees (slide the bend along the previous segment, or
a dogleg), 90 degree corners chamfered at 45 degrees. A run that carries a
T-junction is left as it is. After rewriting, KiCad DRC runs on the board
with the project's own rules; every run near a violation goes back to its
original copper and DRC runs again, until clean.

usage: tidy_tracks.py BOARD.kicad_pcb   (needs BOARD.kicad_pro beside it)
"""
import collections
import json
import math
import os
import subprocess
import sys
import tempfile

import pcbnew

TOL = 200                      # nm: coordinate noise from the router grid
JOG = pcbnew.FromMM(0.25)      # shorter interior pieces are folded away
CHAMFER = pcbnew.FromMM(1.0)   # longest leg cut off a 90 degree corner
MIN_CHAMFER = pcbnew.FromMM(0.2)
MAX_SLIDE = pcbnew.FromMM(1.0)
# pcbnew frees a removed item a second time once Python drops its last
# reference (segfault, KiCad 10.0.8), so every removed item is kept here
GRAVE = []


def remove(board, item):
  board.Remove(item)
  GRAVE.append(item)


def key(p, t):
  return (p.x, p.y, t.GetLayer(), t.GetNetCode())


def octilinear(a, c):
  dx, dy = c[0] - a[0], c[1] - a[1]
  return abs(dx) <= TOL or abs(dy) <= TOL or abs(abs(dx) - abs(dy)) <= TOL


def unit(a, c):
  dx, dy = c[0] - a[0], c[1] - a[1]
  n = math.hypot(dx, dy)
  return (dx / n, dy / n) if n else (0.0, 0.0)


def collinear(a, b, c):
  u, v = unit(a, b), unit(b, c)
  return abs(u[0] * v[1] - u[1] * v[0]) < 1e-3 and u[0] * v[0] + u[1] * v[1] > 0


def dist(a, c):
  return math.hypot(c[0] - a[0], c[1] - a[1])


def snap_to_vias(board):
  """Move free track ends that sit inside a same-net via's copper onto its
  centre. A corner joining two tracks is left alone even if it lies on the
  via's copper."""
  vias = [v for v in board.GetTracks() if v.GetClass() == 'PCB_VIA']
  tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
  ends = collections.Counter(key(e, t) for t in tracks for e in (t.GetStart(), t.GetEnd()))
  n = 0
  for t in tracks:
    for get, put in ((t.GetStart, t.SetStart), (t.GetEnd, t.SetEnd)):
      e = get()
      if ends[key(e, t)] > 1:
        continue
      for v in vias:
        c = v.GetPosition()
        if v.GetNetCode() == t.GetNetCode() and e != c and \
            dist((e.x, e.y), (c.x, c.y)) < v.GetWidth(pcbnew.F_Cu) / 2:
          put(c)
          n += 1
          break
  for t in [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK' and t.GetLength() == 0]:
    remove(board, t)
  return n


def seat_pad_ends(board):
  """Extend a free track end that only grazes a same-net pad along its own
  line to the point nearest the pad centre, so it lands inside the pad."""
  tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
  vias = {(v.GetPosition().x, v.GetPosition().y) for v in board.GetTracks()
          if v.GetClass() == 'PCB_VIA'}
  ends = collections.Counter(key(e, t) for t in tracks for e in (t.GetStart(), t.GetEnd()))
  pads = [p for f in board.GetFootprints() for p in f.Pads()]
  n = 0
  for t in tracks:
    for get, put, other in ((t.GetStart, t.SetStart, t.GetEnd),
                            (t.GetEnd, t.SetEnd, t.GetStart)):
      e = get()
      if ends[key(e, t)] > 1 or (e.x, e.y) in vias:
        continue
      for p in pads:
        if p.GetNetCode() != t.GetNetCode() or not p.IsOnLayer(t.GetLayer()) \
            or p.HitTest(e) or not p.HitTest(e, t.GetWidth() // 2):
          continue
        o, c = other(), p.GetPosition()
        d = (e.x - o.x, e.y - o.y)
        s = ((c.x - o.x) * d[0] + (c.y - o.y) * d[1]) / float(d[0] ** 2 + d[1] ** 2)
        q = pcbnew.VECTOR2I(round(o.x + s * d[0]), round(o.y + s * d[1]))
        if s > 1 and p.HitTest(q):
          put(q)
          n += 1
        break
  return n


def split_at_tees(board):
  """Split every segment whose body carries another same-net track's end, so
  the T-junction becomes an ordinary branch node."""
  tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
  vias = {(v.GetPosition().x, v.GetPosition().y, v.GetNetCode())
          for v in board.GetTracks() if v.GetClass() == 'PCB_VIA'}
  pads = [p for f in board.GetFootprints() for p in f.Pads()]
  node = collections.defaultdict(list)
  for t in tracks:
    node[key(t.GetStart(), t)].append(t)
    node[key(t.GetEnd(), t)].append(t)
  n = 0
  for k, arriving in list(node.items()):
    pt = pcbnew.VECTOR2I(k[0], k[1])
    if len(arriving) != 1 or (k[0], k[1], k[3]) in vias or any(
        p.GetNetCode() == k[3] and p.IsOnLayer(k[2]) and p.HitTest(pt) for p in pads):
      continue
    stem = arriving[0]
    su = unit((stem.GetStart().x, stem.GetStart().y), (stem.GetEnd().x, stem.GetEnd().y))
    for t in tracks:
      if t is stem or (t.GetLayer(), t.GetNetCode()) != (k[2], k[3]) \
          or pt in (t.GetStart(), t.GetEnd()):
        continue
      a, c = t.GetStart(), t.GetEnd()
      cross = (c.x - a.x) * (pt.y - a.y) - (c.y - a.y) * (pt.x - a.x)
      if abs(cross) > TOL * t.GetLength() or not t.HitTest(pt, 0):
        continue
      tu = unit((a.x, a.y), (c.x, c.y))
      if abs(su[0] * tu[1] - su[1] * tu[0]) < 1e-3:   # overlapping, not a T
        continue
      tail = t.Duplicate()
      tail.SetStart(pt)
      t.SetEnd(pt)
      board.Add(tail)
      tracks.append(tail)
      n += 1
      break
  return n


def build_runs(board):
  tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
  vias = {(v.GetPosition().x, v.GetPosition().y, v.GetNetCode())
          for v in board.GetTracks() if v.GetClass() == 'PCB_VIA'}
  pads = [p for f in board.GetFootprints() for p in f.Pads()]
  node = collections.defaultdict(list)
  for t in tracks:
    node[key(t.GetStart(), t)].append(t)
    node[key(t.GetEnd(), t)].append(t)

  def anchored(k):
    ts = node[k]
    if len(ts) != 2 or ts[0].GetWidth() != ts[1].GetWidth():
      return True
    if (k[0], k[1], k[3]) in vias:
      return True
    pt = pcbnew.VECTOR2I(k[0], k[1])
    return any(p.GetNetCode() == k[3] and p.IsOnLayer(k[2]) and p.HitTest(pt)
               for p in pads)

  # free ends that land on another track's body (T-junctions)
  tees = [k for k, ts in node.items() if len(ts) == 1 and (k[0], k[1], k[3]) not in vias
          and not any(p.GetNetCode() == k[3] and p.IsOnLayer(k[2])
                      and p.HitTest(pcbnew.VECTOR2I(k[0], k[1])) for p in pads)]
  seen, runs = set(), []
  for start, ts in node.items():
    if not anchored(start):
      continue
    for t in ts:
      if id(t) in seen:
        continue
      pts, segs, k, cur = [(start[0], start[1])], [], start, t
      while True:
        seen.add(id(cur))
        segs.append(cur)
        a, c = key(cur.GetStart(), cur), key(cur.GetEnd(), cur)
        k = c if a == k else a
        pts.append((k[0], k[1]))
        if anchored(k):
          break
        cur = next(x for x in node[k] if x is not cur)
      hosts_tee = any(
          tk[2] == segs[0].GetLayer() and tk[3] == segs[0].GetNetCode()
          and any(s.HitTest(pcbnew.VECTOR2I(tk[0], tk[1]), 0) and
                  (tk[0], tk[1]) not in ((s.GetStart().x, s.GetStart().y),
                                         (s.GetEnd().x, s.GetEnd().y)) for s in segs)
          for tk in tees)
      runs.append({'pts': pts, 'segs': segs, 'frozen': hosts_tee})
  return runs


def drop_collinear(p):
  out = [p[0]]
  for i in range(1, len(p) - 1):
    if dist(out[-1], p[i]) <= TOL or collinear(out[-1], p[i], p[i + 1]):
      continue
    out.append(p[i])
  out.append(p[-1])
  return out


def fold_jogs(p):
  changed = True
  while changed and len(p) > 2:
    changed = False
    for i in range(len(p) - 1):
      if dist(p[i], p[i + 1]) >= JOG:
        continue
      if 0 < i and i + 1 < len(p) - 1:       # both interior: meet halfway
        m = ((p[i][0] + p[i + 1][0]) // 2, (p[i][1] + p[i + 1][1]) // 2)
        p = p[:i] + [m] + p[i + 2:]
      elif 0 < i:                             # p[i+1] is the end anchor
        p = p[:i] + p[i + 1:]
      elif i + 1 < len(p) - 1:                # p[i] is the start anchor
        p = p[:i + 1] + p[i + 2:]
      else:
        continue
      changed = True
      break
  return p


def line_cross(a, d, c, e):
  """Point where a + s*d meets c + r*e, or None."""
  den = d[0] * e[1] - d[1] * e[0]
  if abs(den) < 1e-9:
    return None
  s = ((c[0] - a[0]) * e[1] - (c[1] - a[1]) * e[0]) / den
  return (a[0] + s * d[0], a[1] + s * d[1]), s


DIRS = [(1, 0), (0, 1), (1, 1), (1, -1)]


def straighten(p):
  i, budget = 0, 200
  while i < len(p) - 1:
    budget -= 1
    if budget == 0:
      raise ValueError('straighten did not converge')
    a, c = p[i], p[i + 1]
    if octilinear(a, c):
      i += 1
      continue
    fixed = False
    # slide an interior end along its other segment
    for move, fixed_pt, along in ((i, c, p[i - 1] if i > 0 else None),
                                  (i + 1, a, p[i + 2] if i + 2 < len(p) else None)):
      if along is None or move in (0, len(p) - 1):
        continue
      cur = p[move]
      d = (cur[0] - along[0], cur[1] - along[1])
      best = None
      for e in DIRS:
        hit = line_cross(along, d, fixed_pt, e)
        if not hit or hit[1] <= 0.05:
          continue
        q = (round(hit[0][0]), round(hit[0][1]))
        if dist(q, cur) <= MAX_SLIDE and (best is None or dist(q, cur) < dist(best, cur)):
          best = q
      if best:
        p = p[:move] + [best] + p[move + 1:]
        fixed = True
        break
    if not fixed:                             # dogleg: diagonal then straight
      dx, dy = c[0] - a[0], c[1] - a[1]
      d = min(abs(dx), abs(dy))
      corner = (a[0] + int(math.copysign(d, dx)), a[1] + int(math.copysign(d, dy)))
      p = p[:i + 1] + [corner] + p[i + 1:]
    i = 0
    p = drop_collinear(p)
  return p


def chamfer(p):
  out = [p[0]]
  for i in range(1, len(p) - 1):
    a, b, c = out[-1], p[i], p[i + 1]
    u, v = unit(a, b), unit(b, c)
    if abs(u[0] * v[0] + u[1] * v[1]) > 1e-3:      # not a right angle
      out.append(b)
      continue
    leg = min(CHAMFER, 0.45 * dist(a, b), 0.45 * dist(b, c))
    if leg < MIN_CHAMFER:
      out.append(b)
      continue
    out.append((round(b[0] - u[0] * leg), round(b[1] - u[1] * leg)))
    out.append((round(b[0] + v[0] * leg), round(b[1] + v[1] * leg)))
  out.append(p[-1])
  return out


def variants(pts):
  """Rewrites from boldest to safest; the last only merges collinear pieces,
  which leaves the copper outline unchanged."""
  out = []
  try:
    p = straighten(fold_jogs(drop_collinear(list(pts))))
    out.append(drop_collinear(chamfer(p)))
  except ValueError:
    pass
  out.append(drop_collinear(chamfer(drop_collinear(list(pts)))))
  out.append(drop_collinear(list(pts)))
  uniq = []
  for v in out:
    if v != list(pts) and v not in uniq:
      uniq.append(v)
  return uniq


def write_run(board, run, pts):
  proto = run['segs'][0]
  new = []
  for a, c in zip(pts, pts[1:]):
    s = pcbnew.PCB_TRACK(board)
    s.SetStart(pcbnew.VECTOR2I(int(a[0]), int(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(int(c[0]), int(c[1])))
    s.SetWidth(proto.GetWidth())
    s.SetLayer(proto.GetLayer())
    s.SetNet(proto.GetNet())
    board.Add(s)
    new.append(s)
  return new


def drc(path):
  out = tempfile.mktemp(suffix='.json')
  subprocess.run(['kicad-cli', 'pcb', 'drc', '--refill-zones', '--format', 'json',
                  '-o', out, path], check=True, capture_output=True)
  d = json.load(open(out))
  os.remove(out)
  # library setup and silkscreen are not copper, which is all this edits
  return [v for v in d['violations'] + d['unconnected_items']
          if not v['type'].startswith(('lib_footprint', 'silk_', 'text_'))]


def main(path):
  board = pcbnew.LoadBoard(path)
  print('track ends snapped to vias:', snap_to_vias(board))
  print('track ends seated in pads:', seat_pad_ends(board))
  print('T-junctions split:', split_at_tees(board))
  changed = []
  for run in build_runs(board):
    if run['frozen']:
      continue
    run['todo'] = variants(run['pts'])
    if not run['todo']:
      continue
    run['saved'] = [s.Duplicate() for s in run['segs']]
    for s in run['segs']:
      remove(board, s)
    run['new'] = write_run(board, run, run['todo'].pop(0))
    changed.append(run)
  print('runs rewritten:', len(changed))
  for rnd in range(8):
    pcbnew.SaveBoard(path, board)
    bad = drc(path)
    if not bad:
      break
    spots = [(pcbnew.FromMM(i['pos']['x']), pcbnew.FromMM(i['pos']['y']))
             for v in bad for i in v['items']]
    hit = [r for r in changed if any(
        s.HitTest(pcbnew.VECTOR2I(int(x), int(y)), pcbnew.FromMM(0.05))
        for s in r['new'] for x, y in spots)]
    if not hit:   # e.g. a zone sliver: fall back to anything nearby
      hit = [r for r in changed if any(
          s.HitTest(pcbnew.VECTOR2I(int(x), int(y)), pcbnew.FromMM(2.0))
          for s in r['new'] for x, y in spots)]
    if not hit:
      sys.exit('DRC violations not traceable to a rewritten run: %s' % bad)
    for r in hit:
      for s in r['new']:
        remove(board, s)
      if r['todo']:
        r['new'] = write_run(board, r, r['todo'].pop(0))
      else:
        for s in r['saved']:
          board.Add(s)
        changed.remove(r)
    print('round %d: %d violations, stepped back %d runs' % (rnd, len(bad), len(hit)))
  else:
    sys.exit('still failing DRC after 8 rounds')
  print('runs kept:', len(changed))


if __name__ == '__main__':
  main(sys.argv[1])
