"""Copy a .kicad_pcb without its routed copper: drops top-level segments, vias,
arcs and copper pours, keeps footprints, text, outline and rule areas
(keep-outs). With NETs (exact names or patterns such as "/ADC_IR*"), drops
only the segments, vias and arcs of those nets and keeps every pour. Works on the file text because pcbnew segfaults
removing many tracks from Python (KiCad 10.0.8).

usage: strip_copper.py IN.kicad_pcb OUT.kicad_pcb [NET ...]
"""
import fnmatch
import re
import sys


def strip(text, nets=None):
    out, pos, depth, in_str, start = [], 0, 0, False, None
    i, n = 0, len(text)
    removed = {}
    while i < n:
        c = text[i]
        if in_str:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
            if depth == 2:
                start = i
        elif c == ")":
            if depth == 2:
                block = text[start:i + 1]
                head = block[1:].split(None, 1)[0]
                if nets is None:
                    drop = head in ("segment", "via", "arc") or (
                        head == "zone" and "(keepout" not in block)
                else:
                    m = re.search(r'\(net "([^"]*)"\)', block)
                    drop = head in ("segment", "via", "arc") and bool(m) and any(
                        fnmatch.fnmatchcase(m.group(1), pat) for pat in nets)
                if drop:
                    k = start
                    while k > 0 and text[k - 1] in " \t\n":
                        k -= 1
                    out.append(text[pos:k])
                    pos = i + 1
                    removed[head] = removed.get(head, 0) + 1
            depth -= 1
        i += 1
    out.append(text[pos:])
    return "".join(out), removed


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    stripped, removed = strip(open(src).read(), set(sys.argv[3:]) or None)
    open(dst, "w").write(stripped)
    print("stripped", removed)
