"""Minimal s-expression reader/writer for KiCad files."""


def parse(text):
    stack = [[]]
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == "(":
            stack.append([])
            i += 1
        elif c == ")":
            node = stack.pop()
            stack[-1].append(node)
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j:j + 2])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            stack[-1].append(Q("".join(buf)))
            i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            stack[-1].append(text[i:j])
            i = j
    return stack[0]


class Q(str):
    """Quoted string atom."""


def dump(node, indent=0):
    if isinstance(node, Q):
        return '"' + str(node) + '"'
    if isinstance(node, str):
        return node
    pad = "\t" * indent
    if all(not isinstance(x, list) for x in node):
        return "(" + " ".join(dump(x) for x in node) + ")"
    parts = []
    for x in node:
        if isinstance(x, list):
            parts.append("\n" + pad + "\t" + dump(x, indent + 1))
        else:
            parts.append((" " if parts else "") + dump(x))
    return "(" + "".join(parts) + "\n" + pad + ")"


def find(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def first(node, key):
    r = find(node, key)
    return r[0] if r else None
