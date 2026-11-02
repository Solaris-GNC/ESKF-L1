#!/usr/bin/env python3
"""update_docs.py — mete en PARAMETROS.md la tabla que genera dump_params --md (fuente: vn_params.c),
entre las marcas <!-- TABLA-INICIO --> y <!-- TABLA-FIN -->. Con --check sólo comprueba."""
import subprocess
import sys

A, B = "<!-- TABLA-INICIO (generada: make -C firmware docs) -->", "<!-- TABLA-FIN -->"


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    md, dump = sys.argv[1], sys.argv[2]
    check = "--check" in sys.argv
    table = subprocess.run([dump, "--md"], capture_output=True, text=True, check=True).stdout.strip()
    txt = open(md).read()
    if A not in txt or B not in txt:
        print(f"{md}: faltan las marcas de la tabla")
        return 1
    pre, rest = txt.split(A, 1)
    _, post = rest.split(B, 1)
    new = pre + A + "\n" + table + "\n" + B + post
    if check:
        if new != txt:
            print(f"docs-check: {md} no coincide con vn_params.c (make -C firmware docs)")
            return 1
        print(f"docs-check: {md} coincide con vn_params.c")
        return 0
    with open(md, "w") as f:
        f.write(new)
    print(f"{md}: tabla actualizada")
    return 0


if __name__ == "__main__":
    sys.exit(main())
