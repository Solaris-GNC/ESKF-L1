#!/usr/bin/env python3
"""update_params_md.py — mete en docs/PARAMETROS_SPP.md la tabla de `svc_replay --dump-md`
(fuente única: src/eskf_svc_params.c) entre las marcas. Con --check sólo comprueba.

  update_params_md.py docs/PARAMETROS_SPP.md build/svc_replay [--check]
"""
import subprocess
import sys

A, B = "<!-- TABLA-INICIO (generada: make docs) -->", "<!-- TABLA-FIN -->"


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    md, exe = sys.argv[1], sys.argv[2]
    table = subprocess.run([exe, "--dump-md"], capture_output=True, text=True, check=True).stdout.strip()
    txt = open(md).read()
    if A not in txt or B not in txt:
        print(f"{md}: faltan las marcas de la tabla")
        return 1
    pre, rest = txt.split(A, 1)
    _, post = rest.split(B, 1)
    new = pre + A + "\n" + table + "\n" + B + post
    if "--check" in sys.argv:
        ok = new == txt
        print(f"docs-check: {md} {'coincide' if ok else 'NO coincide'} con eskf_svc_params.c")
        return 0 if ok else 1
    open(md, "w").write(new)
    print(f"{md}: tabla actualizada")
    return 0


if __name__ == "__main__":
    sys.exit(main())
