#!/usr/bin/env python3
"""
build_pdf — maakt docs/paper_no_cutting_corners.pdf uit de Markdown-versie.

Eén bron, twee uitvoeren: docs/paper_no_cutting_corners.md is wat je op
GitHub leest, deze PDF is wat je rondstuurt. Het script doet onderweg vier
dingen die in de Markdown niet thuishoren:

  1. figuren verwijzen naar docs/img/print/ (op A4-maat gerenderd, 300 dpi)
     in plaats van de bredere webversies;
  2. de brede driepanelen-figuur komt op een eigen liggende pagina;
  3. een kader met repository-URL en QR-code onder de titel;
  4. een inhoudsopgave na de Nederlandse samenvatting.

Engine: pdflatex (zie tools/paper/header.tex voor waarom, en voor de
afhandeling van de Griekse letters).

Voorwaarden: pandoc + pdflatex, en de afdrukfiguren:

    .venv/bin/python tools/viz/make_paper_figures.py --print
    .venv/bin/python tools/paper/make_qr.py
    python3 tools/paper/build_pdf.py
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MD = REPO / "docs" / "paper_no_cutting_corners.md"
PDF = REPO / "docs" / "paper_no_cutting_corners.pdf"
HEADER = REPO / "tools" / "paper" / "header.tex"
IMG = REPO / "docs" / "img"

TITLE = "No Cutting Corners"
# -M geeft letterlijke tekst door: geen Markdown, geen LaTeX. Dus hier
# echte em-dashes en geen macro's.
SUBTITLE = ("A sampling-based model predictive controller for small "
            "Ackermann, skid-steer, differential-drive and holonomic "
            "vehicles — written in dependency-free C")
AUTHOR = "Edwin van den Oetelaar"
DATE = ("Fontys ICT · research related to Lectoraat HTES"
        "  ·  November 2026")

# Deze figuur krijgt een liggende pagina (drie panelen naast elkaar).
WIDE = "fig2_corner_braking.png"

# δ + combinerende punt (U+0307) kan pdflatex niet zetten, ook niet binnen
# \texttt. Eén plaatsvervangend teken, in header.tex gedefinieerd als \dot\delta.
DOT_DELTA = "δ̇"
DOT_DELTA_TOKEN = ""

# Het modelblok staat in de Markdown als codeblok (leesbaar op GitHub), maar
# in een paper hoort het als wiskunde gezet te worden.
EQUATIONS = r"""
```{=latex}
\begin{align*}
  \dot{x}      &= v \cos\theta \\
  \dot{y}      &= v \sin\theta \\
  \dot{\theta} &= \frac{v}{L}\,\tan\delta
                  && \text{$L$ = wheelbase}\\
  \dot{v}      &= \frac{v_{\mathrm{cmd}} - v}{\tau_v}
                  && \text{first-order lag + dead time}\\
  \dot{\delta} &= \operatorname{clamp}\!\left(
                    \frac{\delta_{\mathrm{cmd}} - \delta}{\tau_\delta},\;
                    -\dot{\delta}_{\max},\; +\dot{\delta}_{\max}\right)
                  && \text{servo: rate-limited}
\end{align*}
```
"""


def transform(md: str) -> str:
    """Markdown -> pandoc-invoer met de PDF-specifieke ingrepen."""
    # 1. alles vóór de samenvatting weg: titel en auteur komen uit metadata
    start = md.index("## Samenvatting voor de groep")
    body = md[start:]

    # 2. figuren: afdrukvariant waar die bestaat, anders de webversie
    def img_ref(m: re.Match) -> str:
        name = m.group(2)
        path = f"img/print/{name}" if (IMG / "print" / name).is_file() \
            else f"img/{name}"
        return f"![]({path}){{width=100%}}"

    body = re.sub(r"!\[([^\]]*)\]\(img/([^)]+)\)", img_ref, body)

    # 3. de brede figuur plus zijn onderschrift op een liggende pagina.
    #    Het onderschrift blijft Markdown: pandoc zet alleen de omlijsting
    #    als ruwe LaTeX door en verwerkt de tekst ertussen gewoon.
    wide_path = f"img/print/{WIDE}" if (IMG / "print" / WIDE).is_file() \
        else f"img/{WIDE}"
    pattern = re.compile(
        r"!\[\]\(" + re.escape(wide_path) + r"\)\{width=100%\}\n\n"
        r"(\*\*Figure \d+\.\*\*.*?)(?=\n\n)", re.S)

    def landscape(m: re.Match) -> str:
        caption = m.group(1)
        return (
            "```{=latex}\n"
            f"\\widefigopen{{{wide_path}}}\n"
            "```\n\n"
            f"{caption}\n\n"
            "```{=latex}\n"
            "\\widefigclose\n"
            "```"
        )

    body, n_wide = pattern.subn(landscape, body)
    if n_wide != 1:
        sys.exit(f"FOUT: brede figuur {WIDE} niet precies één keer gevonden "
                 f"(gevonden: {n_wide})")

    # 4. repository-kader onder de titel, inhoudsopgave na de samenvatting
    body = ("```{=latex}\n\\repobox\n```\n\n" + body)
    body = body.replace(
        "\n## 1. Why not an off-the-shelf path follower",
        "\n```{=latex}\n\\clearpage\n\\tableofcontents\n\\clearpage\n```\n"
        "\n## 1. Why not an off-the-shelf path follower", 1)

    # 5. de notitie over de Markdown-versie hoort niet in de PDF
    body = re.sub(r"> Deze Markdown-versie.*?\n\n", "", body, flags=re.S)

    # 6. het modelblok als wiskunde in plaats van als codeblok
    eq_block = re.compile(r"```\n(x'  = v .*?rate-limited\n)```", re.S)
    body, n_eq = eq_block.subn(lambda m: EQUATIONS, body)
    if n_eq != 1:
        sys.exit(f"FOUT: modelblok niet precies één keer gevonden ({n_eq})")

    # 7. delta-met-punt: één teken dat ook binnen \texttt werkt
    body = body.replace(DOT_DELTA, DOT_DELTA_TOKEN)
    return body


def main() -> None:
    for tool in ("pandoc", "pdflatex"):
        if shutil.which(tool) is None:
            sys.exit(f"FOUT: {tool} niet gevonden")
    if not (IMG / "print").is_dir():
        sys.exit("FOUT: docs/img/print/ ontbreekt — draai eerst "
                 "make_paper_figures.py --print")
    if not (IMG / "qr_repo.png").is_file():
        sys.exit("FOUT: docs/img/qr_repo.png ontbreekt — draai eerst "
                 "tools/paper/make_qr.py")

    tmp = REPO / "docs" / ".paper_pdf_input.md"
    tmp.write_text(transform(MD.read_text(encoding="utf-8")), encoding="utf-8")

    # LaTeX zoekt afbeeldingen relatief aan zijn eigen werkmap, niet aan de
    # Markdown; geef het zoekpad expliciet mee (geldt ook voor de QR-code in
    # de preamble, die pandoc niet herschrijft).
    gpath = REPO / "docs" / ".paper_graphicspath.tex"
    gpath.write_text(
        "\\usepackage{graphicx}\n"
        f"\\graphicspath{{{{{REPO / 'docs'}/}}}}\n", encoding="utf-8")

    cmd = [
        "pandoc", str(tmp),
        "-o", str(PDF),
        "--pdf-engine=pdflatex",
        "--from", "markdown-implicit_figures",
        "--resource-path", str(REPO / "docs"),
        "--include-in-header", str(gpath),
        "--include-in-header", str(HEADER),
        "--highlight-style", "tango",
        "-V", "documentclass=article",
        "-V", "fontfamily=mathpazo",
        "-V", "fontsize=11pt",
        "-V", "geometry:a4paper,top=2.4cm,bottom=2.4cm,left=2.5cm,right=2.5cm",
        "-V", "linkcolor=inkblue",
        "-V", "urlcolor=inkblue",
        "-V", "toccolor=black",
        "-V", "colorlinks=true",
        "-V", "subparagraph=yes",
        "-M", f"title={TITLE}",
        "-M", f"subtitle={SUBTITLE}",
        "-M", f"author={AUTHOR}",
        "-M", f"date={DATE}",
        "-M", "lang=en",
    ]
    print(" ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    if res.returncode != 0:
        print(res.stdout[-4000:])
        print(res.stderr[-4000:], file=sys.stderr)
        sys.exit("FOUT: pandoc/lualatex faalde")
    if res.stderr.strip():
        print(res.stderr.strip()[:2000])
    tmp.unlink()
    gpath.unlink()
    size = PDF.stat().st_size / 1024
    print(f"geschreven: {PDF}  ({size:.0f} kB)")


if __name__ == "__main__":
    main()
