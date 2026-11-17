#!/usr/bin/env python3
"""
make_qr — QR-code naar de repository voor de titelpagina van de PDF.

Losse stap, zodat de PDF-bouw zelf geen netwerk of extra pakketten nodig
heeft: het resultaat (docs/img/qr_repo.png) gaat mee in de repo.

    .venv/bin/pip install qrcode
    .venv/bin/python tools/paper/make_qr.py
"""

from pathlib import Path

import qrcode
from qrcode.constants import ERROR_CORRECT_M

URL = "https://github.com/chat-l18l/no_cutting_corners_with_AI"
OUT = Path(__file__).resolve().parents[2] / "docs" / "img" / "qr_repo.png"


def main() -> None:
    qr = qrcode.QRCode(
        version=None,                 # kleinste versie die past
        error_correction=ERROR_CORRECT_M,
        box_size=12,                  # ruim: in de PDF staat hij op ~25 mm
        border=2,                     # stille zone (minimaal 2 modules)
    )
    qr.add_data(URL)
    qr.make(fit=True)
    img = qr.make_image(fill_color="#10151c", back_color="white")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT)
    print(f"{OUT}  ({img.size[0]}x{img.size[1]} px)  ->  {URL}")


if __name__ == "__main__":
    main()
