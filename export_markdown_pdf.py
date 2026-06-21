from __future__ import annotations

import re
import sys
from pathlib import Path
from textwrap import wrap

from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


FONT_CANDIDATES = [
    (Path(r"C:\Windows\Fonts\consola.ttf"), "Consolas"),
    (Path(r"C:\Windows\Fonts\cour.ttf"), "CourierNew"),
]

REPLACEMENTS = {
    "✅": "[OK]",
    "❌": "[ERR]",
    "⚠️": "[WARN]",
    "⚠": "[WARN]",
    "ℹ️": "[INFO]",
    "ℹ": "[INFO]",
    "💾": "[SAVE]",
    "📥": "[LOAD]",
    "📋": "[LIST]",
    "📡": "[NET]",
    "🔌": "[PIN]",
    "🔎": "[SCAN]",
    "→": "->",
    "…": "...",
}


def register_font() -> str:
    for font_path, font_name in FONT_CANDIDATES:
        if font_path.exists():
            pdfmetrics.registerFont(TTFont(font_name, str(font_path)))
            return font_name
    return "Courier"


def sanitize_text(text: str) -> str:
    for source, target in REPLACEMENTS.items():
        text = text.replace(source, target)

    text = text.replace("\t", "    ")
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return text


def wrap_line(line: str, font_name: str, font_size: int, max_width: float) -> list[str]:
    if not line:
        return [""]

    if pdfmetrics.stringWidth(line, font_name, font_size) <= max_width:
        return [line]

    indent = re.match(r"^\s*", line).group(0)
    available = max_width
    approx_char_width = max(pdfmetrics.stringWidth("M", font_name, font_size), 1)
    approx_chars = max(int(available / approx_char_width), 20)

    wrapped = wrap(
        line,
        width=approx_chars,
        replace_whitespace=False,
        drop_whitespace=False,
        subsequent_indent=indent,
        break_long_words=False,
        break_on_hyphens=False,
    )

    if not wrapped:
        return [line]

    output: list[str] = []
    for part in wrapped:
        if pdfmetrics.stringWidth(part, font_name, font_size) <= max_width:
            output.append(part)
            continue

        current = ""
        for char in part:
            candidate = current + char
            if current and pdfmetrics.stringWidth(candidate, font_name, font_size) > max_width:
                output.append(current)
                current = indent + char if current.startswith(indent) else char
            else:
                current = candidate
        if current:
            output.append(current)

    return output


def export_markdown_to_pdf(source_path: Path, output_path: Path) -> None:
    font_name = register_font()
    font_size = 8
    left_margin = 36
    right_margin = 36
    top_margin = 40
    bottom_margin = 40
    line_height = 11

    page_width, page_height = A4
    max_width = page_width - left_margin - right_margin

    raw_text = source_path.read_text(encoding="utf-8")
    text = sanitize_text(raw_text)
    lines = text.split("\n")

    pdf = canvas.Canvas(str(output_path), pagesize=A4)
    pdf.setTitle(source_path.stem)

    def start_page(page_number: int) -> float:
        pdf.setFont(font_name, font_size)
        pdf.drawRightString(page_width - right_margin, page_height - 22, f"Pagina {page_number}")
        return page_height - top_margin

    page_number = 1
    y = start_page(page_number)

    for line in lines:
        wrapped_lines = wrap_line(line, font_name, font_size, max_width)

        for wrapped in wrapped_lines:
            if y <= bottom_margin:
                pdf.showPage()
                page_number += 1
                y = start_page(page_number)

            pdf.drawString(left_margin, y, wrapped)
            y -= line_height

    pdf.save()


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print("Uso: python export_markdown_pdf.py <entrada.md> [salida.pdf]")
        return 1

    source_path = Path(sys.argv[1]).resolve()
    if not source_path.exists():
        print(f"No existe el fichero de entrada: {source_path}")
        return 1

    if len(sys.argv) == 3:
        output_path = Path(sys.argv[2]).resolve()
    else:
        output_path = source_path.with_suffix(".pdf")

    export_markdown_to_pdf(source_path, output_path)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
