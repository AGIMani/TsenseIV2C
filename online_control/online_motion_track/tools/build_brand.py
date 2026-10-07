"""Author vector artwork from the supplied TsenseI reference; no raster upscaling.

Developer-only fontTools utility. Generated SVG paths need no fonts at runtime.
This is a reconstruction, not an original brand-master export.
"""
from pathlib import Path
from fontTools.ttLib import TTFont
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen

DEST = Path(__file__).resolve().parents[1] / "static/brand"
DEST.mkdir(parents=True, exist_ok=True)

BIRD = '''<g fill="none" stroke-linecap="round" stroke-linejoin="round">
<path d="M27 122 43 114 63 117 115 178 187 97 336 24 314 68 275 104 348 64 330 102 267 148 338 111 326 137 286 165 250 204 222 240 184 299 99 299 117 258 72 209 62 169 58 154Z" fill="#e7f6f7" fill-opacity=".3" stroke="#237f93" stroke-width="4.5"/>
<g stroke="#78a4ab" stroke-width="1.6">
<path d="m27 122 31 32 5-37-20-3 19 55 53 9-52-61m-1 52 10 40 43-31 2 80-45-49 27 90 18-41 67 41-18-61 56 2-38 59 66-95-84 34 15-33-64 53 32-47-34-33 48-42 18 69 30-41 39 40-6-58 42 19-19-17 63-46-86 44 31-42 39-36-71 31-1 49-58-51 25 49-46-10 25 69 63-59m-81-49 58-25 49-23-23 29 43-34-29 39-41 23 61-6m-129 18 58 3-55 49 52-20 42 19-21 17-8 39-27-39-46 41m-6 33-17-42 38 9-16-58"/>
</g><circle cx="47" cy="126" r="3.2" fill="#237f93" stroke="none"/>
</g>'''

def lettering(text, font_path, size, x, baseline, spacing=0):
    font = TTFont(font_path, fontNumber=0)
    glyphs, cmap = font.getGlyphSet(), font.getBestCmap()
    scale = size / font['head'].unitsPerEm
    paths = []
    for character in text:
        glyph = glyphs[cmap[ord(character)]]
        pen = SVGPathPen(glyphs)
        glyph.draw(TransformPen(pen, (scale, 0, 0, -scale, x, baseline)))
        paths.append(f'<path d="{pen.getCommands()}"/>')
        x += glyph.width * scale + spacing
    return '<g fill="#151b20">' + ''.join(paths) + '</g>'

logo = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1090 320" role="img">'
        '<title>达显身手 TsenseI ROBOT</title>' + BIRD
        + lettering('达显身手', 'C:/Windows/Fonts/msyhbd.ttc', 126, 360, 222, 45)
        + lettering('TSENSEI ROBOT', 'C:/Windows/Fonts/seguisb.ttf', 49, 365, 293, 19)
        + '</svg>')
(DEST / 'tsensei-logo.svg').write_text(logo, encoding='utf-8')
(DEST / 'tsensei-mark.svg').write_text(
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 370 325"><title>TsenseI</title>'+BIRD+'</svg>', encoding='utf-8')
print(DEST)
