#!/usr/bin/env python3
"""
fix_xml_encoding.py
===================
Script untuk mendeteksi dan memperbaiki semua karakter bermasalah
di file Xacro/URDF/SDF/XML untuk ROS2 + Gazebo.

Penyebab ExpatError "not well-formed (invalid token)":
  1. Karakter Unicode non-ASCII di dalam komentar XML (U+2500, U+2192, dll)
  2. Double-dash "--" di dalam komentar XML <!-- --> (dilarang XML 1.0 spec)
  3. Karakter kontrol tersembunyi (NUL, BEL, dll)

Penggunaan:
  python3 fix_xml_encoding.py <file.xacro>
  python3 fix_xml_encoding.py <file.xacro> --fix   (tulis file baru)
"""

import sys
import re
import xml.etree.ElementTree as ET

# Peta replacement karakter Unicode ke ASCII
UNICODE_MAP = {
    '\u2014': '--',   '\u2013': '-',
    '\u2500': '-',    '\u2501': '-',    '\u2502': '|',
    '\u2550': '=',    '\u2551': '|',
    '\u2192': '->',   '\u2190': '<-',   '\u2191': '^',    '\u2193': 'v',
    '\u21D2': '=>',   '\u21D0': '<=',
    '\u2713': 'OK',   '\u2717': 'NO',   '\u2714': 'OK',
    '\u03B1': 'alpha','\u03B2': 'beta', '\u03B3': 'gamma',
    '\u03B4': 'delta','\u03B8': 'theta','\u03C4': 'tau',
    '\u03C9': 'omega','\u03C3': 'sigma','\u03BC': 'mu',
    '\u00B2': '2',    '\u00B3': '3',    '\u00B9': '1',
    '\u00B7': '*',    '\u00D7': 'x',    '\u00F7': '/',
    '\u2248': '~=',   '\u2260': '!=',   '\u2261': '==',
    '\u2264': '<=',   '\u2265': '>=',
    '\u00B0': 'deg',  '\u2212': '-',
}

def scan_file(filepath):
    """Scan file dan laporkan semua masalah tanpa mengubah file."""
    issues = []
    with open(filepath, 'rb') as f:
        raw = f.read()

    # Decode
    try:
        content = raw.decode('utf-8')
    except UnicodeDecodeError as e:
        return [f"DECODE ERROR: {e}"]

    lines = content.splitlines(keepends=True)

    # 1. Cek karakter non-ASCII
    for lineno, line in enumerate(lines, 1):
        for col, ch in enumerate(line, 1):
            if ord(ch) > 127:
                replacement = UNICODE_MAP.get(ch, f'?U+{ord(ch):04X}?')
                issues.append(
                    f"Line {lineno:4d} Col {col:3d}: "
                    f"U+{ord(ch):04X} {repr(ch):12s} -> '{replacement}'"
                )

    # 2. Cek double-dash di dalam komentar
    for m in re.finditer(r'<!--(.*?)-->', content, re.DOTALL):
        inner = m.group(1)
        if '--' in inner:
            lineno = content[:m.start()].count('\n') + 1
            issues.append(
                f"Line {lineno:4d}: Double-dash '--' di dalam komentar XML "
                f"(INVALID per XML 1.0 spec)"
            )

    # 3. Cek karakter kontrol illegal
    for lineno, line in enumerate(lines, 1):
        for col, ch in enumerate(line, 1):
            cp = ord(ch)
            if cp < 0x20 and ch not in ('\t', '\n', '\r'):
                issues.append(
                    f"Line {lineno:4d} Col {col:3d}: "
                    f"Karakter kontrol ILLEGAL U+{cp:04X}"
                )

    # 4. Coba parse XML
    try:
        ET.fromstring(content)
    except ET.ParseError as e:
        issues.insert(0, f"XML PARSE ERROR: {e}")

    return issues


def fix_file(filepath, output_path=None):
    """Fix semua masalah dan tulis ke file baru."""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Fix 1: Ganti karakter Unicode
    for char, replacement in UNICODE_MAP.items():
        content = content.replace(char, replacement)

    # Fix 2: Ganti karakter Unicode yang tidak ada di map
    result = []
    for ch in content:
        if ord(ch) > 127:
            result.append(f'?U{ord(ch):04X}?')
        else:
            result.append(ch)
    content = ''.join(result)

    # Fix 3: Ganti -- di dalam komentar
    def fix_comment(match):
        inner = match.group(1)
        inner = re.sub(r'--(?!>)', '- ', inner)
        return '<!--' + inner + '-->'

    content = re.sub(r'<!--(.*?)-->', fix_comment, content, flags=re.DOTALL)

    # Fix 4: Hapus karakter kontrol illegal
    content = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F]', '', content)

    # Verifikasi
    try:
        ET.fromstring(content)
        print("[OK] XML valid setelah fix")
    except ET.ParseError as e:
        print(f"[WARN] Masih ada XML error setelah fix: {e}")

    out = output_path or filepath.replace('.xacro', '_fixed.xacro')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"[OK] File ditulis: {out}")
    return out


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    filepath = sys.argv[1]
    do_fix = '--fix' in sys.argv

    print(f"Scanning: {filepath}")
    issues = scan_file(filepath)

    if not issues:
        print("[OK] Tidak ada masalah ditemukan")
    else:
        print(f"\n[FOUND] {len(issues)} masalah:")
        for issue in issues[:50]:  # tampilkan max 50
            print(f"  {issue}")
        if len(issues) > 50:
            print(f"  ... dan {len(issues)-50} masalah lainnya")

    if do_fix and issues:
        print("\n[FIX] Memperbaiki file...")
        out = fix_file(filepath)
        print(f"[DONE] File bersih: {out}")
    elif issues and not do_fix:
        print(f"\nJalankan dengan --fix untuk memperbaiki otomatis:")
        print(f"  python3 fix_xml_encoding.py {filepath} --fix")
