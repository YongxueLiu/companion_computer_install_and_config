#!/usr/bin/env python3
"""Generate WeChat-pasteable HTML from Markdown using codecogs PNG math images.
Style matches 11_ekf2_magnetometer_fusion_wechat.html.
"""
import argparse
import base64
import re
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path


def get_title(md_text: str) -> str:
    m = re.search(r'^#\s+(.+)$', md_text, re.MULTILINE)
    return m.group(1).strip() if m else ''


def md_to_html_body(md_path: Path) -> str:
    result = subprocess.run(
        [
            'pandoc', str(md_path),
            '--from', 'markdown+tex_math_dollars',
            '--to', 'html',
            '--webtex=https://latex.codecogs.com/png.latex?',
            '--toc',
            '--toc-depth=3',
            '-M', 'lang=zh-CN',
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def md_text_to_html_body(md_text: str) -> str:
    result = subprocess.run(
        [
            'pandoc',
            '--from', 'markdown+tex_math_dollars',
            '--to', 'html',
            '--webtex=https://latex.codecogs.com/png.latex?',
            '--toc',
            '--toc-depth=3',
            '-M', 'lang=zh-CN',
        ],
        input=md_text,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def convert_inline_math_to_display(md_text: str) -> str:
    """Convert $...$ inline math to standalone $$...$$ display math blocks.

    WeChat's editor is unreliable for inline formula images (base64 or
    external); block-level formula images paste much more consistently.
    Each converted formula is placed on its own line so pandoc renders it
    as a separate centered paragraph.
    Code blocks and already-display math are preserved.
    """
    code_blocks: list[str] = []
    display_math: list[str] = []

    def protect_code(m: re.Match) -> str:
        code_blocks.append(m.group(0))
        return f'__PROTECTED_CODE_{len(code_blocks) - 1}__'

    def protect_display(m: re.Match) -> str:
        display_math.append(m.group(0))
        return f'__PROTECTED_DISPLAY_{len(display_math) - 1}__'

    # Protect fenced code blocks and inline code
    text = re.sub(r'```[\s\S]*?```', protect_code, md_text)
    text = re.sub(r'`[^`]+`', protect_code, text)
    # Protect display math $$...$$
    text = re.sub(r'\$\$[\s\S]*?\$\$', protect_display, text)

    # Convert inline math $...$ to standalone $$...$$ blocks
    text = re.sub(
        r'(?<!\$)\$([^$\n]+?)\$(?!\$)',
        r'\n\n$$\1$$\n\n',
        text,
    )

    # Restore protected blocks
    for i, s in enumerate(display_math):
        text = text.replace(f'__PROTECTED_DISPLAY_{i}__', s)
    for i, s in enumerate(code_blocks):
        text = text.replace(f'__PROTECTED_CODE_{i}__', s)

    return text


def _escape_underscores_in_text(latex: str) -> str:
    """codecogs renders \text{...} in a restricted mode where bare underscores fail.
    Escape unescaped underscores inside every \text{...} block.
    """
    pattern = re.compile(r'\\text\{([^}]*)\}')

    def repl(m):
        content = m.group(1)
        # Escape underscores that are not already escaped
        content = re.sub(r'(?<!\\)_', r'\\_', content)
        return f'\\text{{{content}}}'

    return pattern.sub(repl, latex)


def _fix_codecogs_url(url: str) -> str:
    """Fix common codecogs incompatibilities in the query string of a latex.codecogs.com URL."""
    if 'latex.codecogs.com' not in url:
        return url
    prefix, sep, query = url.partition('?')
    if not sep:
        return url
    latex = urllib.parse.unquote(query)
    latex = _escape_underscores_in_text(latex)
    return prefix + sep + urllib.parse.quote(latex, safe='')


def _fetch_codecogs_image(url: str) -> bytes | None:
    """Download a single codecogs PNG; return None on failure."""
    try:
        data = urllib.request.urlopen(url, timeout=20).read()
        if len(data) < 100 or b'Invalid Equation' in data:
            return None
        return data
    except Exception as e:
        print(f'Warning: failed to embed {url}: {e}', file=sys.stderr)
        return None


def _collect_codecogs_urls(html: str) -> set[str]:
    codecogs_pattern = re.compile(
        r'(<img\s+[^>]*src=")https://latex\.codecogs\.com/png\.latex\?[^"]+("[^>]*>)',
        re.IGNORECASE,
    )
    urls = set()
    for m in codecogs_pattern.finditer(html):
        prefix = m.group(1)
        suffix = m.group(2)
        full_url = m.group(0)[len(prefix):-len(suffix)]
        urls.add(full_url)
    return urls


def _build_url_to_base64_cache(urls: set[str]) -> dict[str, str]:
    from concurrent.futures import ThreadPoolExecutor

    cache: dict[str, str] = {}
    with ThreadPoolExecutor(max_workers=8) as executor:
        future_to_url = {
            executor.submit(_fetch_codecogs_image, url): url
            for url in urls
        }
        for future in future_to_url:
            url = future_to_url[future]
            data = future.result()
            if data is None:
                cache[url] = url
            else:
                b64 = base64.b64encode(data).decode('ascii')
                cache[url] = f'data:image/png;base64,{b64}'
    return cache


def embed_math_images(html: str) -> str:
    """Download all codecogs formula PNGs and inline them as base64 data URIs."""
    urls = _collect_codecogs_urls(html)
    cache = _build_url_to_base64_cache(urls)

    codecogs_pattern = re.compile(
        r'(<img\s+[^>]*src=")https://latex\.codecogs\.com/png\.latex\?[^"]+("[^>]*>)',
        re.IGNORECASE,
    )

    def repl(m):
        prefix = m.group(1)
        suffix = m.group(2)
        full_url = m.group(0)[len(prefix):-len(suffix)]
        return prefix + cache[full_url] + suffix

    return codecogs_pattern.sub(repl, html)


def embed_math_images_hybrid(html: str) -> str:
    """Embed base64 only for display formula images; keep inline formulas as codecogs URLs.

    Display formulas (centered, large) are the ones most likely to fail when
    fetched by WeChat from codecogs. Inline formulas often survive as external
    URLs, so this hybrid keeps the inline layout while hardening display math.
    """
    urls = _collect_codecogs_urls(html)
    cache = _build_url_to_base64_cache(urls)

    # Replace only display-style images (max-width:100%;display:block;margin:10px auto;)
    display_pattern = re.compile(
        r'(<img\s+src=")https://latex\.codecogs\.com/png\.latex\?[^"]+("\s+alt="[^"]*"\s+title="[^"]*"\s+style="max-width:100%;display:block;margin:10px auto;">)',
        re.IGNORECASE,
    )

    def repl(m):
        prefix = m.group(1)
        suffix = m.group(2)
        full_url = m.group(0)[len(prefix):-len(suffix)]
        return prefix + cache[full_url] + suffix

    return display_pattern.sub(repl, html)


def apply_inlineblock_style(html: str) -> str:
    """Change inline formula style from display:inline to display:inline-block."""
    return html.replace(
        'style="vertical-align:middle;display:inline;margin:0 2px;height:1.2em;"',
        'style="vertical-align:middle;display:inline-block;margin:0 2px;height:1.2em;width:auto;"',
    )


def wrap_inline_math_in_tables(html: str) -> str:
    """Wrap paragraphs containing inline formula images into single-row tables.

    WeChat handles block-level formula images reliably but drops inline formula
    images. This keeps the visual flow by placing each inline formula image in
    its own table cell (where it is block-level) while keeping surrounding text
    in adjacent cells.
    """
    from bs4 import BeautifulSoup

    INLINE_STYLE = 'vertical-align:middle;display:inline;margin:0 2px;height:1.2em;'
    soup = BeautifulSoup(html, 'html.parser')

    for p in list(soup.find_all('p')):
        inline_imgs = p.find_all('img', style=INLINE_STYLE)
        if not inline_imgs:
            continue

        table = soup.new_tag('table', style='border:none;width:100%;margin:0;')
        tr = soup.new_tag('tr', style='border:none;')
        table.append(tr)

        # Collect segments: each segment is a list of children (strings/elements)
        current_segment = []

        def flush_segment():
            if not current_segment:
                return
            # Check if segment is only whitespace
            text_only = all(isinstance(c, str) for c in current_segment)
            if text_only and ''.join(current_segment).strip() == '':
                current_segment.clear()
                return
            td = soup.new_tag('td', style='border:none;padding:0;vertical-align:middle;')
            for child in current_segment:
                td.append(child)
            tr.append(td)
            current_segment.clear()

        for child in list(p.contents):
            if getattr(child, 'name', None) == 'img' and child.get('style') == INLINE_STYLE:
                flush_segment()
                # Convert inline image to block-style inside table cell
                child['style'] = 'max-width:100%;display:block;margin:0 auto;vertical-align:middle;'
                td = soup.new_tag('td', style='border:none;padding:0;vertical-align:middle;text-align:center;')
                td.append(child)
                tr.append(td)
            else:
                current_segment.append(child)

        flush_segment()
        p.replace_with(table)

    return str(soup)


def post_process_math_images(html: str) -> str:
    # 1. Display math: pandoc wraps as <p><br /><img style="vertical-align:middle" src="..." /><br /></p>
    # Replace with <p align="center"><img src="..." style="max-width:100%;display:block;margin:10px auto;"></p>
    display_pattern = re.compile(
        r'<p>\s*<br\s*/?>\s*<img\s+style="vertical-align:middle"\s+src="([^"]+)"\s+alt="([^"]*)"\s+title="([^"]*)"\s*/?>\s*<br\s*/?>\s*</p>',
        re.IGNORECASE,
    )

    def display_repl(m):
        src = _fix_codecogs_url(m.group(1))
        alt = m.group(2)
        title = m.group(3)
        return (
            f'<p align="center">'
            f'<img src="{src}" alt="{alt}" title="{title}" '
            f'style="max-width:100%;display:block;margin:10px auto;">'
            f'</p>'
        )

    html = display_pattern.sub(display_repl, html)

    # 2. Inline math: <img style="vertical-align:middle" src="..." alt="..." title="..." />
    # Replace style with inline-friendly style
    inline_pattern = re.compile(
        r'<img\s+style="vertical-align:middle"\s+src="([^"]+)"\s+alt="([^"]*)"\s+title="([^"]*)"\s*/?>',
        re.IGNORECASE,
    )

    def inline_repl(m):
        src = _fix_codecogs_url(m.group(1))
        alt = m.group(2)
        title = m.group(3)
        return (
            f'<img src="{src}" alt="{alt}" title="{title}" '
            f'style="vertical-align:middle;display:inline;margin:0 2px;height:1.2em;">'
        )

    html = inline_pattern.sub(inline_repl, html)
    return html


def wrap_html(title: str, body: str) -> str:
    return f'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, "Noto Sans SC", "PingFang SC", "Microsoft YaHei", sans-serif; font-size: 16px; line-height: 1.8; color: #333; max-width: 800px; margin: 0 auto; padding: 20px; background: #fff; }}
h1 {{ font-size: 24px; font-weight: bold; color: #1a1a1a; margin: 30px 0 20px; padding-bottom: 10px; border-bottom: 2px solid #e0e0e0; }}
h2 {{ font-size: 20px; font-weight: bold; color: #2c2c2c; margin: 25px 0 15px; }}
h3 {{ font-size: 18px; font-weight: bold; color: #3a3a3a; margin: 20px 0 10px; }}
h4 {{ font-size: 16px; font-weight: bold; color: #444; margin: 15px 0 8px; }}
p {{ margin: 12px 0; text-align: justify; }}
table {{ border-collapse: collapse; width: 100%; margin: 15px 0; font-size: 14px; }}
th, td {{ border: 1px solid #ddd; padding: 10px; text-align: left; }}
th {{ background: #f5f5f5; font-weight: bold; }}
tr:nth-child(even) {{ background: #fafafa; }}
code {{ background: #f4f4f4; padding: 2px 6px; border-radius: 3px; font-family: "Courier New", monospace; font-size: 14px; }}
pre {{ background: #f8f8f8; padding: 15px; border-radius: 5px; overflow-x: auto; font-size: 13px; line-height: 1.5; }}
blockquote {{ border-left: 4px solid #ddd; margin: 15px 0; padding: 10px 15px; background: #f9f9f9; color: #555; }}
hr {{ border: none; border-top: 1px solid #e0e0e0; margin: 20px 0; }}
img {{ max-width: 100%; height: auto; }}
ul, ol {{ margin: 10px 0; padding-left: 25px; }}
li {{ margin: 5px 0; }}
#TOC {{ background: #f9f9f9; padding: 15px; border-radius: 5px; margin-bottom: 20px; }}
#TOC ul {{ padding-left: 20px; }}
#TOC a {{ color: #07c; text-decoration: none; }}
</style>
</head>
<body>

<header id="title-block-header">
<h1 class="title">{title}</h1>
</header>
{body}
</body>
</html>
'''


def generate_wechat_html(
    md_path: Path,
    mode: str,
    out_path: Path | None = None,
) -> Path:
    """Generate a WeChat HTML file using the specified rendering mode.

    Modes:
      external:  codecogs PNG URLs (original behavior)
      block:     base64 embed + inline math promoted to display math
      inline:    base64 embed while keeping inline math layout
      inlineblock: base64 embed + inline math with display:inline-block
      table:     base64 embed + inline math wrapped in single-row tables
      hybrid:    base64 embed for display math only, inline math stays external
    """
    valid_modes = {'external', 'block', 'inline', 'inlineblock', 'table', 'hybrid'}
    if mode not in valid_modes:
        raise ValueError(f'Unknown mode: {mode}. Choose from {valid_modes}')

    md_text = md_path.read_text(encoding='utf-8')
    title = get_title(md_text)

    if mode == 'block':
        md_text = convert_inline_math_to_display(md_text)
        body = md_text_to_html_body(md_text)
    else:
        body = md_to_html_body(md_path)

    body = post_process_math_images(body)

    if mode == 'block':
        body = embed_math_images(body)
    elif mode == 'inline':
        body = embed_math_images(body)
    elif mode == 'inlineblock':
        body = embed_math_images(body)
        body = apply_inlineblock_style(body)
    elif mode == 'table':
        body = embed_math_images(body)
        body = wrap_inline_math_in_tables(body)
    elif mode == 'hybrid':
        body = embed_math_images_hybrid(body)

    if out_path is None:
        out_path = md_path.with_suffix('').with_name(md_path.stem + '_wechat.html')
    out_path.write_text(wrap_html(title, body), encoding='utf-8')
    return out_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate WeChat-pasteable HTML from Markdown.'
    )
    parser.add_argument('md_path', help='Path to the Markdown file')
    parser.add_argument(
        '--mode',
        choices=['external', 'block', 'inline', 'inlineblock', 'table', 'hybrid'],
        default='external',
        help='Rendering mode for formula images (default: external)',
    )
    parser.add_argument(
        '--embed-math',
        action='store_true',
        help='Shortcut for --mode block (fallback when external images fail)',
    )
    args = parser.parse_args()

    md_path = Path(args.md_path)
    if not md_path.exists():
        print(f'File not found: {md_path}', file=sys.stderr)
        sys.exit(1)

    mode = 'block' if args.embed_math else args.mode
    out_path = generate_wechat_html(md_path, mode)
    print(f'Generated: {out_path}')


if __name__ == '__main__':
    main()
