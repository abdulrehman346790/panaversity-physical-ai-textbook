import re
import os
from typing import List, Tuple
import yaml

# Very simple markdown-aware pipeline: split out code fences
CODE_FENCE_RE = re.compile(r'(```[\s\S]*?```)', re.MULTILINE)
GLOSSARY_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'specs', 'feature-urdu-translation', 'glossary', 'terms.yaml'))
GLOSSARY = []
if os.path.exists(GLOSSARY_FILE):
    try:
        with open(GLOSSARY_FILE, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or []
            # Each entry is a map with term_en and optionally term_ur
            for entry in data:
                if entry and 'term_en' in entry:
                    GLOSSARY.append({
                        'term_en': entry['term_en'],
                        'term_ur': entry.get('term_ur', None)
                    })
    except Exception:
        GLOSSARY = []


def extract_code_blocks(md: str) -> Tuple[str, List[str]]:
    """Extract code fences and replace them with placeholders."""
    code_blocks = CODE_FENCE_RE.findall(md)
    placeholder_md = CODE_FENCE_RE.sub('<<CODE_BLOCK>>', md)
    return placeholder_md, code_blocks


def reinsert_code_blocks(md: str, code_blocks: List[str]) -> str:
    """Reinsert the code blocks back into placeholder positions."""
    parts = md.split('<<CODE_BLOCK>>')
    out = []
    for i, p in enumerate(parts):
        out.append(p)
        if i < len(code_blocks):
            out.append(code_blocks[i])
    return ''.join(out)


def translate_markdown(md: str, translate_fn) -> str:
    """Translate markdown while preserving code blocks using translate_fn(text)->translation"""
    placeholder_md, code_blocks = extract_code_blocks(md)
    # Protect glossary terms by replacing with placeholders
    glossary_placeholders = {}
    for i, entry in enumerate(GLOSSARY):
        term = entry.get('term_en')
        placeholder = f'<<GLOSSARY_{i}>>'
        # Use word boundary replace
        pattern = re.compile(rf'\b{re.escape(term)}\b')
        if pattern.search(placeholder_md):
            glossary_placeholders[placeholder] = entry
            placeholder_md = pattern.sub(placeholder, placeholder_md)

    # For simplicity, send the whole placeholder_md to translate_fn
    translated_placeholder = translate_fn(placeholder_md)

    # Reinsert code blocks
    final = reinsert_code_blocks(translated_placeholder, code_blocks)
    # Reinsert glossary terms
    for placeholder, entry in glossary_placeholders.items():
        # If glossary has a target translation, use it; otherwise, use the original English
        term_en = entry.get('term_en')
        term_ur = entry.get('term_ur')
        final = final.replace(placeholder, term_ur if term_ur else term_en)
    return final


if __name__ == '__main__':
    # Example run with a dummy translator
    def dummy_translator(text):
        return text.replace('Hello', 'سلام')

    sample = "# Hello\n\nThis is sample.\n```python\nprint('Hello')\n```"
    out = translate_markdown(sample, dummy_translator)
    print(out)
