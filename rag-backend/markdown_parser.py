import re
import yaml

def extract_frontmatter(content: str):
    """
    Extracts YAML frontmatter from Markdown content.
    Returns (frontmatter_dict, body_text).
    """
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, content, re.DOTALL)
    
    if match:
        frontmatter_raw = match.group(1)
        try:
            frontmatter = yaml.safe_load(frontmatter_raw)
            body = content[match.end():]
            return frontmatter, body
        except yaml.YAMLError:
            return None, content
    
    return None, content

def extract_code_blocks(content: str):
    """
    Replaces code blocks with placeholders.
    Returns (content_with_placeholders, blocks_map).
    """
    blocks = {}
    counter = 0
    
    def replace_block(match):
        nonlocal counter
        placeholder = f"__CODE_BLOCK_{counter}__"
        blocks[placeholder] = match.group(0)
        counter += 1
        return placeholder

    # Fenced code blocks
    pattern = r'```[\s\S]*?```'
    content = re.sub(pattern, replace_block, content)
    
    return content, blocks

def extract_inline_code(content: str):
    """
    Replaces inline code with placeholders.
    Returns (content_with_placeholders, inline_map).
    """
    inline_map = {}
    counter = 0
    
    def replace_inline(match):
        nonlocal counter
        placeholder = f"__INLINE_CODE_{counter}__"
        inline_map[placeholder] = match.group(0)
        counter += 1
        return placeholder

    # Inline code (backticks)
    pattern = r'`[^`\n]+`'
    content = re.sub(pattern, replace_inline, content)
    
    return content, inline_map

def restore_elements(content: str, blocks_map: dict, inline_map: dict):
    """
    Restores code blocks and inline code from placeholders.
    """
    # Restore inline code first (to avoid conflict with blocks if any)
    for placeholder, original in inline_map.items():
        content = content.replace(placeholder, original)
        
    # Restore blocks
    for placeholder, original in blocks_map.items():
        content = content.replace(placeholder, original)
        
    return content

def apply_glossary(content: str, glossary: dict):
    """
    Applies glossary terms to content.
    For now, this is a placeholder for more complex logic.
    """
    # Simple replacement for now, but ideally should be context-aware
    # or passed as system prompt to LLM
    return content
