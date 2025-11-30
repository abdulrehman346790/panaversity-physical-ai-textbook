import pytest
from markdown_parser import extract_frontmatter, extract_code_blocks, extract_inline_code, restore_elements

def test_extract_frontmatter():
    content = """---
title: Hello World
description: A test post
---
# Content
This is the body."""
    
    frontmatter, body = extract_frontmatter(content)
    
    assert frontmatter['title'] == 'Hello World'
    assert frontmatter['description'] == 'A test post'
    assert body.strip() == "# Content\nThis is the body."

def test_extract_frontmatter_none():
    content = "# Just content\nNo frontmatter here."
    frontmatter, body = extract_frontmatter(content)
    assert frontmatter is None
    assert body == content

def test_extract_code_blocks():
    content = """Here is some code:
```python
def hello():
    print("world")
```
And more text."""
    
    masked, blocks = extract_code_blocks(content)
    
    assert "__CODE_BLOCK_0__" in masked
    assert "def hello():" not in masked
    assert blocks["__CODE_BLOCK_0__"] == '```python\ndef hello():\n    print("world")\n```'

def test_extract_inline_code():
    content = "Use `print()` to output text."
    
    masked, inline = extract_inline_code(content)
    
    assert "__INLINE_CODE_0__" in masked
    assert "`print()`" not in masked
    assert inline["__INLINE_CODE_0__"] == "`print()`"

def test_restore_elements():
    content = "Start __CODE_BLOCK_0__ Middle __INLINE_CODE_0__ End"
    blocks = {"__CODE_BLOCK_0__": "```BLOCK```"}
    inline = {"__INLINE_CODE_0__": "`INLINE`"}
    
    restored = restore_elements(content, blocks, inline)
    
    assert restored == "Start ```BLOCK``` Middle `INLINE` End"

def test_full_pipeline_simulation():
    original = """---
title: Test
---
# Heading
Check this `code`:
```javascript
console.log('hi');
```
End."""

    # 1. Extract frontmatter
    fm, body = extract_frontmatter(original)
    assert fm['title'] == 'Test'
    
    # 2. Extract blocks
    body_masked, blocks = extract_code_blocks(body)
    assert "console.log" not in body_masked
    
    # 3. Extract inline
    body_masked, inline = extract_inline_code(body_masked)
    assert "`code`" not in body_masked
    
    # Simulate translation (just upper casing the text parts)
    translated_body = body_masked.replace("Heading", "HEADING").replace("Check this", "CHECK THIS").replace("End", "END")
    
    # 4. Restore
    final = restore_elements(translated_body, blocks, inline)
    
    assert "HEADING" in final
    assert "CHECK THIS" in final
    assert "`code`" in final
    assert "console.log('hi')" in final
    assert "END" in final
