from translate_pipeline import translate_markdown


def dummy_translator(text):
    # Replace 'Hello' with Urdu 'سلام' for demonstration
    return text.replace('Hello', 'سلام')


def test_code_block_preserved():
    md = "# Hello\n\nThis is a sample.\n\n```python\nprint('Hello')\n```\n"
    out = translate_markdown(md, dummy_translator)
    assert '```python' in out
    assert "print('Hello')" in out  # code block must be preserved
    assert 'سلام' in out  # headings translated


def test_yaml_frontmatter_preserved():
    md = "---\ntitle: Hello\nslug: /hello\n---\n\nHello world"
    out = translate_markdown(md, dummy_translator)
    assert 'title: Hello' in out  # frontmatter unchanged
    assert 'سلام' in out
