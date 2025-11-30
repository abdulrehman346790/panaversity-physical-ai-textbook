from translate_pipeline import translate_markdown


def dummy_translator(text):
    return text.replace('Value', 'قیمت')


def test_table_preservation():
    md = "| Column | Value |\n|---|---|\n| Topic | /ros/topic |"
    out = translate_markdown(md, dummy_translator)
    assert '|' in out
    assert '/ros/topic' in out
    assert 'قیمت' in out
