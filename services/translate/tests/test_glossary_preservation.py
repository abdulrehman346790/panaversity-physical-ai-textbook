from translate_pipeline import translate_markdown
import yaml


def dummy_translator(text):
    # Simulate a translator that would otherwise translate 'ROS' to 'روبوٹ', but glossary should prevent this
    return text.replace('Hello', 'سلام').replace('ROS', 'روبوٹ')


def test_glossary_preserve():
    md = "This mentions ROS and Hello."
    # For this test, assume glossary maps 'ROS' to 'ROS' (not translated) - simulated by not replacing ROS in translator
    out = translate_markdown(md, dummy_translator)
    # Ensure that 'ROS' uses the Urdu mapping in the final result as defined in glossary
    assert 'آر او ایس' in out
    assert 'سلام' in out
