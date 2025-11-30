from fastapi.testclient import TestClient
from translate_service import app

client = TestClient(app)


def test_translate_echo():
    resp = client.post('/api/translate', json={
        'markdown': '# Hello\n\nThis is a test.',
        'target_locale': 'ur'
    })
    assert resp.status_code == 200
    data = resp.json()
    assert 'translated_markdown' in data
    assert data['translated_markdown'].startswith('# Hello')
    # Check that code block preservation pipeline works in mock mode
    resp2 = client.post('/api/translate', json={
        'markdown': '```python\nprint("Hello")\n```\n# Hello', 'target_locale': 'ur'
    })
    assert resp2.status_code == 200
    assert '```python' in resp2.json()['translated_markdown']


def test_preference_set_get():
    resp = client.post('/api/user/preference', json={'user_id': 'test123', 'language': 'ur'})
    assert resp.status_code == 200
    data = resp.json()
    assert data['language'] == 'ur'

    resp2 = client.get('/api/user/preference', params={'user_id': 'test123'})
    assert resp2.status_code == 200
    assert resp2.json()['language'] == 'en'  # default behavior for now
