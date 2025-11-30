from fastapi.testclient import TestClient
from translate_service import app

client = TestClient(app)


def test_feedback_endpoint_post():
    # Test that feedback logs are accepted
    resp = client.post('/api/translate/feedback', params={'user_id': 'abc', 'source_path': '/docs/intro', 'comments': 'Poor translation'})
    assert resp.status_code == 200
    assert resp.json().get('status') == 'ok'
*** End Patch