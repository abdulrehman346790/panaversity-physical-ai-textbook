from fastapi.testclient import TestClient
from translate_service import app

client = TestClient(app)


def test_feedback_endpoint():
    resp = client.post('/api/translate/feedback', params={'user_id': '123', 'source_path': '/docs/intro', 'comments': 'Bad translation of ROS term'})
    assert resp.status_code == 200
    assert resp.json().get('status') == 'ok'
*** End Patch