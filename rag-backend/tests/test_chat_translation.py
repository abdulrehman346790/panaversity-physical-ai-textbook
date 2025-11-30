from fastapi.testclient import TestClient
from main import app
from auth import create_access_token

client = TestClient(app)


def test_chat_translation_flow():
    # Create a token for user id '1' using create_access_token helper
    token = create_access_token({"sub": "1", "email": "test@example.com"})

    # Set user preference to Urdu
    resp = client.post('/api/user/lang', params={'token': token, 'language': 'ur'})
    assert resp.status_code == 200

    # Send a chat request
    resp = client.post('/chat', json={'message': 'Explain ROS 2 nodes', 'token': token})
    assert resp.status_code == 200
    data = resp.json()
    assert 'response' in data
    # For now, we expect translation integration to be called, so the response should be non-empty
    assert len(data['response']) > 0
    assert data.get('translated', False) is True
