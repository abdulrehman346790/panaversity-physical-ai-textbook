import httpx

TRANSLATE_SERVICE_URL = 'http://localhost:8001'

async def translate_text_via_service(text: str, target_locale: str='ur') -> str:
    async with httpx.AsyncClient() as client:
        resp = await client.post(f"{TRANSLATE_SERVICE_URL}/api/translate", json={
            'markdown': text,
            'target_locale': target_locale
        }, timeout=20)
        resp.raise_for_status()
        data = resp.json()
        return data.get('translated_markdown') or data.get('translated_text') or text
