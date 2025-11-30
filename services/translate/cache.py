"""
Cache wrapper for translation service. Uses Redis if available, fallback to in-memory dictionary.
"""
from typing import Optional

try:
    import redis
except Exception:
    redis = None

CACHE = {}

class Cache:
    def __init__(self, redis_url: Optional[str] = None):
        self.redis_url = redis_url
        self.client = None
        if redis and redis_url:
            try:
                self.client = redis.from_url(redis_url)
            except Exception:
                self.client = None

    def get(self, key: str):
        if self.client:
            return self.client.get(key)
        return CACHE.get(key)

    def set(self, key: str, value, ex: Optional[int] = None):
        if self.client:
            self.client.set(key, value, ex=ex)
            return
        CACHE[key] = value

cache = Cache()
