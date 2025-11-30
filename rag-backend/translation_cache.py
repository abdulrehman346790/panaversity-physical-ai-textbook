import redis
import json
import hashlib
import os
from typing import Optional

# Initialize Redis client
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_PORT = int(os.getenv("REDIS_PORT", 6379))
REDIS_DB = int(os.getenv("REDIS_DB", 0))

try:
    redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=REDIS_DB, decode_responses=True)
    redis_client.ping()
    print("Connected to Redis for translation cache")
except redis.ConnectionError:
    print("Warning: Redis not available, using in-memory cache fallback")
    redis_client = None

# In-memory fallback
_memory_cache = {}

def get_content_hash(text: str) -> str:
    return hashlib.md5(text.encode('utf-8')).hexdigest()

def get_cached_translation(text: str, target_lang: str) -> Optional[str]:
    content_hash = get_content_hash(text)
    key = f"trans:{target_lang}:{content_hash}"
    
    if redis_client:
        try:
            return redis_client.get(key)
        except redis.RedisError:
            pass
            
    return _memory_cache.get(key)

def set_cached_translation(text: str, target_lang: str, translation: str, ttl: int = 2592000):
    content_hash = get_content_hash(text)
    key = f"trans:{target_lang}:{content_hash}"
    
    if redis_client:
        try:
            redis_client.setex(key, ttl, translation)
            return
        except redis.RedisError:
            pass
            
    _memory_cache[key] = translation

def get_cache_stats():
    if redis_client:
        try:
            info = redis_client.info()
            return {
                "type": "redis",
                "used_memory_human": info.get("used_memory_human"),
                "connected_clients": info.get("connected_clients")
            }
        except:
            pass
            
    return {
        "type": "memory",
        "items": len(_memory_cache)
    }
