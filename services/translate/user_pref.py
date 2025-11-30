# Simple in-memory user preference store for demo purposes
USER_PREFS = {}

def set_preference(user_id: str, language: str):
    USER_PREFS[user_id] = language
    return True

def get_preference(user_id: str):
    return USER_PREFS.get(user_id, 'en')
