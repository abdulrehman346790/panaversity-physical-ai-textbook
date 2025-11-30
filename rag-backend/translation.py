from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, List
import os
import google.generativeai as genai
from translation_cache import get_cached_translation, set_cached_translation
from markdown_parser import extract_frontmatter, extract_code_blocks, extract_inline_code, restore_elements

router = APIRouter()

# Initialize Gemini client
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-2.5-flash')

class TranslationRequest(BaseModel):
    text: str
    target_lang: str = "ur"
    format: str = "markdown"  # markdown or text

class TranslationResponse(BaseModel):
    translated_text: str
    source_lang: str
    target_lang: str
    cached: bool = False

def translate_with_gemini(text: str, target_lang: str) -> str:
    """
    Translates text using Gemini API.
    """
    if not text.strip():
        return ""
        
    try:
        prompt = f"""You are a professional translator. Translate the following text to {target_lang}. 
        Preserve all Markdown formatting, special tokens, and placeholders (like __CODE_BLOCK_0__). 
        Do not translate technical terms like 'ROS 2', 'Python', 'Node', 'Topic', 'Service'.
        
        Text to translate:
        {text}
        """
        
        response = model.generate_content(prompt)
        return response.text.strip()
    except Exception as e:
        print(f"Gemini API Error: {e}")
        raise e

def translate_markdown(content: str, target_lang: str) -> str:
    """
    Full pipeline: Parse -> Translate -> Restore -> Cache
    """
    # Check cache first
    cached = get_cached_translation(content, target_lang)
    if cached:
        return cached

    # 1. Parse Markdown and extract protected elements
    frontmatter, body = extract_frontmatter(content)
    body_masked, code_blocks = extract_code_blocks(body)
    body_masked, inline_code = extract_inline_code(body_masked)
    
    # 2. Translate the masked body
    translated_body = translate_with_gemini(body_masked, target_lang)
    
    # 3. Restore protected elements
    final_text = restore_elements(translated_body, code_blocks, inline_code)
    
    # 4. Re-attach frontmatter if it existed
    if frontmatter:
        import yaml
        frontmatter_str = yaml.dump(frontmatter, allow_unicode=True, default_flow_style=False)
        final_text = f"---\n{frontmatter_str}---\n{final_text}"

    # 5. Cache the result
    set_cached_translation(content, target_lang, final_text)
    
    return final_text

@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Translate text to target language with Markdown preservation and caching.
    """
    try:
        final_text = translate_markdown(request.text, request.target_lang)
        
        # Check if it was cached (simple check by comparing with cache get)
        # Optimization: translate_markdown could return tuple (text, cached)
        # For now, we just return the text
        
        return TranslationResponse(
            translated_text=final_text,
            source_lang="auto",
            target_lang=request.target_lang,
            cached=False # We don't track this in the simple refactor yet
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")

@router.post("/translate/batch")
async def translate_batch(texts: List[str], target_lang: str = "ur"):
    """
    Translate multiple texts (simple batch wrapper)
    """
    results = []
    for text in texts:
        try:
            # Reuse the single translation logic (could be optimized for batch API calls)
            # For MVP, loop is fine as we rely on cache
            req = TranslationRequest(text=text, target_lang=target_lang)
            res = await translate_text(req)
            results.append({
                "original": text,
                "translated": res.translated_text,
                "cached": res.cached
            })
        except Exception as e:
            results.append({
                "original": text,
                "error": str(e)
            })
            
    return {"translations": results}
