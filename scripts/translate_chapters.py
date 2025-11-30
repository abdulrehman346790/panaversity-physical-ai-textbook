import argparse
import os
import sys
import glob
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rag-backend', '.env'))

# Add rag-backend to path to import translation logic
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(PROJECT_ROOT, 'rag-backend'))

try:
    from translation import translate_markdown
except ImportError:
    print("Error: Could not import translation logic. Make sure rag-backend is in python path.")
    sys.exit(1)

DOCS_DIR = os.path.join(PROJECT_ROOT, 'docusaurus', 'book', 'docs')
I18N_DIR = os.path.join(PROJECT_ROOT, 'docusaurus', 'book', 'i18n', 'ur', 'docusaurus-plugin-content-docs', 'current')

def translate_file(file_path, force=False):
    rel_path = os.path.relpath(file_path, DOCS_DIR)
    target_path = os.path.join(I18N_DIR, rel_path)
    
    if os.path.exists(target_path) and not force:
        print(f"Skipping {rel_path} (already exists). Use --force to overwrite.")
        return

    print(f"Translating {rel_path}...")
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        translated = translate_markdown(content, "ur")
        
        os.makedirs(os.path.dirname(target_path), exist_ok=True)
        with open(target_path, 'w', encoding='utf-8') as f:
            f.write(translated)
            
        print(f"✅ Saved to {target_path}")
        
    except Exception as e:
        print(f"❌ Failed to translate {rel_path}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Translate documentation chapters to Urdu")
    parser.add_argument('--chapters', type=str, help="Comma-separated chapter numbers (e.g. 1,2,3)")
    parser.add_argument('--file', type=str, help="Specific file to translate (relative to docs dir)")
    parser.add_argument('--all', action='store_true', help="Translate all files")
    parser.add_argument('--force', action='store_true', help="Overwrite existing translations")
    
    args = parser.parse_args()
    
    if not os.getenv("GEMINI_API_KEY"):
        print("Warning: GEMINI_API_KEY not set. Translation will fail if not cached.")

    files_to_translate = []
    
    if args.file:
        files_to_translate.append(os.path.join(DOCS_DIR, args.file))
    elif args.chapters:
        chapters = args.chapters.split(',')
        for ch in chapters:
            pattern = os.path.join(DOCS_DIR, f"chapter{ch}*.md")
            files_to_translate.extend(glob.glob(pattern))
            # Also look for subdirectories if any (e.g. chapter1/foo.md)
            # For now, assuming flat or standard naming
    elif args.all:
        files_to_translate.extend(glob.glob(os.path.join(DOCS_DIR, "**", "*.md"), recursive=True))
    else:
        parser.print_help()
        return

    print(f"Found {len(files_to_translate)} files to translate.")
    
    for file_path in files_to_translate:
        translate_file(file_path, force=args.force)

if __name__ == "__main__":
    main()
