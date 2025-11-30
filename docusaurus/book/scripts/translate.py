#!/usr/bin/env python3
"""
Automated Translation Script for Docusaurus i18n
Translates English Markdown files to Urdu using deep-translator
"""

import os
import re
from pathlib import Path
from deep_translator import GoogleTranslator

# Configuration
DOCS_DIR = Path("../docs")
I18N_DIR = Path("../i18n/ur/docusaurus-plugin-content-docs/current")

# Technical terms to keep in English
TECHNICAL_TERMS = {
    'ROS 2': 'ROS 2',
    'ROS': 'ROS',
    'Webots': 'Webots',
    'Python': 'Python',
    'Gazebo': 'Gazebo',
    'Node': 'Node',
    'Topic': 'Topic',
}

def preserve_code_blocks(content):
    """Extract code blocks and replace with placeholders"""
    code_blocks = []
    pattern = r'```[\s\S]*?```'
    
    def replacer(match):
        code_blocks.append(match.group(0))
        return f'__CODE_BLOCK_{len(code_blocks) - 1}__'
    
    content = re.sub(pattern, replacer, content)
    return content, code_blocks

def restore_code_blocks(content, code_blocks):
    """Restore code blocks from placeholders"""
    for i, block in enumerate(code_blocks):
        content = content.replace(f'__CODE_BLOCK_{i}__', block)
    return content

def translate_markdown(content):
    """Translate Markdown content while preserving structure"""
    translator = GoogleTranslator(source='en', target='ur')
    
    # Preserve code blocks
    content, code_blocks = preserve_code_blocks(content)
    
    # Split into paragraphs
    paragraphs = content.split('\n\n')
    translated_paragraphs = []
    
    for para in paragraphs:
        # Skip empty paragraphs
        if not para.strip():
            translated_paragraphs.append(para)
            continue
        
        # Skip frontmatter
        if para.startswith('---'):
            translated_paragraphs.append(para)
            continue
        
        # Skip URLs and file paths
        if re.match(r'^(http|\/|\.\/)', para):
            translated_paragraphs.append(para)
            continue
        
        # Skip placeholders
        if '__CODE_BLOCK_' in para:
            translated_paragraphs.append(para)
            continue
        
        try:
            # Translate
            translated = translator.translate(para)
            translated_paragraphs.append(translated)
        except Exception as e:
            print(f"Translation error: {e}")
            translated_paragraphs.append(para)  # Keep original on error
    
    # Join paragraphs
    result = '\n\n'.join(translated_paragraphs)
    
    # Restore code blocks
    result = restore_code_blocks(result, code_blocks)
    
    return result

def process_file(file_path, relative_path):
    """Process a single Markdown file"""
    print(f"Translating: {relative_path}")
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    translated = translate_markdown(content)
    
    # Create output directory
    output_path = I18N_DIR / relative_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Write translated file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(translated)
    
    print(f"✓ Saved: {relative_path}")

def process_directory(directory):
    """Recursively process all Markdown files"""
    for file_path in directory.rglob('*.md'):
        relative_path = file_path.relative_to(DOCS_DIR)
        process_file(file_path, relative_path)
    
    for file_path in directory.rglob('*.mdx'):
        relative_path = file_path.relative_to(DOCS_DIR)
        process_file(file_path, relative_path)

def main():
    """Main execution"""
    print("🚀 Starting Urdu translation...\n")
    
    # Create i18n directory
    I18N_DIR.mkdir(parents=True, exist_ok=True)
    
    # Process all docs
    process_directory(DOCS_DIR)
    
    print("\n✅ Translation complete!")
    print(f"📁 Urdu files saved to: {I18N_DIR}")

if __name__ == "__main__":
    # Change to script directory
    os.chdir(Path(__file__).parent)
    main()
