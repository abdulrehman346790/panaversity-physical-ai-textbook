#!/usr/bin/env node

/**
 * Automated Translation Script for Docusaurus i18n
 * Translates English Markdown files to Urdu using OpenAI API
 */

const fs = require('fs').promises;
const path = require('path');
const { GoogleTranslator } = require('deep-translator');

// Configuration
const DOCS_DIR = path.join(__dirname, '../docs');
const I18N_DIR = path.join(__dirname, '../i18n/ur/docusaurus-plugin-content-docs/current');
const BATCH_SIZE = 5; // Translate 5 paragraphs at a time

// Technical terms glossary (keep in English)
const TECHNICAL_TERMS = {
    'ROS': 'ROS',
    'ROS 2': 'ROS 2',
    'Node': 'نوڈ',
    'Topic': 'ٹاپک',
    'Publisher': 'پبلشر',
    'Subscriber': 'سبسکرائبر',
    'Webots': 'Webots',
    'Python': 'Python',
    'simulation': 'سمولیشن',
    'robot': 'روبوٹ',
    'controller': 'کنٹرولر',
};

/**
 * Translate text while preserving Markdown structure
 */
async function translateMarkdown(content) {
    const translator = new GoogleTranslator({ source: 'en', target: 'ur' });

    // Split content into blocks
    const blocks = content.split('\n\n');
    const translatedBlocks = [];

    for (const block of blocks) {
        // Skip code blocks
        if (block.startsWith('```') || block.includes('```')) {
            translatedBlocks.push(block);
            continue;
        }

        // Skip frontmatter
        if (block.startsWith('---')) {
            translatedBlocks.push(block);
            continue;
        }

        // Skip URLs and file paths
        if (block.match(/^(http|\/|\.\/)/)) {
            translatedBlocks.push(block);
            continue;
        }

        try {
            // Translate the block
            const translated = await translator.translate(block);
            translatedBlocks.push(translated);

            // Rate limiting
            await new Promise(resolve => setTimeout(resolve, 100));
        } catch (error) {
            console.error(`Translation error: ${error.message}`);
            translatedBlocks.push(block); // Keep original on error
        }
    }

    return translatedBlocks.join('\n\n');
}

/**
 * Process a single Markdown file
 */
async function processFile(filePath, relativePath) {
    console.log(`Translating: ${relativePath}`);

    const content = await fs.readFile(filePath, 'utf-8');
    const translated = await translateMarkdown(content);

    // Create output directory
    const outputPath = path.join(I18N_DIR, relativePath);
    const outputDir = path.dirname(outputPath);
    await fs.mkdir(outputDir, { recursive: true });

    // Write translated file
    await fs.writeFile(outputPath, translated, 'utf-8');
    console.log(`✓ Saved: ${relativePath}`);
}

/**
 * Recursively process all Markdown files
 */
async function processDirectory(dir, baseDir = dir) {
    const entries = await fs.readdir(dir, { withFileTypes: true });

    for (const entry of entries) {
        const fullPath = path.join(dir, entry.name);

        if (entry.isDirectory()) {
            await processDirectory(fullPath, baseDir);
        } else if (entry.name.endsWith('.md') || entry.name.endsWith('.mdx')) {
            const relativePath = path.relative(baseDir, fullPath);
            await processFile(fullPath, relativePath);
        }
    }
}

/**
 * Main execution
 */
async function main() {
    console.log('🚀 Starting Urdu translation...\n');

    try {
        // Create i18n directory structure
        await fs.mkdir(I18N_DIR, { recursive: true });

        // Process all docs
        await processDirectory(DOCS_DIR);

        console.log('\n✅ Translation complete!');
        console.log(`📁 Urdu files saved to: ${I18N_DIR}`);
    } catch (error) {
        console.error('❌ Translation failed:', error);
        process.exit(1);
    }
}

// Run if called directly
if (require.main === module) {
    main();
}

module.exports = { translateMarkdown, processFile };
