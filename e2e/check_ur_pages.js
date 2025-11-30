const fs = require('fs');
const path = require('path');

const urPath = path.join(__dirname, '..', 'docusaurus', 'i18n', 'ur', 'docusaurus-plugin-content-docs', 'current');

if (!fs.existsSync(urPath)) {
  console.error('Urdu i18n directory not found:', urPath);
  process.exit(2);
}

const files = fs.readdirSync(urPath);
console.log('Found files in Urdu i18n dir:', files);
if (files.length === 0) {
  console.error('No files found under Urdu i18n dir');
  process.exit(3);
}

console.log('Urdu i18n check passed.');
