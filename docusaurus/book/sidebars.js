// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  bookSidebar: [
    'intro',
    'chapter1',
    'chapter2',
    'chapter3',
    {
      type: 'category',
      label: 'Chapter 5: Advanced Humanoid AI',
      items: [
        'chapter-5-advanced-humanoid-ai/intro',
        'chapter-5-advanced-humanoid-ai/multi-sensor-fusion',
        'chapter-5-advanced-humanoid-ai/cognitive-planning',
        'chapter-5-advanced-humanoid-ai/ai-integration',
        'chapter-5-advanced-humanoid-ai/troubleshooting',
      ],
    },
  ],
};

export default sidebars;
