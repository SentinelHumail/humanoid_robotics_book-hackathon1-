const fs = require('fs');
const path = require('path');

const rootDir = 'c:/Users/lordh/coding/hackathon1_book/docs';

function walkDir(dir) {
    const files = fs.readdirSync(dir);
    for (const file of files) {
        const filePath = path.join(dir, file);
        const stat = fs.statSync(filePath);
        if (stat.isDirectory()) {
            walkDir(filePath);
        } else if (file.endsWith('.md')) {
            try {
                const content = fs.readFileSync(filePath, 'utf8');
                const firstLine = content.trim().split('\n')[0];
                if (!firstLine.trim().startsWith('---')) {
                    console.log(`MISSING FRONTMATTER: ${filePath}`);
                }
            } catch (err) {
                console.error(`Error reading ${filePath}: ${err}`);
            }
        }
    }
}

console.log("Scanning for files missing frontmatter...");
walkDir(rootDir);
console.log("Scan complete.");
