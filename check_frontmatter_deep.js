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
                const lines = content.split(/\r?\n/);

                // Check if starts with ---
                if (!lines[0].trim().startsWith('---')) {
                    console.log(`[MISSING START]: ${filePath}`);
                    continue;
                }

                // Check for closing ---
                let closingIndex = -1;
                for (let i = 1; i < lines.length; i++) {
                    if (lines[i].trim() === '---') {
                        closingIndex = i;
                        break;
                    }
                }

                if (closingIndex === -1) {
                    console.log(`[MISSING END]: ${filePath}`);
                    continue;
                }

                // Check for empty frontmatter
                if (closingIndex === 1) {
                    console.log(`[EMPTY FRONTMATTER]: ${filePath}`);
                }

                // Basic YAML check (key: value)
                for (let i = 1; i < closingIndex; i++) {
                    const line = lines[i].trim();
                    if (line && !line.includes(':')) {
                        console.log(`[INVALID YAML SYNTAX line ${i + 1}]: ${filePath}`);
                        console.log(`  Line: "${line}"`);
                    }
                }

            } catch (err) {
                console.error(`Error reading ${filePath}: ${err}`);
            }
        }
    }
}

console.log("Deep scanning frontmatter...");
walkDir(rootDir);
console.log("Scan complete.");
