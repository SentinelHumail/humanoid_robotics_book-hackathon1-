import os

root_dir = 'c:/Users/lordh/coding/hackathon1_book/docs'

print("Scanning for files missing frontmatter...")
for dirpath, dirnames, filenames in os.walk(root_dir):
    for filename in filenames:
        if filename.endswith('.md'):
            filepath = os.path.join(dirpath, filename)
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    first_line = f.readline().strip()
                    if first_line != '---':
                        print(f"MISSING FRONTMATTER: {filepath}")
            except Exception as e:
                print(f"Error reading {filepath}: {e}")
print("Scan complete.")
