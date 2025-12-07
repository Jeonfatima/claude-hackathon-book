import os
import re
import glob

def fix_frontmatter(file_path):
    """
    Fix the frontmatter in a Markdown file by:
    1. Replacing slashes in 'id' with dashes
    2. Ensuring 'title' values are properly quoted
    3. Keeping 'sidebar_position' unchanged
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Find the frontmatter (content between --- delimiters)
    frontmatter_match = re.match(r'^---\n(.*?)\n---', content, re.DOTALL)
    if not frontmatter_match:
        return  # No frontmatter found, skip this file

    frontmatter = frontmatter_match.group(1)
    original_frontmatter = frontmatter

    # Replace slashes in 'id' with dashes
    def replace_id_slashes(match):
        id_line = match.group(0)
        # Extract the value after 'id: '
        id_value = id_line.split(':', 1)[1].strip()
        # Replace slashes with dashes in the id value
        new_id_value = id_value.replace('/', '-')
        return f"id: {new_id_value}"

    frontmatter = re.sub(r'^id: .+$', replace_id_slashes, frontmatter, flags=re.MULTILINE)

    # Ensure 'title' values are properly quoted, especially if they contain special characters like ':'
    def quote_title(match):
        title_line = match.group(0)
        title_value = title_line.split(':', 1)[1].strip()

        # If the title is not already quoted and contains special characters, quote it
        if not (title_value.startswith('"') and title_value.endswith('"')) and (':' in title_value or "'" in title_value):
            # Escape any existing quotes in the title
            escaped_title = title_value.replace('"', '\\"')
            return f'title: "{escaped_title}"'
        return title_line

    frontmatter = re.sub(r'^title: .+$', quote_title, frontmatter, flags=re.MULTILINE)

    # Replace the original frontmatter with the fixed one if there were changes
    if frontmatter != original_frontmatter:
        fixed_content = content.replace(f'---\n{original_frontmatter}\n---', f'---\n{frontmatter}\n---', 1)

        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(fixed_content)

        print(f"Fixed frontmatter in {file_path}")

def process_docs_directory(docs_path):
    """
    Process all Markdown files in the docs directory and its subdirectories
    """
    # Find all .md files in docs/ and subfolders
    md_files = glob.glob(os.path.join(docs_path, "**/*.md"), recursive=True)

    for md_file in md_files:
        print(f"Processing {md_file}")
        fix_frontmatter(md_file)

if __name__ == "__main__":
    # Define the docs directory path
    docs_directory = "C:/Users/LAPTOP WORLD/Documents/code/hackathon-qwen/textbook/docs"

    # Process all Markdown files in the docs directory
    process_docs_directory(docs_directory)

    print("Frontmatter fixing complete!")