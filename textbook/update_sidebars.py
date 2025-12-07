import re

def update_sidebars_ts(file_path):
    """
    Update the sidebars.ts file to replace slash-style IDs with dash-style IDs
    """
    # Read the current content of the sidebars.ts file
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Store the original content for comparison
    original_content = content

    # Replace all occurrences of slash-style IDs with dash-style IDs
    # This handles both items arrays and individual id references
    content = re.sub(r"'module-(\d+)/intro'", r"'module-\1-intro'", content)
    content = re.sub(r"'module-(\d+)/chapter-(\d+)'", r"'module-\1-chapter-\2'", content)

    # Handle the id field in link objects if any exist
    content = re.sub(r"id: 'module-(\d+)/intro'", r"id: 'module-\1-intro'", content)
    content = re.sub(r"id: 'module-(\d+)/chapter-(\d+)'", r"id: 'module-\1-chapter-\2'", content)

    # Write the updated content back to the file if changes were made
    if content != original_content:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(content)
        print(f"Updated {file_path} with dash-style IDs")
    else:
        print(f"No changes needed in {file_path}")

if __name__ == "__main__":
    # Define the path to the sidebars.ts file
    sidebars_path = "C:/Users/LAPTOP WORLD/Documents/code/hackathon-qwen/textbook/sidebars.ts"

    # Update the sidebars.ts file
    update_sidebars_ts(sidebars_path)

    print("Sidebar update complete!")