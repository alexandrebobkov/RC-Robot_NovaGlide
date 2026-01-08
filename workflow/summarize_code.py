#!/usr/bin/env python3
import os
import re
from pathlib import Path

def create_mega_markdown(output_file="CODE_STRUCTURE.md", base_dirs=["main", "subsystem"]):
    """
    Scan through specified directories and create a comprehensive markdown file
    with all source code files organized by their directory structure.
    """
    with open(output_file, 'w', encoding='utf-8') as md_file:
        # Write header
        md_file.write("# NovaGlide Code Structure\n\n")
        md_file.write("This document contains the complete source code for the NovaGlide project.\n\n")
        md_file.write("## Table of Contents\n")

        # First pass to generate TOC
        toc_entries = []
        file_paths = []

        for base_dir in base_dirs:
            for root, _, files in os.walk(base_dir):
                for file in sorted(files):
                    if file.endswith(('.c', '.h')):
                        # Get relative path
                        rel_path = os.path.relpath(root, start=".")
                        if rel_path == ".":
                            rel_path = ""

                        # Create section title
                        if rel_path:
                            section_title = f"{rel_path}/{file}"
                        else:
                            section_title = file

                        # Create anchor for markdown
                        anchor = re.sub(r'[^a-zA-Z0-9]', '-', section_title.lower())
                        anchor = re.sub(r'-+', '-', anchor).strip('-')

                        toc_entries.append(f"  - [{section_title}](#{anchor})")
                        file_paths.append((os.path.join(root, file), rel_path, file, anchor))

        # Write TOC
        md_file.write("\n".join(toc_entries) + "\n\n")

        # Write each file content
        for file_path, rel_path, filename, anchor in file_paths:
            # Write section header
            if rel_path:
                section_title = f"{rel_path}/{filename}"
            else:
                section_title = filename

            md_file.write(f"\n## {section_title} <a name=\"{anchor}\"></a>\n\n")

            # Add code block
            md_file.write(f"```c\n")

            # Write file content
            try:
                with open(file_path, 'r', encoding='utf-8') as src_file:
                    content = src_file.read()
                    # Escape special markdown characters in code blocks
                    content = content.replace('```', '\`\`\`')
                    md_file.write(content)
            except Exception as e:
                md_file.write(f"// Error reading file: {e}\n")

            md_file.write("\n```\n")

if __name__ == "__main__":
    # Create the mega markdown file
    create_mega_markdown()

    # Print confirmation
    print("Successfully generated CODE_STRUCTURE.md with all source files")
