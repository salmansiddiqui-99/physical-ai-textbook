"""
Front matter validation script for Physical AI & Humanoid Robotics textbook chapters.

This script validates that all chapter Markdown files have correct YAML front matter
according to the schema defined in specs/001-physical-ai-textbook/contracts/frontmatter-schema.json.
"""

import json
import re
import yaml
from pathlib import Path
from typing import Dict, List, Tuple


def extract_frontmatter(markdown_file: Path) -> Tuple[Dict, str]:
    """
    Extract YAML front matter from a Markdown file.

    Args:
        markdown_file: Path to the Markdown file

    Returns:
        Tuple of (front_matter_dict, error_message)
        error_message is empty string if successful
    """
    try:
        content = markdown_file.read_text(encoding='utf-8')

        # Match YAML front matter between --- delimiters
        pattern = r'^---\s*\n(.*?)\n---\s*\n'
        match = re.match(pattern, content, re.DOTALL)

        if not match:
            return {}, f"No front matter found in {markdown_file}"

        yaml_content = match.group(1)
        front_matter = yaml.safe_load(yaml_content)

        return front_matter, ""

    except Exception as e:
        return {}, f"Error parsing {markdown_file}: {str(e)}"


def validate_against_schema(front_matter: Dict, schema: Dict, file_path: Path) -> List[str]:
    """
    Validate front matter against JSON schema.

    Args:
        front_matter: Parsed YAML front matter
        schema: JSON schema definition
        file_path: Path to the file (for error messages)

    Returns:
        List of validation error messages (empty if valid)
    """
    errors = []

    # Check required fields
    required_fields = schema.get('required', [])
    for field in required_fields:
        if field not in front_matter:
            errors.append(f"{file_path}: Missing required field '{field}'")

    # Validate field types and constraints
    properties = schema.get('properties', {})

    for field, value in front_matter.items():
        if field not in properties:
            continue  # Allow additional properties for now

        prop_schema = properties[field]

        # Type validation
        expected_type = prop_schema.get('type')
        if expected_type == 'string' and not isinstance(value, str):
            errors.append(f"{file_path}: Field '{field}' must be string, got {type(value).__name__}")
        elif expected_type == 'integer' and not isinstance(value, int):
            errors.append(f"{file_path}: Field '{field}' must be integer, got {type(value).__name__}")
        elif expected_type == 'array' and not isinstance(value, list):
            errors.append(f"{file_path}: Field '{field}' must be array, got {type(value).__name__}")

        # String length constraints
        if expected_type == 'string' and isinstance(value, str):
            min_length = prop_schema.get('minLength')
            max_length = prop_schema.get('maxLength')
            if min_length and len(value) < min_length:
                errors.append(f"{file_path}: Field '{field}' too short (min {min_length} chars)")
            if max_length and len(value) > max_length:
                errors.append(f"{file_path}: Field '{field}' too long (max {max_length} chars)")

        # Integer range constraints
        if expected_type == 'integer' and isinstance(value, int):
            minimum = prop_schema.get('minimum')
            maximum = prop_schema.get('maximum')
            if minimum and value < minimum:
                errors.append(f"{file_path}: Field '{field}' below minimum ({minimum})")
            if maximum and value > maximum:
                errors.append(f"{file_path}: Field '{field}' above maximum ({maximum})")

        # Array constraints
        if expected_type == 'array' and isinstance(value, list):
            min_items = prop_schema.get('minItems')
            max_items = prop_schema.get('maxItems')
            if min_items and len(value) < min_items:
                errors.append(f"{file_path}: Field '{field}' has too few items (min {min_items})")
            if max_items and len(value) > max_items:
                errors.append(f"{file_path}: Field '{field}' has too many items (max {max_items})")

        # Pattern validation
        pattern = prop_schema.get('pattern')
        if pattern and expected_type == 'string' and isinstance(value, str):
            if not re.match(pattern, value):
                errors.append(f"{file_path}: Field '{field}' does not match required pattern")

    return errors


def main():
    """
    Main validation function.

    Validates all chapter files in docs/moduleX/ directories.
    """
    # Get paths
    repo_root = Path(__file__).parent.parent
    schema_file = repo_root / 'specs' / '001-physical-ai-textbook' / 'contracts' / 'frontmatter-schema.json'
    docs_dir = repo_root / 'docs'

    # Load schema
    if not schema_file.exists():
        print(f"Schema file not found: {schema_file}")
        return 1

    with open(schema_file, 'r', encoding='utf-8') as f:
        schema = json.load(f)

    # Find all chapter files
    chapter_files = []
    for module_dir in docs_dir.glob('module*'):
        if module_dir.is_dir():
            chapter_files.extend(module_dir.glob('ch*.md'))

    if not chapter_files:
        print("No chapter files found to validate")
        return 0

    # Validate each file
    total_errors = []
    for chapter_file in sorted(chapter_files):
        front_matter, error = extract_frontmatter(chapter_file)

        if error:
            total_errors.append(error)
            continue

        file_errors = validate_against_schema(front_matter, schema, chapter_file)
        total_errors.extend(file_errors)

    # Report results
    if total_errors:
        print(f"\n[FAIL] Front matter validation FAILED with {len(total_errors)} errors:\n")
        for error in total_errors:
            print(f"  - {error}")
        return 1
    else:
        print(f"\n[PASS] Front matter validation PASSED for {len(chapter_files)} chapter(s)")
        return 0


if __name__ == '__main__':
    exit(main())
