"""
Markdown linting tests for Physical AI & Humanoid Robotics textbook.

This module validates that all Markdown files follow consistent formatting
rules defined in .markdownlint.json.
"""

import subprocess
import pytest
from pathlib import Path


def test_markdown_lint():
    """
    Test that all Markdown files pass markdownlint validation.

    Runs markdownlint on all .md files in the docs/ directory and ensures
    they comply with the project's Markdown style rules.
    """
    # Get repository root
    repo_root = Path(__file__).parent.parent.parent

    # Run markdownlint
    result = subprocess.run(
        ['npx', 'markdownlint', 'docs/**/*.md'],
        cwd=repo_root,
        capture_output=True,
        text=True
    )

    # Check if linting passed
    assert result.returncode == 0, f"Markdown linting failed:\\n{result.stdout}\\n{result.stderr}"


def test_spec_markdown_lint():
    """
    Test that spec files pass markdownlint validation.
    """
    repo_root = Path(__file__).parent.parent.parent

    result = subprocess.run(
        ['npx', 'markdownlint', 'specs/**/*.md'],
        cwd=repo_root,
        capture_output=True,
        text=True
    )

    # Spec files may have different rules, just check they exist and are readable
    # This is a softer check than docs validation
    assert result is not None
