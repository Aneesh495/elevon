#!/usr/bin/env python3
"""
Lint script to enforce the zero em dash typography invariant across all
files in the repository.

Ensures that no Unicode em dash (U+2014), en dash (U+2013), horizontal bar
(U+2015), or prose double-hyphens appear anywhere in comments, documentation,
source code, or manifests.
"""

import os
import sys
import re

IGNORED_DIRS = {
    ".git",
    "build",
    "bin",
    "__pycache__",
    ".pytest_cache"
}

IGNORED_EXTENSIONS = {
    ".o",
    ".a",
    ".so",
    ".dylib",
    ".fdr",
    ".journal",
    ".pcap",
    ".png",
    ".jpg",
    ".pdf"
}

FORBIDDEN_CHARS = [
    ("\u2014", "Unicode Em Dash (U+2014)"),
    ("\u2013", "Unicode En Dash (U+2013)"),
    ("\u2015", "Horizontal Bar (U+2015)"),
]

# Regex to catch prose double-hyphens like "word -- word" or "end of thought--new thought"
# Avoids matching CLI flags like "--verbose", decrement like "i--", or markdown headers like "---"
PROSE_DOUBLE_HYPHEN = re.compile(r'(?<=[a-zA-Z0-9])\s*--\s*(?=[a-zA-Z0-9])')


def check_file(filepath):
    violations = []
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except Exception as e:
        return [(0, f"Failed to read file: {e}")]

    for line_idx, line in enumerate(lines, start=1):
        for char, name in FORBIDDEN_CHARS:
            if char in line:
                violations.append((line_idx, f"Contains forbidden {name}: '{line.strip()}'"))
        if PROSE_DOUBLE_HYPHEN.search(line):
            violations.append((line_idx, f"Contains prose double-hyphen used as em dash: '{line.strip()}'"))

    return violations


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    total_violations = 0
    checked_files = 0

    for root, dirs, files in os.walk(repo_root):
        dirs[:] = [d for d in dirs if d not in IGNORED_DIRS]

        for file in files:
            ext = os.path.splitext(file)[1].lower()
            if ext in IGNORED_EXTENSIONS:
                continue

            filepath = os.path.join(root, file)
            relpath = os.path.relpath(filepath, repo_root)

            # Skip the linter itself from self-matching the regex patterns
            if relpath == "tools/lint_no_em_dash.py":
                continue

            violations = check_file(filepath)
            checked_files += 1
            if violations:
                for line_idx, msg in violations:
                    print(f"ERROR: {relpath}:{line_idx}: {msg}", file=sys.stderr)
                    total_violations += 1

    if total_violations > 0:
        print(f"\nFailed: Found {total_violations} typography violations across {checked_files} files.", file=sys.stderr)
        sys.exit(1)
    else:
        print(f"Passed: Checked {checked_files} files with 0 typography violations.")
        sys.exit(0)


if __name__ == "__main__":
    main()
