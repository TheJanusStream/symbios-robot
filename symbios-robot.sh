#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Use the first argument as the project directory, or default to the current directory
PROJECT_DIR="${1:-.}"

# Define the output file with a timestamp
OUTPUT_FILE="symbios_robot_primer_$(date +%Y%m%d_%H%M%S).txt"

# --- Header ---
echo "--- SYMBIOS ROBOT PRIMER ---" > "$OUTPUT_FILE"
echo "Generated on: $(date)" >> "$OUTPUT_FILE"
echo "Project Directory: $(realpath "$PROJECT_DIR")" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# --- Git Status ---
echo "--- START OF GIT STATUS ---" >> "$OUTPUT_FILE"
(cd "$PROJECT_DIR" && git status) >> "$OUTPUT_FILE"
echo "--- END OF GIT STATUS ---" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# --- Git Log (Last 5 Commits) ---
echo "--- START OF GIT LOG ---" >> "$OUTPUT_FILE"
(cd "$PROJECT_DIR" && git log -n 5 --oneline --graph) >> "$OUTPUT_FILE"
echo "--- END OF GIT LOG ---" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# --- Directory Tree ---
echo "--- START OF DIRECTORY TREE ---" >> "$OUTPUT_FILE"
(cd "$PROJECT_DIR" && tree -L 3 -I 'target') >> "$OUTPUT_FILE"
echo "--- END OF DIRECTORY TREE ---" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# --- Key File Contents ---
KEY_FILES=(
    "$PROJECT_DIR/Cargo.toml"
    "$PROJECT_DIR/rust-toolchain.toml"
    "$PROJECT_DIR/README.md"
    "$PROJECT_DIR/.gitignore"
    "$PROJECT_DIR/.github/workflows/rust.yml"
)

# Add source files
if [ -d "$PROJECT_DIR/src" ]; then
    while IFS= read -r file; do
        KEY_FILES+=("$file")
    done < <(cd "$PROJECT_DIR" && find src -type f -name "*.rs" | sort)
fi

# Add test files
if [ -d "$PROJECT_DIR/tests" ]; then
    while IFS= read -r file; do
        KEY_FILES+=("$file")
    done < <(cd "$PROJECT_DIR" && find tests -type f -name "*.rs" | sort)
fi

# Add example files
if [ -d "$PROJECT_DIR/examples" ]; then
    while IFS= read -r file; do
        KEY_FILES+=("$file")
    done < <(cd "$PROJECT_DIR" && find examples -type f -name "*.rs" | sort)
fi

# Add bench files
if [ -d "$PROJECT_DIR/benches" ]; then
    while IFS= read -r file; do
        KEY_FILES+=("$file")
    done < <(cd "$PROJECT_DIR" && find benches -type f -name "*.rs" | sort)
fi

if [ -f "build.rs" ]; then
    KEY_FILES+=("build.rs")
fi

for file in "${KEY_FILES[@]}"; do
    FILE_PATH="$PROJECT_DIR/$file"
    if [ -f "$FILE_PATH" ]; then
        echo "--- START OF FILE: $file ---" >> "$OUTPUT_FILE"
        cat "$FILE_PATH" >> "$OUTPUT_FILE"
        echo "--- END OF FILE: $file ---" >> "$OUTPUT_FILE"
        echo "" >> "$OUTPUT_FILE"
    else
        echo "--- WARNING: FILE NOT FOUND: $file ---" >> "$OUTPUT_FILE"
        echo "" >> "$OUTPUT_FILE"
    fi
done

if command -v chainlink &> /dev/null; then
    echo "--- START OF CHAINLINK TREE OF OPEN ISSUES ---" >> "$OUTPUT_FILE"
    # Capture pending tasks and recent history
    (cd "$PROJECT_DIR" && chainlink tree -s open | head -n 20) >> "$OUTPUT_FILE" 
    echo "--- END OF CHAINLINK TREE ---" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
fi

echo "Symbios robot primer created successfully at: $OUTPUT_FILE"
