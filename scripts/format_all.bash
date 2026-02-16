#!/usr/bin/env bash

# 1. Configuration
TARGET_DIRS=("lib" "tests")
EXTENSIONS='.*\.\(hpp\|cpp\|h\|c\)$'

# 2. Safety Check
if ! command -v clang-format &> /dev/null; then
    echo "Error: clang-format is not installed."
    exit 1
fi

# 3. Gather files
mapfile -t FILE_ARRAY < <(find "${TARGET_DIRS[@]}" -type f -regex "$EXTENSIONS" 2>/dev/null)
TOTAL_FILES=${#FILE_ARRAY[@]}

if [ "$TOTAL_FILES" -eq 0 ]; then
    echo "No source files found to format."
    exit 0
fi

# 4. Iterate and Format
COUNTER=0
for FILE in "${FILE_ARRAY[@]}"; do
    ((COUNTER++))
    PERCENT=$(( COUNTER * 100 / TOTAL_FILES ))

    # --- THE MAGIC FOR CLEARING THE LINE ---
    # \r  = Move cursor to the start of the line
    # \033[K = ANSI Escape code to clear everything from cursor to end of line
    printf "\r\033[K[%d/%d] (%d%%) Formatting: %s" "$COUNTER" "$TOTAL_FILES" "$PERCENT" "$FILE"

    clang-format -i -fallback-style=Google "$FILE"
done

# Final cleanup: move to a new line so the prompt doesn't overwrite the last status
echo -e "\n----------------------------------------------"
echo "Formatting complete!"