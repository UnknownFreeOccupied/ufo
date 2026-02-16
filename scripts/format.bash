#!/usr/bin/env bash

# 1. Configuration
TARGET_DIRS=("lib" "tests")
EXTENSIONS="\.hpp$|\.cpp$|\.h$|\.c$"
REMOTE="origin"
BASE_BRANCH="main"

# 2. Safety Checks
if ! command -v clang-format &> /dev/null; then
    echo "Error: clang-format is not installed."
    exit 1
fi

if ! git rev-parse --is-inside-work-tree &> /dev/null; then
    echo "Error: Not a git repository."
    exit 1
fi

# 3. Fetch the latest from the remote branch
# This ensures $REMOTE/$BASE_BRANCH is actually the "latest"
echo "Fetching latest changes from $REMOTE/$BASE_BRANCH..."
git fetch "$REMOTE" "$BASE_BRANCH" --quiet

# 4. Gather changed files compared to the REMOTE branch
# Using origin/main (or equivalent) instead of just 'main'
mapfile -t FILE_ARRAY < <(git diff --name-only --diff-filter=d "$REMOTE/$BASE_BRANCH" | \
    grep -E "^($(IFS="|"; echo "${TARGET_DIRS[*]}"))/" | \
    grep -E "$EXTENSIONS")

TOTAL_FILES=${#FILE_ARRAY[@]}

if [ "$TOTAL_FILES" -eq 0 ]; then
    echo "No changed files found compared to $REMOTE/$BASE_BRANCH."
    exit 0
fi

echo "Found $TOTAL_FILES files that differ from $REMOTE/$BASE_BRANCH."

# 5. Iterate and Format
COUNTER=0
for FILE in "${FILE_ARRAY[@]}"; do
    ((COUNTER++))
    PERCENT=$(( COUNTER * 100 / TOTAL_FILES ))

    # Clear line and print progress
    printf "\r\033[K[%d/%d] (%d%%) Formatting: %s" "$COUNTER" "$TOTAL_FILES" "$PERCENT" "$FILE"

    # Only format if the file exists locally (edge case safety)
    if [ -f "$FILE" ]; then
        clang-format -i -fallback-style=Google "$FILE"
    fi
done

echo -e "\n----------------------------------------------"
echo "Formatting complete!"