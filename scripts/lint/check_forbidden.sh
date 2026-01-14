#!/bin/bash
# check_forbidden.sh
# Scans source code for forbidden headers and keywords (new/delete).
# Returns 1 (error) if found, 0 (ok) if clean.

TARGET_DIRS="."
EXCLUDE_DIRS={.git,build,cmake}
FORBIDDEN_HEADERS="iostream sstream locale regex vector list map set unordered_map unordered_set"
FORBIDDEN_KEYWORDS=" new" # Space prefix to avoid matching 'newline' etc.

echo "Running static check for forbidden C++ features..."

EXIT_CODE=0

if [ "$#" -ge 1 ]; then
    EXCEPTIONS_FILE="$1"
else
    EXCEPTIONS_FILE="forbidden_exceptions.txt"
fi

# Ensure the exceptions file exists (create empty if neither arg nor local file exists)
if [ ! -f "$EXCEPTIONS_FILE" ]; then
    # If using the default name and it doesn't exist, just use /dev/null logic or create empty temp
    # We will just touch a temp file to avoid grep errors if the user didn't provide one
    EXCEPTIONS_FILE=$(mktemp)
fi

for dir in $TARGET_DIRS; do
    if [ ! -d "$dir" ]; then continue; fi
    
    # 1. Check Headers
    for header in $FORBIDDEN_HEADERS; do
        # grep recursively, finding usages
        # exclude the script itself and build dirs if any
        MATCHES=$(grep -rn --exclude-dir=$EXCLUDE_DIRS --include=*.cpp --include=*.hpp "#include <$header>" "$dir" | grep -vFf "$EXCEPTIONS_FILE")
        if [ ! -z "$MATCHES" ]; then
            echo "ERROR: Forbidden header <$header> found:"
            echo "$MATCHES"
            EXIT_CODE=1
        fi
    done
    
    # 2. Check Allocation (new)
    # Be careful with this pattern, trying to verify 'new ' usage
    MATCHES_NEW=$(grep -rn --exclude-dir=$EXCLUDE_DIRS --include=*.cpp --include=*.hpp " new " "$dir" | grep -vE "placement new|//|/\\*" | grep -vFf "$EXCEPTIONS_FILE")
    if [ ! -z "$MATCHES_NEW" ]; then
         echo "ERROR: Forbidden keyword 'new' found (use static allocation):"
         echo "$MATCHES_NEW"
         EXIT_CODE=1
    fi

    # 3. Check Types (double)
    # Exclude comments
    MATCHES_DOUBLE=$(grep -rn --exclude-dir=$EXCLUDE_DIRS --include=*.cpp --include=*.hpp " double " "$dir" | grep -vE "//|/\\*" | grep -vFf "$EXCEPTIONS_FILE")
    if [ ! -z "$MATCHES_DOUBLE" ]; then
         echo "ERROR: Forbidden type 'double' found (use float):"
         echo "$MATCHES_DOUBLE"
         EXIT_CODE=1
    fi
done

if [ $EXIT_CODE -eq 0 ]; then
    echo "Static check passed."
else
    echo "Static check FAILED."
fi

exit $EXIT_CODE
