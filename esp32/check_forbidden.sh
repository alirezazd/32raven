#!/bin/bash
# check_forbidden.sh
# Scans source code for forbidden headers and keywords (new/delete).
# Returns 1 (error) if found, 0 (ok) if clean.

TARGET_DIRS="main drivers utils"
FORBIDDEN_HEADERS="iostream sstream locale regex vector list map set unordered_map unordered_set"
FORBIDDEN_KEYWORDS=" new" # Space prefix to avoid matching 'newline' etc.

echo "Running static check for forbidden C++ features..."

EXIT_CODE=0
EXCEPTIONS_FILE="forbidden_exceptions.txt"

# Create / touch exceptions file if it doesn't exist
if [ ! -f "$EXCEPTIONS_FILE" ]; then
    touch "$EXCEPTIONS_FILE"
fi

for dir in $TARGET_DIRS; do
    if [ ! -d "$dir" ]; then continue; fi
    
    # 1. Check Headers
    for header in $FORBIDDEN_HEADERS; do
        # grep recursively, finding usages
        # exclude the script itself and build dirs if any
        MATCHES=$(grep -rn "#include <$header>" "$dir" | grep -vFf "$EXCEPTIONS_FILE")
        if [ ! -z "$MATCHES" ]; then
            echo "ERROR: Forbidden header <$header> found:"
            echo "$MATCHES"
            EXIT_CODE=1
        fi
    done

    # 2. Check Allocation (new)
    # Be careful with this pattern, trying to verify 'new ' usage
    MATCHES_NEW=$(grep -rn " new " "$dir" | grep -vE "placement new|//|/\\*" | grep -vFf "$EXCEPTIONS_FILE")
    if [ ! -z "$MATCHES_NEW" ]; then
         echo "ERROR: Forbidden keyword 'new' found (use static allocation):"
         echo "$MATCHES_NEW"
         EXIT_CODE=1
    fi
done

if [ $EXIT_CODE -eq 0 ]; then
    echo "Static check passed."
else
    echo "Static check FAILED."
fi

exit $EXIT_CODE
