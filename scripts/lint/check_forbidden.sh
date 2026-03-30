#!/bin/bash
# check_forbidden.sh
# Scans source code for forbidden headers, keywords, and dynamic-allocation APIs.
# Returns 1 (error) if found, 0 (ok) if clean.

EXCLUDE_DIRS=(.git build cmake third_party)
FORBIDDEN_HEADERS="iostream sstream locale regex vector list map set unordered_map unordered_set"
FORBIDDEN_KEYWORDS=" new" # Space prefix to avoid matching 'newline' etc.
FORBIDDEN_FREERTOS_DYNAMIC_APIS="xTaskCreate xTaskCreatePinnedToCore xQueueCreate xSemaphoreCreateBinary xSemaphoreCreateCounting xSemaphoreCreateMutex xSemaphoreCreateRecursiveMutex xTimerCreate xEventGroupCreate xStreamBufferCreate xMessageBufferCreate pvPortMalloc"

echo "Running static check for forbidden C++ features..."

EXIT_CODE=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [ "$#" -ge 1 ]; then
    EXCEPTIONS_FILE="$1"
    shift
else
    EXCEPTIONS_FILE="${SCRIPT_DIR}/forbidden_exceptions.txt"
fi

if [ "$#" -ge 1 ]; then
    TARGET_DIRS=("$@")
else
    TARGET_DIRS=("${REPO_ROOT}")
fi

# Ensure the exceptions file exists (create empty if neither arg nor local file exists)
if [ ! -f "$EXCEPTIONS_FILE" ]; then
    # If using the default name and it doesn't exist, just use /dev/null logic or create empty temp
    # We will just touch a temp file to avoid grep errors if the user didn't provide one
    EXCEPTIONS_FILE=$(mktemp)
fi

GREP_EXCLUDES=()
for excluded in "${EXCLUDE_DIRS[@]}"; do
    GREP_EXCLUDES+=(--exclude-dir="$excluded")
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then continue; fi
    
    # 1. Check Headers
    for header in $FORBIDDEN_HEADERS; do
        # grep recursively, finding usages
        # exclude the script itself and build dirs if any
        MATCHES=$(grep -rn "${GREP_EXCLUDES[@]}" --include=*.cpp --include=*.hpp "#include <$header>" "$dir" | grep -vFf "$EXCEPTIONS_FILE")
        if [ ! -z "$MATCHES" ]; then
            echo "ERROR: Forbidden header <$header> found:"
            echo "$MATCHES"
            EXIT_CODE=1
        fi
    done
    
    # 2. Check Allocation (new)
    # Be careful with this pattern, trying to verify 'new ' usage
    MATCHES_NEW=$(grep -rn "${GREP_EXCLUDES[@]}" --include=*.cpp --include=*.hpp " new " "$dir" | grep -vE "placement new|//|/\\*" | grep -vFf "$EXCEPTIONS_FILE")
    if [ ! -z "$MATCHES_NEW" ]; then
         echo "ERROR: Forbidden keyword 'new' found (use static allocation):"
         echo "$MATCHES_NEW"
         EXIT_CODE=1
    fi

    # 3. Check Types (double)
    # Exclude comments
    MATCHES_DOUBLE=$(grep -rn "${GREP_EXCLUDES[@]}" --include=*.cpp --include=*.hpp " double " "$dir" | grep -vE "//|/\\*" | grep -vFf "$EXCEPTIONS_FILE")
    if [ ! -z "$MATCHES_DOUBLE" ]; then
         echo "ERROR: Forbidden type 'double' found (use float):"
         echo "$MATCHES_DOUBLE"
         EXIT_CODE=1
    fi

    # 4. Check FreeRTOS dynamic allocation APIs
    for api in $FORBIDDEN_FREERTOS_DYNAMIC_APIS; do
        MATCHES_API=$(grep -rn "${GREP_EXCLUDES[@]}" --include=*.cpp --include=*.hpp "${api}(" "$dir" | grep -vFf "$EXCEPTIONS_FILE")
        if [ ! -z "$MATCHES_API" ]; then
            echo "ERROR: Forbidden FreeRTOS dynamic allocation API '${api}' found:"
            echo "$MATCHES_API"
            EXIT_CODE=1
        fi
    done
done

if [ $EXIT_CODE -eq 0 ]; then
    echo "Static check passed."
else
    echo "Static check FAILED."
fi

exit $EXIT_CODE
