#!/bin/sh

# Check that the code files are formatted correctly and pass clang-tidy. This is
# mostly intended for CI: locally, you can build everything with
# -DCMAKE_CXX_CLANG_TIDY=clang-tidy to check clang-tidy on build, but this won't
# check includes in header files. This script runs clang-tidy on both .cpp and
# .h files to ensure that headers get properly checked.
#
# Usage: ./run_lint.sh <build_dir>
#
# The build directory must have been generated with
# -DCMAKE_EXPORT_COMPILE_COMMANDS=ON.

# Fail if any of the commands fails.
set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <build_dir>" >&2
    exit 1
fi

FILES_TO_LINT_PATH=$(mktemp)
find ./src ./apps ./tests ./include -type f '(' -name '*.cpp' -o -name '*.h' ')' -print0 > "${FILES_TO_LINT_PATH}"

echo 'Checking formatting' >&2
# clang-format runs very quickly when just giving all the files at once. No need
# to parallelize.
xargs --null clang-format --dry-run --Werror < "${FILES_TO_LINT_PATH}"
echo 'Formatting OK' >&2

printf '\nRunning clang-tidy\n' >&2
parallel --null --quote clang-tidy -p="$1" --warnings-as-errors='*' --header-filter='.*/include/bananas_aruco/.*' < "${FILES_TO_LINT_PATH}"
echo 'clang-tidy OK' >&2

rm -- "${FILES_TO_LINT_PATH}"
