#!/usr/bin/env bash
set -euo pipefail

# Simple: read impacted targets from file, build non-test targets, run test targets.
# bazel-diff (or other tool) should have already written $HOME/impacted_targets.txt

TGT_FILE="$HOME/impacted_targets.txt"

if [ ! -f "$TGT_FILE" ]; then
  echo "Missing target file: $TGT_FILE"
  exit 2
fi

# read, trim, dedupe
mapfile -t targets < <(sed 's/#.*//; s/^[[:space:]]*//; s/[[:space:]]*$//' "$TGT_FILE" | grep -v '^$' | sort -u)

if [ ${#targets[@]} -eq 0 ]; then
  echo "No targets found in $TGT_FILE"
  exit 0
fi

echo "Impacted targets:"
printf ' - %s\n' "${targets[@]}"

# Separate into test and non-test targets
declare -a test_targets=()
declare -a non_test_targets=()

for t in "${targets[@]}"; do
  # Check if this is a test target (query kind)
  set +e
  kind=$(bazel query "kind(test, ${t})" 2>/dev/null || true)
  set -e
  
  if [ -n "$kind" ]; then
    test_targets+=("$t")
  else
    non_test_targets+=("$t")
  fi
done

# Build non-test targets
if [ ${#non_test_targets[@]} -gt 0 ]; then
  echo "=== Building non-test targets ==="
  printf ' - %s\n' "${non_test_targets[@]}"
  printf '%s\n' "${non_test_targets[@]}" | xargs -r bazel build --config=ci --keep_going || true
fi

# Run test targets
if [ ${#test_targets[@]} -gt 0 ]; then
  echo "=== Running test targets ==="
  printf ' - %s\n' "${test_targets[@]}"
  printf '%s\n' "${test_targets[@]}" | xargs -r bazel test --config=ci --keep_going --test_output=errors
else
  echo "No test targets to run."
fi

echo "=== Build and test complete ==="
