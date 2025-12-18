#!/usr/bin/env bash
# Usiamo -uo pipefail, ma evitiamo -e globale per gestire noi gli errori di bazel
set -uo pipefail

TGT_FILE="$HOME/impacted_targets.txt"
EXIT_STATUS=0 # Variabile sentinella

if [ ! -f "$TGT_FILE" ]; then
  echo "Missing target file: $TGT_FILE"
  exit 2
fi

mapfile -t targets < <(sed 's/#.*//; s/^[[:space:]]*//; s/[[:space:]]*$//' "$TGT_FILE" | grep -v '^$' | sort -u)

if [ ${#targets[@]} -eq 0 ]; then
  echo "No targets found in $TGT_FILE"
  exit 0
fi

echo "Impacted targets:"
printf ' - %s\n' "${targets[@]}"

declare -a test_targets=()
declare -a non_test_targets=()

#split targets into test and non-test
for t in "${targets[@]}"; do
  kind=$(bazel query "kind(test, ${t})" 2>/dev/null || true)
  
  if [ -n "$kind" ]; then
    test_targets+=("$t")
  else
    non_test_targets+=("$t")
  fi
done

# Bazel build
# if build fails, exit immediately
if [ ${#non_test_targets[@]} -gt 0 ]; then
  echo "=== Building non-test targets ==="
  printf ' - %s\n' "${non_test_targets[@]}"
  if ! printf '%s\n' "${non_test_targets[@]}" | xargs -r bazel build --config=ci --keep_going; then
    echo "❌ Build failed for some targets."
    exit 1
  fi
fi

#Bazel test
if [ ${#test_targets[@]} -gt 0 ]; then
  echo "=== Running test targets ==="
  printf ' - %s\n' "${test_targets[@]}"
  
  # Eseguiamo bazel test. Se fallisce, EXIT_STATUS diventa 1
  if ! printf '%s\n' "${test_targets[@]}" | xargs -r bazel test --config=ci --keep_going --test_output=errors; then
    echo "❌ Test failed for some targets."
    EXIT_STATUS=1
  fi
else
  echo "No test targets to run."
fi

echo "=== Build and test complete with exit code $EXIT_STATUS ==="

# Ritorna lo stato globale: se uno dei due è fallito, lo step di GitHub fallirà
exit $EXIT_STATUS