#!/bin/bash
set -e

DEST_DIR=".py_external_symlink"
EXTERNAL_ROOT=$(bazel info output_base)/external
mkdir -p "$DEST_DIR/bin"
echo "🔎 Scanning Bazel external for Python components..."

# Iterate through all rules_python managed repositories using the Bzlmod pattern
for REPO_PATH in "$EXTERNAL_ROOT"/rules_python[+][+]*; do
    [ -d "$REPO_PATH" ] || continue
    REPO_NAME=$(basename "$REPO_PATH")
    # We look for the python toolchain repositories (usually ending in _x86_64-unknown-linux-gnu)
    if [[ "$REPO_NAME" == *"_x86_64-unknown-linux-gnu"* ]]; then
        PYTHON_BIN="$REPO_PATH/bin/python3"
        if [ -f "$PYTHON_BIN" ]; then
            echo "📝 Found Hermetic Interpreter: $REPO_NAME"
            echo "   └─ Mapping binary to $DEST_DIR/bin/python3"
            
            ln -sf "$PYTHON_BIN" "$DEST_DIR/bin/python3"
        fi
    fi
    # We look for the pip repositories (have *pip* after the prefix)=
    if [[ "$REPO_NAME" == *"pip"* ]]; then
        # Look for site-packages directories inside the pip hub or wheel repos
        SITE_PKG=$(find "$REPO_PATH" -maxdepth 4 -type d -name "site-packages" 2>/dev/null | head -n 1)
        if [ -n "$SITE_PKG" ]; then
            echo "📦 Found Pip Repository: $REPO_NAME"
            echo "   └─ Merging site-packages into $DEST_DIR"
            
            # cp -as creates the symlink forest (merges directories)
            cp -as "$SITE_PKG"/* "$DEST_DIR/" 2>/dev/null || true
        fi
    fi
done

echo -e "\n✅ IDE sync complete."
echo "   Interpreter path: ${PWD}/${DEST_DIR}/bin/python3"
echo "   Package path:     ${PWD}/${DEST_DIR}"