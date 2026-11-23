#!/bin/sh
# One-time setup per clone: use the versioned hooks in tools/hooks/.
#   pre-commit     refuses files > 1 MiB (heavy outputs go to dash-reports)
#   post-commit, post-merge, post-checkout, post-rewrite
#                  refresh the graphify graph in the background, if graphify is installed
set -e
cd "$(git rev-parse --show-toplevel)"
chmod +x tools/hooks/*
git config core.hooksPath tools/hooks
echo "Hooks installed (core.hooksPath = tools/hooks)."
command -v graphify >/dev/null 2>&1 \
  && { echo "Building the graphify graph (AST only)..."; graphify update . >/dev/null && echo "Graph: graphify-out/graph.json"; } \
  || echo "graphify not found: install it with  uv tool install graphifyy  (or pipx install graphifyy), then rerun."
