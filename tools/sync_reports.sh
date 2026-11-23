#!/bin/sh
# Copy the report workspace of this clone, heavy files included, into a dash-reports clone.
#
#   tools/sync_reports.sh [--push] [path/to/dash-reports]     (default: ../dash-reports)
#
# This repo keeps only the files under 1 MiB (the pre-commit hook enforces it). Videos, progress
# logs, archives and traces stay in your working tree, untracked; this script copies
# report/, docs/experiments/ and RLframework/results/ into dash-reports at the same paths,
# commits there, and pushes only with --push. Nothing is ever deleted in dash-reports.
# report/presentations/ is never copied (presentations are not published).
set -e
PUSH=0
if [ "$1" = "--push" ]; then PUSH=1; shift; fi
SRC=$(git rev-parse --show-toplevel)
DST=${1:-"$SRC/../dash-reports"}
if [ ! -d "$DST/.git" ]; then
  echo "No dash-reports clone at $DST. Clone it first:" >&2
  echo "  git clone https://github.com/Dash-organisation/dash-reports.git \"$DST\"" >&2
  exit 1
fi
DST=$(cd "$DST" && pwd)
git -C "$DST" pull -q --ff-only
for d in report docs/experiments RLframework/results; do
  [ -d "$SRC/$d" ] || continue
  mkdir -p "$DST/$d"
  rsync -a --exclude '.DS_Store' --exclude '__MACOSX/' --exclude '__pycache__/' \
        --exclude '/presentations/' "$SRC/$d/" "$DST/$d/"
done
SHA=$(git -C "$SRC" rev-parse --short HEAD)
BRANCH=$(git -C "$SRC" rev-parse --abbrev-ref HEAD)
git -C "$DST" add -A
if git -C "$DST" diff --cached --quiet; then
  echo "dash-reports is already up to date."
  exit 0
fi
git -C "$DST" diff --cached --stat | tail -1
git -C "$DST" commit -q -m "report: sync from dash-sim2real@$SHA ($BRANCH)"
echo "Committed in $DST: $(git -C "$DST" log --oneline -1)"
if [ "$PUSH" -eq 1 ]; then
  git -C "$DST" push -q origin HEAD && echo "Pushed to dash-reports."
else
  echo "Not pushed. To publish: git -C \"$DST\" push   (or rerun with --push)"
fi
