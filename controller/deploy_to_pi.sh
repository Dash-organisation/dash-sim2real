#!/usr/bin/env bash
# deploy_to_pi.sh — copy the controller/ code of one git commit to the robot's Raspberry Pi.
#
# The Pi has no internet and its ~/running_robot is a clone of another repo, so code reaches it by
# rsync from this repo. On the Pi, this repo's controller/X lives at ~/running_robot/robot/X.
#
#   ./controller/deploy_to_pi.sh                  deploy HEAD (asks before copying and before restarting)
#   ./controller/deploy_to_pi.sh --check          show what would be deployed, write nothing
#   ./controller/deploy_to_pi.sh --rollback       restore the last backup
#
# What it does, stopping at the first surprise:
#   1. reads which commit the Pi runs (~/deploy_backups/deployed_commit, or --from)
#   2. lists the controller/ files that changed between that commit and the target (tests excluded)
#   3. checks each of those files on the Pi still IS the deployed version (line endings ignored);
#      a file edited on the Pi, or a Pi that is not on the commit you think, aborts the deploy
#   4. backs up exactly the files it is about to replace or delete (~/deploy_backups/<name>/)
#   5. rsyncs the target versions (taken from git, not from the working tree), deletes removed files
#   6. checks md5 Pi == git, py_compile on .py, json.tool on .json, then records the new commit
#   7. restarts the service if asked, and checks the journal for errors
#
# It never talks to the motors: no /api call, no run_policy.py, no CAN. Restarting the service
# releases the motors (0 A), so the robot must be supported or lying down when you say yes.
#
# Bash 3.2 compatible (macOS default shell). Configuration through the environment:
#   DASH_PI_HOST   (nemo@192.168.4.1)       DASH_PI_DIR  (running_robot/robot, relative to the Pi home)
#   DASH_PI_PY     (running_robot/.venv/bin/python)   DASH_PI_SERVICE (runningrobot-webui)
#   DASH_PI_BACKUPS (deploy_backups, relative to the Pi home; holds backups, history.log, deployed_commit)
#   DASH_PI_TEST=1  allow a DASH_PI_DIR the service does not run (to try the script on a scratch copy)
set -euo pipefail

HOST="${DASH_PI_HOST:-nemo@192.168.4.1}"
PI_DIR="${DASH_PI_DIR:-running_robot/robot}"
PI_PY="${DASH_PI_PY:-running_robot/.venv/bin/python}"
SERVICE="${DASH_PI_SERVICE:-runningrobot-webui}"
BACKUPS="${DASH_PI_BACKUPS:-deploy_backups}"   # in the Pi home
MARKER="$BACKUPS/deployed_commit"
EXCLUDE_RE='(^|/)tests/|^deploy_to_pi[.]sh$'   # never deployed (paths relative to controller/)

usage() {
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options:
  --to <commit>       commit to deploy (default HEAD)
  --from <commit>     commit the Pi runs now (default: read from the Pi; needed on the first run)
  --check             dry run: connect, compare, show the plan, write nothing
  --yes               do not ask before copying (the restart still asks)
  --restart           restart the service without asking      --no-restart  never restart
  --force             copy even if files on the Pi differ from --from (they are backed up first)
  --rollback [name]   restore a backup from ~/deploy_backups (default: the latest)
  -h, --help
EOF
}

TO="HEAD"; FROM=""; MODE="deploy"; YES=0; RESTART="ask"; FORCE=0; RB_NAME=""
while [ $# -gt 0 ]; do
  case "$1" in
    --to) TO="${2:?--to needs a commit}"; shift 2 ;;
    --from) FROM="${2:?--from needs a commit}"; shift 2 ;;
    --check) MODE="check"; shift ;;
    --yes) YES=1; shift ;;
    --restart) RESTART="yes"; shift ;;
    --no-restart) RESTART="no"; shift ;;
    --force) FORCE=1; shift ;;
    --rollback) MODE="rollback"; shift
                if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then RB_NAME="$1"; shift; fi ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ -t 1 ]; then B=$'\e[1m'; R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'; else B=""; R=""; G=""; Y=""; N=""; fi
say()  { printf '%s\n' "${B}==>${N} $*"; }
ok()   { printf '%s\n' "  ${G}ok${N} $*"; }
warn() { printf '%s\n' "  ${Y}warning:${N} $*" >&2; }
die()  { printf '%s\n' "${R}STOP:${N} $*" >&2; exit 1; }
ask()  { local a; read -r -p "$1 [y/N] " a; case "$a" in y|Y|yes|o|O|oui) return 0 ;; *) return 1 ;; esac; }

for v in "$HOST" "$PI_DIR" "$PI_PY" "$SERVICE" "$BACKUPS"; do
  case "$v" in *[[:space:]\'\"\`\$\\]*) die "configuration value with spaces or quotes: '$v'" ;; esac
done

REPO="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
cd "$REPO"

T="${TMPDIR:-/tmp}"; WORK="$(mktemp -d "${T%/}/deploy_to_pi.XXXXXX")"
CM="$(mktemp -d /tmp/dtp.XXXXXX)"        # short path: ssh control sockets are limited to ~100 chars
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$CM/s" -o ControlPersist=300 -o ConnectTimeout=10)
cleanup() { ssh -o "ControlPath=$CM/s" -O exit "$HOST" >/dev/null 2>&1 || true; rm -rf "$WORK" "$CM"; }
trap cleanup EXIT
pi()  { ssh "${SSH_OPTS[@]}" "$HOST" "$@"; }
pit() { ssh -t "${SSH_OPTS[@]}" "$HOST" "$@"; }

connect() {
  say "Connecting to $HOST"
  pi true || die "cannot reach $HOST over ssh"
  pi "test -f $PI_DIR/fixed_gait/webui/server.py" \
    || die "~/$PI_DIR/fixed_gait/webui/server.py not found on the Pi (wrong DASH_PI_DIR?)"
  # the service must run exactly ~/$PI_DIR/fixed_gait/webui/server.py, or copying there changes nothing
  local es wd home script got want
  es="$(pi "systemctl show -p ExecStart --value $SERVICE" 2>/dev/null || true)"
  wd="$(pi "systemctl show -p WorkingDirectory --value $SERVICE" 2>/dev/null || true)"
  home="$(pi 'echo $HOME')"
  script="$(printf '%s' "$es" | sed -n 's/.*argv\[\]=[^ ]* \([^ ;]*\).*/\1/p')"
  case "$script" in /*) got="$script" ;; *) got="${wd%/}/$script" ;; esac
  want="$home/$PI_DIR/fixed_gait/webui/server.py"
  if [ "$got" = "$want" ]; then
    ok "service $SERVICE runs $want"
  elif [ "${DASH_PI_TEST:-0}" = 1 ]; then
    warn "DASH_PI_TEST=1: service runs $got, deploying to $want anyway (test directory)"
  else
    die "service $SERVICE runs ${got:-<unknown>}, not $want — copying there would change nothing.
  ExecStart: ${es:-<unit not found>}"
  fi
}

# same <a> <b>: identical bytes, or identical text once CR are removed (the Pi has CRLF copies)
same() {
  cmp -s "$1" "$2" && return 0
  LC_ALL=C grep -Iq . "$1" 2>/dev/null && LC_ALL=C grep -Iq . "$2" 2>/dev/null || return 1
  cmp -s <(tr -d '\r' < "$1") <(tr -d '\r' < "$2")
}

restart_service() {   # $1 = why
  local need="$1"
  if [ "$RESTART" = "no" ]; then
    warn "not restarting ($need). Later: ssh $HOST 'sudo systemctl restart $SERVICE'"; return 0
  fi
  if [ "$RESTART" = "ask" ]; then
    echo
    echo "${Y}Restarting releases the motors (0 A). The robot must be supported or lying down.${N}"
    ask "Restart $SERVICE now?" || { warn "not restarted. Later: ssh $HOST 'sudo systemctl restart $SERVICE'"; return 0; }
  fi
  say "Restarting $SERVICE"
  local t0 n0 n1 state
  t0="$(pi 'date +%s')"
  n0="$(pi "systemctl show -p NRestarts --value $SERVICE" || echo 0)"
  if ! pi "sudo -n systemctl restart $SERVICE" 2>/dev/null; then
    echo "  sudo needs a password on the Pi:"
    pit "sudo systemctl restart $SERVICE" || die "restart failed"
  fi
  sleep 8
  state="$(pi "systemctl is-active $SERVICE" || true)"
  n1="$(pi "systemctl show -p NRestarts --value $SERVICE" || echo 0)"
  pi "journalctl -u $SERVICE --since @$t0 --no-pager" > "$WORK/journal" || true
  echo "----- journal since the restart (last 40 lines) -----"
  tail -n 40 "$WORK/journal"
  echo "-----------------------------------------------------"
  local bad=""
  [ "$state" = "active" ] || bad="service is '$state'"
  [ "${n1:-0}" -le "${n0:-0}" ] || bad="${bad:+$bad; }service crashed and restarted itself ($n0 -> $n1)"
  if grep -Eq 'Traceback|Error|Exception|No module named|exited, code|failed' "$WORK/journal"; then
    bad="${bad:+$bad; }errors in the journal:"
    grep -En 'Traceback|Error|Exception|No module named|exited, code|failed' "$WORK/journal" | head -10 >&2
  fi
  [ -z "$bad" ] || die "$bad
  Rollback: ./controller/deploy_to_pi.sh --rollback"
  ok "service active, no error in the journal"
}

# ---------------------------------------------------------------------------------- rollback
if [ "$MODE" = "rollback" ]; then
  connect
  if [ -z "$RB_NAME" ]; then
    RB_NAME="$(pi "cd $BACKUPS 2>/dev/null && ls -1d */ 2>/dev/null | sed 's#/\$##' | sort | tail -1" || true)"
    [ -n "$RB_NAME" ] || die "no backup in ~/$BACKUPS on the Pi"
  fi
  case "$RB_NAME" in */*|*[[:space:]]*|"") die "bad backup name: $RB_NAME" ;; esac
  pi "test -f $BACKUPS/$RB_NAME/files.tgz" || die "~/$BACKUPS/$RB_NAME/files.tgz not found"
  say "Backup $RB_NAME"
  pi "cat $BACKUPS/$RB_NAME/info"
  echo "  restores:"; pi "tar tzf $BACKUPS/$RB_NAME/files.tgz" | sed 's/^/    /'
  echo "  removes (added by that deploy):"; pi "cat $BACKUPS/$RB_NAME/added.txt" | sed 's/^/    /'
  [ "$YES" = 1 ] || ask "Restore this backup on the Pi?" || die "nothing done"
  pi "set -e; cd $PI_DIR; tar xzf ~/$BACKUPS/$RB_NAME/files.tgz;
      while IFS= read -r f; do [ -n \"\$f\" ] && rm -f -- \"\$f\"; done < ~/$BACKUPS/$RB_NAME/added.txt;
      cp ~/$BACKUPS/$RB_NAME/from_commit ~/$MARKER;
      echo \"$(date '+%Y-%m-%d %H:%M:%S') rollback $RB_NAME -> \$(cat ~/$MARKER)\" >> ~/$BACKUPS/history.log"
  ok "files restored; the Pi is recorded as running $(git rev-parse --short "$(pi "cat $MARKER")" 2>/dev/null || pi "cat $MARKER")"
  restart_service "rollback"
  exit 0
fi

# ---------------------------------------------------------------------------------- deploy / check
TO_SHA="$(git rev-parse --verify --quiet "$TO^{commit}")" || die "unknown commit: $TO"
if [ "$TO" = "HEAD" ] && [ -n "$(git status --porcelain -- controller)" ]; then
  warn "uncommitted changes in controller/ are NOT deployed (only commits are). Commit them first:"
  git status --short -- controller | head -10 >&2
fi

connect

PI_MARK="$(pi "cat $MARKER 2>/dev/null" || true)"
if [ -z "$FROM" ]; then
  [ -n "$PI_MARK" ] || die "the Pi does not say which commit it runs (~/$MARKER missing).
  Pass it once: --from <commit>   (2026-11-17: 830017f)"
  FROM="$PI_MARK"
elif [ -n "$PI_MARK" ] && [ "$(git rev-parse --verify --quiet "$FROM^{commit}" || true)" != "$PI_MARK" ]; then
  warn "--from $FROM overrides the Pi's record ($(echo "$PI_MARK" | cut -c1-7))"
fi
FROM_SHA="$(git rev-parse --verify --quiet "$FROM^{commit}")" \
  || die "commit $FROM (running on the Pi) is not in this repo — git fetch?"
F7="$(git rev-parse --short "$FROM_SHA")"; T7="$(git rev-parse --short "$TO_SHA")"

say "Pi runs $F7 — $(git log -1 --format=%s "$FROM_SHA")"
say "Target   $T7 — $(git log -1 --format=%s "$TO_SHA")"
[ "$FROM_SHA" != "$TO_SHA" ] || { ok "nothing to deploy, the Pi is already on $T7"; exit 0; }
git merge-base --is-ancestor "$FROM_SHA" "$TO_SHA" \
  || warn "$T7 does not descend from $F7 (going back, or another branch)"

# changed files, relative to controller/ (no rename detection: a rename = delete + add)
git diff --no-renames --name-status "$FROM_SHA" "$TO_SHA" -- controller/ \
  | awk -F'\t' '{ sub(/^controller\//, "", $2); print $1 "\t" $2 }' > "$WORK/all"
awk -F'\t' -v re="$EXCLUDE_RE" '$2 ~ re'    "$WORK/all" > "$WORK/skipped"
awk -F'\t' -v re="$EXCLUDE_RE" '!($2 ~ re)' "$WORK/all" > "$WORK/changes"
[ -s "$WORK/changes" ] || { ok "no deployable change in controller/ between $F7 and $T7"; exit 0; }
if cut -f2 "$WORK/changes" | grep -q '[[:space:]"'"'"'`$\\]'; then die "a changed path has spaces or quotes; not supported"; fi
awk -F'\t' '$1 ~ /^[AMT]$/ { print $2 }' "$WORK/changes" > "$WORK/put"
awk -F'\t' '$1 == "D"      { print $2 }' "$WORK/changes" > "$WORK/del"
if awk -F'\t' '$1 !~ /^[AMTD]$/' "$WORK/changes" | grep -q .; then die "unexpected git status in: $(cat "$WORK/changes")"; fi
cut -f2 "$WORK/changes" > "$WORK/touch"

# what the Pi has now, for every file we would replace or delete
say "Fetching the Pi's copy of $(wc -l < "$WORK/touch" | tr -d ' ') file(s)"
mkdir -p "$WORK/pi" "$WORK/from" "$WORK/to"
pi "cd $PI_DIR && while IFS= read -r f; do [ -e \"\$f\" ] && printf '%s\n' \"\$f\"; done | tar cf - -T -" \
  < "$WORK/touch" | tar xf - -C "$WORK/pi"
if [ -s "$WORK/put" ]; then
  # shellcheck disable=SC2046
  git archive --format=tar "$TO_SHA" $(sed 's#^#controller/#' "$WORK/put") | tar xf - -C "$WORK/to"
fi
while IFS= read -r p; do
  if git cat-file -e "$FROM_SHA:controller/$p" 2>/dev/null; then
    mkdir -p "$WORK/from/$(dirname "$p")"; git show "$FROM_SHA:controller/$p" > "$WORK/from/$p"
  fi
done < "$WORK/touch"

DIFFS="${TMPDIR:-/tmp}"; DIFFS="${DIFFS%/}/deploy_to_pi_diffs_$(date +%Y%m%d_%H%M%S)"
: > "$WORK/added"; : > "$WORK/problems"
while IFS="$(printf '\t')" read -r st p; do
  piv="$WORK/pi/$p"; fromv="$WORK/from/$p"; tov="$WORK/to/controller/$p"
  if [ ! -e "$piv" ]; then
    [ "$st" = "A" ] && { echo "$p" >> "$WORK/added"; continue; }
    [ "$st" = "D" ] && continue                                   # already gone
    echo "$p: missing on the Pi (expected the $F7 version)" >> "$WORK/problems"; continue
  fi
  if [ -e "$fromv" ] && same "$piv" "$fromv"; then continue; fi    # Pi == deployed version
  if [ -e "$tov" ] && same "$piv" "$tov"; then continue; fi        # already the target version
  if [ "$st" = "A" ]; then echo "$p: exists on the Pi with other content (new in $T7)" >> "$WORK/problems"
  else echo "$p: on the Pi it differs from $F7" >> "$WORK/problems"; fi
  mkdir -p "$DIFFS/$(dirname "$p")"
  [ -e "$fromv" ] || fromv=/dev/null
  diff -u --strip-trailing-cr "$fromv" "$piv" > "$DIFFS/$p.diff" 2>/dev/null || true
done < "$WORK/changes"

echo
say "Plan: $F7 -> $T7 on $HOST:~/$PI_DIR"
git log --oneline "$FROM_SHA..$TO_SHA" -- controller/ 2>/dev/null | head -15 | sed 's/^/    /'
sed 's/^A/  + new    /; s/^M/  ~ change /; s/^T/  ~ change /; s/^D/  - delete /' "$WORK/changes"
[ -s "$WORK/skipped" ] && echo "  ($(wc -l < "$WORK/skipped" | tr -d ' ') test/script file(s) not deployed)"
if grep -q 'deploy/deploy_map.json' "$WORK/touch"; then
  warn "deploy_map.json changes: the joint signs are NOT verified — run check_direction with the robot suspended."
fi
if [ -s "$WORK/problems" ]; then
  echo
  echo "${R}The Pi is not on $F7 for these files:${N}" >&2
  sed 's/^/  /' "$WORK/problems" >&2
  [ -d "$DIFFS" ] && { echo "  diffs (${F7} -> Pi): $DIFFS" >&2; find "$DIFFS" -name '*.diff' -exec head -n 30 {} \; >&2; }
  [ "$FORCE" = 1 ] || die "nothing copied. Look at the diffs; if the Pi's versions can be thrown away, re-run with --force (they are backed up first)."
  warn "--force: replacing them anyway"
fi

if [ "$MODE" = "check" ]; then echo; ok "check only: nothing written on the Pi"; exit 0; fi
echo
[ "$YES" = 1 ] || ask "Copy to the Pi?" || die "nothing copied"

# ---- backup exactly what will be replaced or deleted
NAME="$(date +%Y%m%d_%H%M%S)_${F7}_to_${T7}"       # Mac clock: the Pi's clock is wrong (no NTP)
say "Backup -> ~/$BACKUPS/$NAME"
(cd "$WORK/pi" && find . -type f | sed 's#^\./##' | sort) > "$WORK/existing"
pi "mkdir -p $BACKUPS/$NAME"
pi "cd $PI_DIR && tar czf ~/$BACKUPS/$NAME/files.tgz -T -" < "$WORK/existing"
pi "cat > $BACKUPS/$NAME/added.txt" < "$WORK/added"
echo "$FROM_SHA" | pi "cat > $BACKUPS/$NAME/from_commit"
printf 'deploy %s -> %s\nmac time %s\nfrom: %s\nto:   %s\n' "$F7" "$T7" "$(date '+%Y-%m-%d %H:%M:%S')" \
  "$(git log -1 --format=%s "$FROM_SHA")" "$(git log -1 --format=%s "$TO_SHA")" | pi "cat > $BACKUPS/$NAME/info"
n_bk="$(pi "tar tzf $BACKUPS/$NAME/files.tgz | wc -l" | tr -d ' ')"
[ "$n_bk" = "$(wc -l < "$WORK/existing" | tr -d ' ')" ] || die "backup has $n_bk files, expected $(wc -l < "$WORK/existing")"
ok "$n_bk file(s) saved, $(wc -l < "$WORK/added" | tr -d ' ') new file(s) recorded"

# ---- copy and delete
FAIL_HINT="Rollback: ./controller/deploy_to_pi.sh --rollback $NAME"
if [ -s "$WORK/put" ]; then
  say "Copying $(wc -l < "$WORK/put" | tr -d ' ') file(s)"
  # shellcheck disable=SC2046   (paths were checked: no spaces or quotes)
  (cd "$WORK/to/controller" && rsync -aR -e "ssh -o ControlMaster=auto -o ControlPath=$CM/s" \
    $(cat "$WORK/put") "$HOST:$PI_DIR/") || die "rsync failed. $FAIL_HINT"
fi
if [ -s "$WORK/del" ]; then
  say "Deleting $(wc -l < "$WORK/del" | tr -d ' ') file(s)"
  pi "cd $PI_DIR && while IFS= read -r f; do rm -f -- \"\$f\"; done" < "$WORK/del" || die "delete failed. $FAIL_HINT"
fi

# ---- verify
say "Verifying"
if [ -s "$WORK/put" ]; then
  (cd "$WORK/to/controller" && while IFS= read -r f; do printf '%s %s\n' "$(md5 -q "$f")" "$f"; done < "$WORK/put") | sort > "$WORK/md5.mac"
  pi "cd $PI_DIR && xargs md5sum" < "$WORK/put" | awk '{ print $1, $2 }' | sort > "$WORK/md5.pi"
  diff "$WORK/md5.mac" "$WORK/md5.pi" >&2 || die "md5 on the Pi differs from git. $FAIL_HINT"
  ok "md5 Pi == git ($(wc -l < "$WORK/put" | tr -d ' ') files)"
fi
if [ -s "$WORK/del" ]; then
  left="$(pi "cd $PI_DIR && while IFS= read -r f; do [ -e \"\$f\" ] && echo \"\$f\"; done" < "$WORK/del" || true)"
  [ -z "$left" ] || die "still present on the Pi: $left. $FAIL_HINT"
  ok "deleted files are gone"
fi
grep '\.py$'   "$WORK/put" > "$WORK/py"   || true
grep '\.json$' "$WORK/put" > "$WORK/json" || true
if [ -s "$WORK/py" ]; then
  pi "cd $PI_DIR && xargs \$HOME/$PI_PY -m py_compile" < "$WORK/py" || die "py_compile failed. $FAIL_HINT"
  ok "py_compile ($(wc -l < "$WORK/py" | tr -d ' ') files)"
fi
if [ -s "$WORK/json" ]; then
  pi "cd $PI_DIR && while IFS= read -r f; do \$HOME/$PI_PY -m json.tool \"\$f\" > /dev/null || { echo \"invalid JSON: \$f\"; exit 1; }; done" \
    < "$WORK/json" || die "JSON check failed. $FAIL_HINT"
  ok "json valid ($(wc -l < "$WORK/json" | tr -d ' ') files)"
fi
echo "$TO_SHA" | pi "cat > $MARKER"
echo "$(date '+%Y-%m-%d %H:%M:%S') deploy $F7 -> $T7 backup=$NAME" | pi "cat >> $BACKUPS/history.log"
ok "the Pi is recorded as running $T7"

# ---- restart (only if something the daemon loads changed; static files are read per request)
if grep -qv '^fixed_gait/webui/static/' "$WORK/touch"; then
  restart_service "Python/data changed"
else
  ok "only static files changed: no restart needed"
fi
echo
say "Done. Reload the web UI with Cmd+Shift+R."
echo "  If something is wrong: ./controller/deploy_to_pi.sh --rollback $NAME"
