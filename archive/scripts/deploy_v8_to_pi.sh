#!/bin/bash
# Deploy commits 1481932 + 9eaa4e1 (twin v8 + Home = v8 export pose) to the DASH-01 Pi.
# Run from your Mac, on the robot's Wi-Fi:   bash ~/Desktop/EPFL/Projets/Dash-01/code/deploy_v8_to_pi.sh
# Applies ONLY the diff 95ea100..9eaa4e1 to the Pi's own files (keeps whatever was done on the Pi),
# keeps their line endings, replaces static/twin/ whole, restarts the services only if LIMP.
set -euo pipefail
PI=${PI:-nemo@192.168.4.1}
REMOTE=running_robot/robot/fixed_gait/webui
REPO="$(cd "$(dirname "$0")/DASH-01" && pwd)"
W=controller/fixed_gait/webui
FROM=95ea100; TO=9eaa4e1
FILES="daemon.py static/twin3d.js static/index.html"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
say(){ printf '\n== %s\n' "$*"; }

cd "$REPO"
git cat-file -e $TO 2>/dev/null || { echo "commit $TO absent du repo"; exit 1; }

say "1. Pi joignable, mode du robot"
MODE=$(ssh -o ConnectTimeout=8 "$PI" "curl -s http://127.0.0.1:8081/api/state | python3 -c 'import json,sys; print(json.load(sys.stdin)[\"mode\"])'" || echo "?")
echo "mode = $MODE"

say "2. Sauvegarde sur le Pi"
# code only: data/ (black-box logs, written live while the daemon runs) is left out; tar exit 1
# ("file changed as we read it") is a warning, not a failure
ssh "$PI" "cd ~/running_robot/robot/fixed_gait && { tar czf ~/backup_webui_before_v8_\$(date +%s).tar.gz --exclude=webui/data webui; rc=\$?; [ \$rc -le 1 ]; } && ls -lt ~/backup_webui_before_v8_* | head -1"

say "3. Patch des fichiers du Pi (diff $FROM..$TO uniquement)"
for f in $FILES; do
  mkdir -p "$TMP/$(dirname $f)"
  scp -q "$PI:$REMOTE/$f" "$TMP/$f.pi"
  if grep -q $'\r' "$TMP/$f.pi"; then CRLF=1; else CRLF=0; fi
  perl -pe 's/\r\n/\n/' "$TMP/$f.pi" > "$TMP/$f"
  git diff $FROM $TO -- "$W/$f" > "$TMP/$f.diff"
  if patch --dry-run -s -N "$TMP/$f" "$TMP/$f.diff" >/dev/null; then
    patch -s -N "$TMP/$f" "$TMP/$f.diff"
    [ $CRLF = 1 ] && perl -pi -e 's/\n/\r\n/' "$TMP/$f"
    echo "ok    $f (CRLF=$CRLF)"
  else
    echo "ÉCHEC $f : le fichier du Pi a divergé, le patch ne s'applique pas."
    [ "$f" = "static/index.html" ] && { echo "      (index.html = texte d'infobulle seulement, on continue sans)"; rm "$TMP/$f"; continue; }
    echo "      rien n'a été copié. Arrêt."; exit 1
  fi
done
grep -q 'dash01_homing_v8.xml' "$TMP/static/twin3d.js"
grep -q '"left.cam": 55.47' "$TMP/daemon.py"

say "4. Copie : static/twin/ complet + fichiers patchés"
( cd "$W" && COPYFILE_DISABLE=1 tar cf - --no-xattrs --no-mac-metadata static/twin ) \
  | ssh "$PI" "cd ~/$REMOTE && rm -rf static/twin && tar xmf -"
for f in $FILES; do [ -f "$TMP/$f" ] && scp -q "$TMP/$f" "$PI:$REMOTE/$f"; done
ssh "$PI" "cd ~/$REMOTE && python3 -m py_compile daemon.py && grep -n 'STAND_POSE_DEG = ' -A1 daemon.py && grep -n TWIN_XML static/twin3d.js | head -1 && ls static/twin"

say "5. Redémarrage des services"
if [ "$MODE" = "LIMP" ]; then
  ssh "$PI" "sudo systemctl restart runningrobot-webui runningrobot-ui && sleep 4 && systemctl is-active runningrobot-webui runningrobot-ui && journalctl -u runningrobot-webui --since '-1 min' --no-pager | tail -15"
  echo; echo "Fait. Recharge la page (http://192.168.4.1:8080, Cmd+Shift+R pour vider le cache)."
else
  echo "Robot en mode '$MODE', PAS LIMP : fichiers copiés mais services NON redémarrés."
  echo "Robot soutenu → passe en LIMP, puis :  ssh $PI sudo systemctl restart runningrobot-webui runningrobot-ui"
fi
