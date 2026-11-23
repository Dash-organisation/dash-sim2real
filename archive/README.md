# archive/ — copies figées, rien d'actif

Rien ici n'est lu par le code. Les fichiers actifs restent à leur place.

- `models/standing/` : copie figée des trois plants de STAND-03/04 (`stand_v1.xml`, `stand_v2.xml`, `stand_xshape.xml`), avec leurs références au repos (`quiet_*.json`) et leur générateur (`build_three_standing.py`), copiés depuis `BalanceRL/model/` à l'import de DASH-01@3574ddd. Sert à reproduire la comparaison des trois plants. Les originaux restent dans `BalanceRL/model/`, parce que `BalanceRL/config.py` construit ses presets `stand_*` et `recover_*` à partir de leur chemin. `stand_xshape.xml` est le plant de la policy déployée `standing04_xshape_best`.
- `scripts/deploy_v8_to_pi.sh` : déploiement ponctuel du diff `95ea100..9eaa4e1` (twin v8), fichier daté du 09/11. Il est remplacé par `controller/deploy_to_pi.sh`. **Ne pas relancer.**

L'historique complet (stacks de Nathann, `sprint_runner/`, homing v7, toutes les branches) est dans `DASH-01-archive`.
