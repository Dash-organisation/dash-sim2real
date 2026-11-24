# DASH-01 — Marche à suivre : outil CubeMars et re-test du sifflement MIT

Pour la séance qui suit le 24/11. Contexte : `DASH01_journal_2026-11-24.md` (§1 sécurité, §4.3
observations, §7). Le but : trouver un réglage des drives AKE90-8 qui supprime l'oscillation à 2,5 kHz
du mode force control, puis le vérifier avec les mêmes tests que le 24/11.

**Règle d'or : un seul changement à la fois, et un test après chaque changement.**

---

## 0. Matériel et sécurité (5 min)

- [ ] Batterie chargée, **chargeur débranché**.
- [ ] Robot dans le harnais, harnais tendu, E-STOP à portée de main.
- [ ] Téléphone avec l'appli de spectrogramme, posé à environ 20 cm de la came droite.
- [ ] Un terminal sur le Pi (`cd ~/running_robot/robot`). Interface web ouverte **seulement** pour le
      MIT hold, et fermée avant tout `policy_cli.py run`.
- [ ] La règle du 24/11 : **en MIT, au premier gros sifflement → LIMP.** Il ne s'arrête pas seul.

## 1. Relever l'état actuel, avant toute modification (15 min)

Pour **chacun des six drives**, relève les valeurs ci-dessous. Note aussi le bus (`can0` / `can1`)
et l'id CAN que l'outil affiche, pour être sûr de quel drive il s'agit. Le journal du 20/11 cite
`left.abd` = 104, `left.thigh` = 106 et `right.cam` = 0x69 sur `can0`.

| drive | moteur | version firmware | cur kp | cur ki | spd kp | spd ki | pos kp | réglages force control / MIT (lesquels ?) | PWM (kHz) |
|---|---|---|---|---|---|---|---|---|---|
| right.abd | AK60-39 | | | | | | | | |
| right.cam | AKE90-8 | | | | | | | | |
| right.thigh | AKE90-8 | | | | | | | | |
| left.abd | AK60-39 | | | | | | | | |
| left.cam | AKE90-8 | | | | | | | | |
| left.thigh | AKE90-8 | | | | | | | | |

- [ ] Exporte la configuration de chaque drive si l'outil le permet. Sinon, prends une photo de
      chaque écran. Range le tout dans `docs/sim2real/Logs/DASH01_cubemars_<date>/`.
- [ ] Note **tout ce qui diffère** entre les AK60-39 (qui ne sifflent pas) et les AKE90-8.
- [ ] Les valeurs du 04/08 sont dans `controller/fixed_gait/webui/dynstore.py` (`DRIVE_GAINS`).
      Vérifie qu'elles n'ont pas bougé.

## 2. Le test de référence : le test d'appui (10 min, à faire avant ET après chaque changement)

Robot **suspendu**, pieds libres.

1. `python3 tools/policy_cli.py pose kdval_B_kd5.npz` (ou Home), puis attendre `arrived`.
2. Interface web : **MIT hold activé**, came et cuisse à **kp 50 / kd 1**, abduction à 100 / 2.
3. Spectrogramme : 10 s **sans toucher**. Note la présence de la raie à 2,5 kHz et son niveau.
4. Appuie sur le pied **progressivement**, toujours de la même façon. Note à partir de quand ça
   s'emballe : léger, moyen ou fort appui. Si tu as un peson, la force exacte.
5. Dès que ça s'emballe : **LIMP**.
6. Même chose sur l'autre jambe.

Fiche :

| essai | changement | jambe | raie 2,5 kHz au repos (non / faible / forte) | seuil d'emballement (léger / moyen / fort / jamais) | T max (°C) | remarque |
|---|---|---|---|---|---|---|
| réf. | aucun | G | | | | |
| réf. | aucun | D | | | | |
| 1 | | | | | | |
| 2 | | | | | | |

## 3. Essayer un réglage (une carte à la fois)

Ordre conseillé, du moins risqué au plus risqué. Teste sur **une seule cuisse ou came**, et compare
avec l'autre jambe, qui sert de témoin.

1. **Réglage propre au force control / MIT**, s'il en existe un dans l'outil (gains, filtre, bande
   passante). C'est la question posée au support.
2. **Relancer l'autotuning de la boucle de courant** du drive. L'écart G/D actuel vient
   probablement d'un autotuning par carte.
3. **Baisser les gains de la boucle de courant** à la main, par exemple cur kp et ki divisés par deux.
4. **Fréquence de PWM**, si elle est réglable.
5. **Mise à jour du firmware**, en dernier. Après une mise à jour, **avant tout mouvement** :
   vérifie que le force control répond encore au format de `controller/deploy/mit.py` (trame
   étendue mode 8, ordre KP, KD, P, V, T). Commence par un MIT hold à kp 0 / kd 0 sur la jambe
   suspendue, puis kp 20 : la jambe doit être molle, puis légèrement tenue. Rien d'autre.

Après chaque changement : **refais le §2** et remplis une ligne de la fiche.
- Le seuil monte nettement, ou l'emballement disparaît : garde ce réglage et applique-le à l'autre
  jambe.
- Rien ne change : reviens au réglage d'origine (§1) avant d'essayer le suivant.

## 4. Valider debout, une fois le test d'appui réussi (15 min)

Seulement si le §2 ne s'emballe plus, même avec un appui fort. Robot posé, harnais tendu, LIMP et
E-STOP prêts.

1. **SET_POS debout** : `pose kdval_B_kd5.npz`, robot posé, 10 s d'audio. C'est la référence.
2. **MIT hold debout**, kp 200 / kd 1 sur came et cuisse : 10 s d'audio. Au moindre emballement,
   LIMP.
3. Si c'est calme, **un run figé** (interface web fermée) :
   ```bash
   python3 tools/policy_cli.py pose kdval_D_kd1.npz
   python3 tools/policy_cli.py run  kdval_D_kd1.npz --allow-uncalibrated-thermal --seconds 10
   ```
   puis la même chose avec `kdval_B_kd5.npz`. Ctrl+C une fois = arrêt doux, deux fois = arrêt dur.

Le 24/11, D et B donnaient une ondulation de 6 à 7 A sur les cames. Un réglage qui fonctionne doit
la ramener près de celle du SET_POS (< 0,1 A).

## 5. Récupérer et analyser (depuis le Mac, dans `code/dash-sim2real`)

```bash
R=docs/sim2real/Logs/DASH01_cubemars_<date>
mkdir -p $R/raw
scp "nemo@192.168.4.1:running_robot/robot/fixed_gait/webui/data/policyruns/policyrun_*kdval*" $R/raw/
python3 tools/kd_validation/analyze_runs.py $R/raw/<les nouveaux runs>.npz \
    docs/sim2real/Logs/DASH01_kd_2026-11-24/raw/policyrun_20260822_111817_kdval_D_kd1.npz \
    --labels D_new D_24nov --out $R
```

Regarde `I_ripple_A` et `v_flip` des cames et cuisses dans `$R/summary.md`, avant et après. Envoie
ensuite à Claude la fiche du §2, le tableau du §1 et `summary.md` pour compléter le journal.

## 6. Fin de séance

- [ ] `python3 tools/policy_cli.py limp`, robot tenu par le harnais.
- [ ] Interface web : **MIT hold désactivé**. Comme le réglage est enregistré dans
      `data/balance.json`, il resterait actif à la prochaine séance.
- [ ] Noter quel réglage est maintenant sur chaque drive (§1), surtout si ce n'est plus celui
      d'origine.
