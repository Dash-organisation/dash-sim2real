# DASH-01 — Plan de tests : le sifflement des moteurs pendant la politique

Version 1, 2026-11-20. Suite de `Logs/ANALYSE_bruit_moteurs.md` (runs 3 et 4 du 17/11).

---

## 1. Le problème et les hypothèses

Pendant la politique, les moteurs émettent un bruit strident et aigu. Ce que montrent les logs des
runs 3 et 4 :

- **ce n'est pas une limite de courant** : les cames montent à 22–27 A (un pic à 52 A), alors que
  les mêmes drivers ont sorti ~100 A dans la même session ; aucun code d'erreur, aucune saturation ;
- **le courant des cames ondule de 4 à 6 A rms** en POLICY, contre 0.01–0.07 A en SET_POS immobile.
  À vitesse égale, c'est 4 à 50 fois plus qu'en SET_POS. Les cuisses ondulent 3 à 5 fois moins ;
- **la came et la cuisse se battent à travers le pied** : la politique place la consigne de la came
  3 à 7° devant sa position et celle de la cuisse 2 à 5° derrière. Leurs puissances sont de signe
  opposé 52 à 80 % du temps en POLICY, contre 0 à 5 % en SET_POS. Le bruit de la came augmente avec
  cette lutte (corrélation de 0.2 à 0.6).

La black box enregistre à 100 Hz et le sifflement est bien au-dessus : les logs ne suffisent pas à
conclure. Trois hypothèses restent ouvertes :

| | Hypothèse | Mécanisme |
|---|---|---|
| **H1** | les gains MIT | kd = 5 (le maximum du driver) et kp = 200 amplifient le bruit de la vitesse estimée par le driver et le jeu de la transmission de la came : la came vibre seule |
| **H2** | la lutte came/cuisse | deux boucles raides qui se poussent à travers la chaîne fermée par le pied entrent en vibration ensemble |
| **H3** | les deux | les gains rendent la came prête à vibrer, la lutte lui donne l'énergie |

Les quatre tests ajoutent les ingrédients un par un : d'abord les gains seuls, puis la chaîne
fermée, puis la lutte, puis la lutte avec un kd réduit.

| Test | Pied au sol | Lutte came/cuisse | Gains | Isole |
|---|---|---|---|---|
| 1 | non (suspendu) | non | kp 200, kd 5 → 2 → 1 | les gains seuls (H1) |
| 2 | oui | non (seulement le poids) | kp 200, kd 5 | gains + chaîne fermée |
| 3 | oui | oui (came +3°, cuisse −3° en plus) | kp 200, kd 5 | la lutte (H2) |
| 4 | oui | oui, comme le test 3 | kp 200, **kd 2** | si baisser kd suffit malgré la lutte |

---

## 2. Matériel et mesure

### 2.1 Le son (la mesure principale)

- Téléphone avec une application de **spectrogramme** qui enregistre aussi un fichier audio
  (Spectroid sur Android, SpectrumView ou équivalent sur iOS).
- **Même position à chaque test** : téléphone fixé (pince, ruban adhésif) à ~20 cm de la **came
  droite**, micro tourné vers le moteur. Ne pas le tenir à la main.
- Au début de chaque étape, **annoncer à voix haute** le numéro du test et les gains (« test 1,
  kd 5 »). L'audio se recale ainsi sur la black box.
- Pour chaque étape, noter : présence du sifflement (oui / non / faible), **fréquence du pic** lue
  sur le spectrogramme, niveau en dB s'il est affiché.

### 2.2 La black box

- L'anneau garde ~40 s à 100 Hz. `/api/blackbox/mark` écrit un dump des ~37 s précédentes et des 3 s
  suivantes, avec le texte de la marque dans les événements.
- **Poser une marque à la fin de chaque étape**, pas au début :

```bash
curl -s -X POST http://<pi>:8080/api/blackbox/mark \
     -H 'Content-Type: application/json' -d '{"text":"T1 kp200 kd5 suspendu"}'
```

- Garder chaque étape **sous 30 s**, pour qu'un dump la contienne en entier.
- Les dumps se récupèrent comme d'habitude (`/api/blackbox/list`, `/download`) et se rangent dans
  `SimToReal/Logs/DASH01_bruit_<date>/raw/`.

### 2.3 Référence sonore

Avant le test 1, enregistrer 10 s de chaque situation :
- robot **LIMP** (ventilateurs, alimentation) ;
- **SET_POS** debout, immobile : c'est le niveau « sans sifflement » auquel on compare.

---

## 3. Sécurité

- **Harnais en place pour tous les tests**, y compris les tests debout. Les tests 2 à 4 doivent
  être faits en harnais.
- **Revenir de l'option MIT exige de passer par LIMP** : le daemon refuse le retour direct en
  SET_POS. LIMP lâche les moteurs : **le robot doit être tenu par le harnais au moment du LIMP**.
- **À l'engagement du MIT, la jambe s'affaisse** : la consigne reste la pose tenue, donc le couple
  part de zéro, et la jambe descend jusqu'à ce que kp × erreur porte le poids. À kp 200 avec ~20 N·m
  sur la came, cela fait **~6°**, pour une marge workspace de ~7° au genou. Voir 4.2 pour l'éviter.
- Garde à portée de main : bouton E-STOP du panneau. Arrêter tout test qui fait chauffer un moteur
  au-delà de 60 °C (panneau thermique).
- Toujours commencer par les gains par défaut du MIT hold (kp 100, kd 2), puis monter.

---

## 4. Les tests

### 4.0 Préparation (une fois)

1. Déployer la version courante, calibrer, robot debout dans le harnais.
2. Panneau MANUAL : vérifier que l'option **« MIT hold (stiffness / damping) »** est présente et
   **décochée**.
3. Faire les références sonores (2.3) et poser une marque `"REF SET_POS debout"`.

### 4.1 Test 1 : robot suspendu, gains seuls

But : la came siffle-t-elle sans pied au sol et sans politique ? Si oui, H1.

1. Soulever le robot dans le harnais : **pieds libres**, à plusieurs centimètres du sol.
2. MANUAL, Home (pose debout), en SET_POS. Attendre que la jambe soit immobile.
3. Régler les gains du MIT hold, rôles **cam** et **thigh** à **kp 100, kd 2**, puis cocher MIT hold.
   Écouter 10 s, marque `"T1 kp100 kd2 suspendu"`.
4. Passer cam/thigh à **kp 200, kd 5** (les gains de la politique). 10 s immobile, puis
   **2–3 petits mouvements lents** de la came au curseur (±2°), parce que le bruit augmente avec la
   vitesse. Marque `"T1 kp200 kd5 suspendu"`.
5. Même chose à **kp 200, kd 2**, puis **kp 200, kd 1**, puis **kp 100, kd 5**. Une marque par
   étape.
6. Tenir le robot, LIMP, décocher MIT hold.

À noter : à quel couple (kp, kd) le sifflement apparaît, et sa fréquence.

### 4.2 Test 2 : debout, poids seul, sans lutte ajoutée

But : la chaîne fermée par le pied fait-elle siffler aux gains de la politique, sans aucune lutte
ajoutée ?

La pose de référence est la **stance d'entraînement**, celle de la trame de handover. Dans les runs 3
et 4, sa consigne était déjà en avance sur la pose mesurée : **came +3.9°, cuisse −1.5°**. C'est
l'écart dont la simulation a besoin pour porter le poids à kp 200. Le test 2 reproduit cette trame
de handover, tenue sans politique.

**Choix à faire avant le test**, à cause de l'affaissement à l'engagement (voir 3) :

- **Option A, sans code : alléger d'abord le robot avec le harnais.**
  1. Robot debout, harnais tendu pour qu'il porte **la plus grande partie du poids** (pieds
     posés, peu chargés).
  2. MANUAL, Home en SET_POS. MIT hold à **kp 200, kd 5**, cocher. L'affaissement reste petit,
     puisque la charge est faible.
  3. Appliquer les trims d'articulation **came +3.9°, cuisse −1.5°** sur les deux jambes
     (`/api/balance/joint_trim` ou le panneau). Pendant un maintien, un trim déplace la consigne
     lentement, à la vitesse de Home. **Vérifier d'abord sur le panneau que `cmd_norm` de la came
     augmente**, c'est-à-dire que le signe est le bon.
  4. Détendre lentement le harnais jusqu'à ce que le robot porte son poids. Pose attendue : proche
     de la pose debout, à ~1° près.
  5. 20 s immobile. Marque `"T2 stance kp200 kd5 debout"`.
- **Option B, petit ajout de code** : un mode « maintien MIT à la stance de la politique », qui
  envoie en continu la trame de handover (`q_des` = stance, `drive_kp` / `drive_kd` du bundle), avec
  un décalage par articulation réglable. C'est une politique dont l'action est nulle : même
  armement, même handover, mêmes gardes. Les tests 2 à 4 deviennent propres et reproductibles, mais
  il faut écrire, tester et déployer ce mode.

**Recommandation** : commencer par l'option A. Si l'affaissement ou le réglage au harnais rendent le
test peu fiable, faire l'option B avant les tests 3 et 4.

Rester ensuite en MIT pour le test 3, sans passer par LIMP.

### 4.3 Test 3 : debout, lutte imposée (comme la politique)

But : reproduire la lutte que la politique crée. Si seul ce test siffle, H2.

1. Partir de l'état final du test 2 (MIT kp 200 / kd 5, stance, robot debout).
2. Ajouter la lutte **par paliers**, sur les deux jambes à la fois, avec les trims :

   | palier | trim came | trim cuisse | soit, par rapport à la pose mesurée |
   |---|---|---|---|
   | 3a | +3.9 + 1.5 = **+5.4°** | −1.5 − 1.5 = **−3.0°** | came +5°, cuisse −3° |
   | 3b | +3.9 + 3 = **+6.9°** | −1.5 − 3 = **−4.5°** | ≈ ce que la politique a fait (came +7°, cuisse −5°) |

3. Chaque palier : attendre que la jambe soit stable, 15 s, marque `"T3a ..."` / `"T3b ..."`.
4. Surveiller la marge workspace et l'inclinaison sur le panneau. Si le robot bascule de plus de
   5°, revenir au palier précédent (les trims se règlent en direct).

Remarque : came et cuisse étant couplées par le pied, le robot ne prendra pas exactement ces angles.
Une partie de l'écart restera en effort interne, et c'est précisément ce qu'on veut produire.
L'analyse le vérifiera avec les puissances.

### 4.4 Test 4 : la même lutte, kd 2

But : baisser kd suffit-il, même avec la lutte ?

1. Rester au palier 3b.
2. Gains cam/thigh : **kp 200, kd 2**, 15 s, marque `"T4 kp200 kd2 lutte"`.
3. Puis **kp 200, kd 1**, 15 s, marque `"T4 kp200 kd1 lutte"`.
4. Optionnel : kd 5 et **kp 100**, pour voir si kp joue aussi.
5. Fin : retirer les trims d'un coup (retour à 0, à la vitesse de Home), tenir le robot avec le
   harnais, LIMP, décocher MIT hold, **remettre les gains MIT hold par défaut** (kp 100, kd 2). Ils
   sont enregistrés sur le robot et survivent au redémarrage.

---

## 5. Lecture des résultats

| T1 suspendu kd 5 | T2 debout stance | T3 lutte | T4 lutte kd 2 | Conclusion | Piste |
|---|---|---|---|---|---|
| siffle | siffle | siffle | se tait | **H1** : les gains | baisser kd au déploiement, réentraîner avec un kd réaliste, voir si le firmware filtre la vitesse |
| silence | siffle | siffle | — | gains + **raideur de la chaîne fermée** | modéliser la souplesse du pied (semelles TPU) et le jeu dans la simulation, baisser kd |
| silence | silence | siffle | se tait | **H3** : la lutte excite des gains trop élevés | baisser kd, et pénaliser la lutte à l'entraînement |
| silence | silence | siffle | siffle | **H2** : la lutte elle-même | pénaliser à l'entraînement les couples qui se contrarient dans la jambe, revoir la stance |
| silence partout | | | | le bruit vient de la **dynamique de la politique** (consignes à 76 Hz, à-coups) | refaire après le plan C (100 Hz tenus), interpoler les consignes |

Le cinquième cas compte : si aucun test ne siffle, c'est que la politique fait quelque chose que les
consignes fixes ne reproduisent pas.

## 6. Analyse après les tests

Pour chaque dump, je referai les mesures de `Logs/ANALYSE_bruit_moteurs.md` :
- ondulation du courant des cames et des cuisses (rms autour de la moyenne glissante) ;
- bruit de la vitesse renvoyée ;
- puissances came/cuisse de signe opposé (part de lutte) ;
- corrélation entre la lutte et le bruit.

Puis je les comparerai avec le son noté (présence, fréquence) et avec les runs 3 et 4.

À ranger dans `SimToReal/Logs/DASH01_bruit_<date>/` :
- `raw/` : dumps et événements ;
- `audio/` : un fichier par test ;
- `notes.md` : tableau test / gains / sifflement / fréquence / remarques.

## 7. À faire en parallèle

- Continuer à **sauvegarder le fichier de chaque run de politique** (`policyrun_*.csv` / `.npz`,
  colonnes kp6 / kd6 par tick), comme au run 5. Le run 5 montre que la politique colle le kd des cames
  à 5 et fait sauter leur kp de 40–55 N·m/rad par tick (voir `Logs/ANALYSE_bruit_moteurs.md` §3 bis).
  Si les tests désignent kd, ces deux comportements se corrigent à l'entraînement (plage de kd,
  pénalité sur les variations de kp).
