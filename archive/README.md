# archive/ — frozen copies, nothing active

Nothing here is read by the code. The active files stay where they are.

- `models/standing/`: frozen copy of the three STAND-03/04 plants (`stand_v1.xml`, `stand_v2.xml`, `stand_xshape.xml`), with their rest references (`quiet_*.json`) and their generator (`build_three_standing.py`), copied from `BalanceRL/model/` when DASH-01@3574ddd was imported. Used to reproduce the comparison of the three plants. The originals stay in `BalanceRL/model/`, because `BalanceRL/config.py` builds its `stand_*` and `recover_*` presets from their paths. `stand_xshape.xml` is the plant of the deployed policy `standing04_xshape_best`.
- `scripts/deploy_v8_to_pi.sh`: one-off deployment of the `95ea100..9eaa4e1` diff (twin v8), file dated 2026-11-09. It is replaced by `controller/deploy_to_pi.sh`. **Do not run it again.**

The full history (Nathann's stacks, `sprint_runner/`, homing v7, every branch) is in `DASH-01-archive`.
