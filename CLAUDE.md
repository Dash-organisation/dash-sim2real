# CLAUDE.md — dash-sim2real

DASH-01: a 6-DoF biped (Raspberry Pi 3B, CAN motors, 100 Hz loop). This repo holds the robot stack, the RL training, the robot model and the reports' light files. Human docs: `README.md`. Write code, docs and commit messages in English.

## Layout (phase 1 names, do not rename)

- `controller/` — robot stack (= `~/running_robot/robot/` on the Pi). Owner: Tim.
- `BalanceRL/` (standing: STAND, QUIET) and `RLframework/` (locomotion) — training. Owner: Léo-Paul.
- `report/` — Léo-Paul's report workspace (md, html galleries, figures, small data). Its scripts resolve paths from the repo root, like the old DASH-01 repo.
- `docs/experiments/` — run records; `docs/rl/EXPERIMENTS.md` — one entry per series; `docs/rl/PROJECT_CONTEXT.md` — RL handoff notes.
- `docs/sim2real/` — Tim's journals (French; English summaries in its README).
- `dash-01CAD/` — current robot model; `checkpoints/` — warm-start `best.msgpack`; `archive/` — frozen, never edit.

## Workflow

- Work on one branch: `train/<topic>` (training), `hw/<topic>` (hardware), `exp/<topic>` (throwaway). Never commit to `main` directly.
- Commit messages: `area: what changes` (e.g. `BalanceRL: add quiet-pose curriculum`).
- **Files over 1 MiB never go in this repo** (pre-commit hook; only `controller/deploy/bundles/*.npz` and `checkpoints/**/best.msgpack` are allowed). Videos, `progress.jsonl`, archives and traces stay untracked; `tools/sync_reports.sh` copies them to the `dash-reports` repo. Do not use `--no-verify` to get around it.
- `runs/`, `results/`, `logs/` are training outputs: ignored, never commit them.
- New series: add a row and an entry to `docs/rl/EXPERIMENTS.md`, records in `docs/experiments/<series>/`.
- Other repos are read-only references, cloned next to this one: `../dash-reports` (videos, heavy data, old reports), `../dash-cad` (CAD, later).

## Tests

```bash
python -m pytest BalanceRL/tests RLframework/tests          # training
python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests   # robot stack, offline (mock CAN)
```
`controller/` has 7–9 known failing tests (some flaky in `test_policy_run`, `test_pose_anchor`); only a new failure matters.

## Robot safety (always)

- Never run anything that can move the motors: `policy_cli.py pose|run`, `run_policy.py` without `--mock`, the web UI API (`curl …:8081/api/…`), or `deploy_to_pi.sh` without `--check`. Ask the user and let the operator do it.
- `ssh`/`scp` to the Pi (`nemo@192.168.4.1`) only when the user asks.
- Touching `controller/` means the full checklist in `README.md` §8 before merging into `main`; tell Tim.

## Saving tokens

- Do not open large or generated files: `report/data/**` (jsonl, csv, traces), `*.npz`, `*.msgpack`, `docs/experiments/**/*.json` manifests, `graphify-out/graph.json`. Use `head`, `jq` or a short Python one-liner on what you need.
- Do not read `docs/sim2real/` journals whole: use `graphify section "<heading>"`.

## graphify

The knowledge graph lives in `graphify-out/` (built locally, not committed; `tools/install-hooks.sh` builds it and the git hooks refresh it after each commit/checkout/merge).

- When `graphify-out/graph.json` exists, run `graphify query "<question>"` before your first grep, search or source-file read for a codebase question, and put this rule in every subagent prompt that explores code. Read raw files only for lines the graph or an earlier read already located. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. When the answer is under a Markdown heading, `graphify section "<heading>"` prints that section's text, so the .md need not be opened.
- Read `graphify-out/GRAPH_REPORT.md` only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` (AST only, no API cost) if the git hooks are not installed.
- If `graphify` is not installed, skip these rules and say once that `uv tool install graphifyy` would enable them.
