# AGENTS.md — dash-sim2real

Instructions for AI coding assistants (Codex, Cursor, Copilot, ChatGPT, …). Claude Code reads `CLAUDE.md`; the rules there apply to every assistant, so read it first.

## Setup check (start of every session)

Run `git config core.hooksPath`. If it does not print `tools/hooks`, the git hooks are not installed in this clone. Tell the user in one sentence (they block files over 1 MiB and keep the graphify graph fresh) and offer to run `tools/install-hooks.sh`. If `graphify` is missing, also offer `uv tool install graphifyy` (or `pipx install graphifyy`) first. When the user asks to "set up the repo", do both. Details: `README.md`, "First-time setup".

## The essentials (full version in CLAUDE.md)

- Work on one branch (`train/<topic>`, `hw/<topic>`, `exp/<topic>`), never directly on `main`. Commit messages: `area: what changes`, in English.
- Never commit files over 1 MiB, `runs/`, `results/` or `logs/`; do not bypass the pre-commit hook with `--no-verify`.
- Never run anything that can move the robot's motors (`policy_cli.py pose|run`, `run_policy.py` without `--mock`, the web UI API, `deploy_to_pi.sh` without `--check`), and no `ssh` to the Pi unless the user asks.
- Ask before `git push`; never force-push.
