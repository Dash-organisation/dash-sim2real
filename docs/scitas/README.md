# Training on SCITAS (EPFL): Izar from a Mac

How to connect to the EPFL **Izar** GPU cluster, set up the Python environment, and run the
DASH-01 trainings (`BalanceRL/`, `RLframework/`) there. Based on the official SCITAS
documentation (<https://scitas-doc.epfl.ch>); check it when something here looks outdated.

Throughout this guide, replace `<gaspar>` with your own EPFL (GASPAR) username.

---

## Contents

0. [Quick use: the daily loop](#0-quick-use-the-daily-loop) ← **start here if your machine is already set up**
1. [Before you start](#1-before-you-start)
2. [Setting up a new Mac](#2-setting-up-a-new-mac)
3. [SSH keys: what they are and how to handle them](#3-ssh-keys-what-they-are-and-how-to-handle-them)
4. [Python environment for this repo](#4-python-environment-for-this-repo)
5. [Running a DASH-01 training](#5-running-a-dash-01-training)
6. [Minimal example from scratch](#6-minimal-example-from-scratch)
7. [Command cheat sheet](#7-command-cheat-sheet)
8. [Official links](#8-official-links)

---

## 0. Quick use: the daily loop

> Read this first **if your machine is already set up** (public key on the cluster,
> `~/.ssh/config` in place, venv built). Otherwise, do §2 and §4 once, then come back here.

### 0.1 The mental model

```
Mac (edit, git)  --git push-->  GitHub  --git pull-->  frontend izar1  --sbatch-->  Slurm queue
                                                       (no GPU)                        |
                                                                                       v
                                                                  compute node i0x (V100 32 GB)
```

One rule everything follows from: **never compute on the frontend, only submit from it**.
The frontend is for editing, transferring files and running `sbatch` (§1).

### 0.2 The daily loop

```bash
# --- on the Mac (EPFL network or VPN on, otherwise ssh times out) ---
git push                                   # your train/<topic> branch
ssh izar

# --- on the frontend ---
cd ~/dash-sim2real
git pull
SEEDS=0 LINKS=1 BalanceRL/slurm/izar_balance.sh   # → "bal_s0 link 1 = 3167xxx"
Squeue                                     # ST: PD = pending, R = running
tail -f BalanceRL/logs/bal_s0-3167xxx.out  # follow the output live (Ctrl+C to quit)

# --- back on the Mac ---
exit
rsync -avz izar:~/dash-sim2real/BalanceRL/runs/bal_s0/ BalanceRL/runs/bal_s0/   # fetch results
```

That is 90 % of the usage.

### 0.3 Project phases

**Phase A: once per project (about 30 min)**

1. **Decide where files live** (§7):
   - `/home/<gaspar>`: code, scripts, small results. Backed up, with a quota.
   - `/scratch/...`: compute data and large outputs. Fast, no quota,
     **files older than 30 days are deleted without notice**.
2. **Build the Python environment in an interactive session**, never on the frontend (§4).
3. **Validate with a short job in the `debug` QOS** (1 h max, 2 GPUs max, high priority):
   a few minutes that prove the environment loads and the GPU is visible, before any real run.

**Phase B: every iteration (the loop in §0.2)**

- Edit and commit **on the Mac**. The cluster copy is disposable: it is rebuilt with a
  `git pull`. Do not edit on both sides.
- `git pull` → `sbatch` → `Squeue` → read the `.out` → `rsync` back.
- `scancel <jobid>` if a job goes wrong; `Sjob <jobid>` to inspect a finished job.

**Phase C: production**

- Use `--qos=gpu` (3 days, the default) or `--qos=long` (7 days) (table in §5.4).
- Keep `--time` **tight but realistic**: short jobs get scheduled first. Oversizing `--time`
  costs you queue time. The repo's launchers chain 4.5 h links that resume from the last
  checkpoint (`--resume auto`) instead of asking for one very long job.
- One job = one configuration. The job id (`%j`) in the output file name keeps earlier runs.
- Run `sausage` from time to time to see your monthly usage (§5.6).

**Phase D: wrap-up**

- Fetch or move to `/home` everything that matters **before** the 30-day `/scratch` purge.
- Anything that must outlive the project goes to your Mac or to git (code, configs,
  `docs/experiments/`), or to `dash-reports` for heavy files. Not on the cluster.

### 0.4 Pitfalls that cost an hour

| Symptom | Real cause | See |
|---|---|---|
| `ssh` hangs, then times out | Neither EPFL network nor VPN. It is **not** a key problem. | §2.1 |
| The job runs but JAX only sees `CpuDevice` | `--gres=gpu:N` missing (mandatory on Izar), or `jax` installed without `[cuda12]`. | §4, §5.4 |
| Everything is slow, `nvidia-smi` shows nothing useful | Computation started on the frontend instead of being submitted. | §1 |
| The job stays `PD` for a long time | `--time` or resources oversized. Do not cancel and resubmit in a loop. | §5.5 |
| First connection refused | Typed `y` instead of `yes` in full. | §2.4 |
| Files disappeared | `/scratch`: purged after 30 days, without notice. | §7 |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | Do not force it. Have the fingerprint confirmed by support. | §3.3 |

---

## 1. Before you start

**Prerequisites (from the SCITAS docs)**

- A valid EPFL GASPAR account.
- An approved SCITAS account (request it through SCITAS or your lab / course).
- Being **on the EPFL network** or connected to the **EPFL VPN** (<https://network.epfl.ch/vpn>).

**Which cluster for whom?**

| Status | Clusters |
|---|---|
| Student (bachelor / master) | **Izar** (academic GPU) and **Jed** via `--partition=academic` (CPU) |
| Lab member (PhD, post-doc) | Jed (CPU) and Kuma (research GPU, pay-per-use) |

**Two key notions**

- **Frontend (login node)**: `izar1`, the machine you land on after `ssh`. It is for editing
  files, light compiling and **submitting** jobs. It has **no usable GPU**.
- **Compute nodes**: `i01`, `i08`… You only reach them through Slurm (`sbatch`, `Sinteract`).
  That is where the GPUs are (Tesla V100 32 GB, 2 per node on the `gpu` partition).

**`/home` is shared** between SCITAS clusters: a public key added once works everywhere.

---

## 2. Setting up a new Mac

Two strategies:

- **Option A (recommended)**: generate a **new key pair** on the new Mac. One key per machine
  is cleaner: if you lose a computer, you revoke only its key.
- **Option B**: copy your existing key from the old Mac. Possible but less safe. See §2.8.

### 2.1 Connect to the EPFL network or VPN

Without it, `ssh` will hang (timeout).

### 2.2 Check whether keys already exist

```bash
ls -la ~/.ssh
```

- If you see `id_ed25519` and `id_ed25519.pub`: **do not regenerate**, you would overwrite
  them. Go to step 2.4.
- If the folder does not exist or only contains `known_hosts`: continue.

### 2.3 Generate the key pair

Official SCITAS command:

```bash
ssh-keygen -t ed25519 -C "${USER}@clusters" -f ${HOME}/.ssh/id_ed25519
```

- `-t ed25519`: modern key type (elliptic curves), short and strong.
- `-C "..."`: a comment to recognise the key, e.g. `-C "<gaspar>@my-macbook"`. Without `-C`,
  the default comment is `user@<network hostname>`, which on EPFL Wi-Fi is not meaningful.
- `-f`: file path.
- **Passphrase**: the docs recommend one. With the macOS keychain (step 2.6) you only type it once.

Then secure the permissions:

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
```

### 2.4 Send the public key to the cluster

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub <gaspar>@izar.hpc.epfl.ch
```

- Type your **GASPAR password** (the last time you will need it).
- On the very first connection, SSH shows the server fingerprint and asks `yes/no`: answer
  **`yes` in full** (`y` is refused). See §3.3 to check the fingerprint.

If `ssh-copy-id` is not available, the manual equivalent:

```bash
cat ~/.ssh/id_ed25519.pub | ssh <gaspar>@izar.hpc.epfl.ch \
  "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

### 2.5 Test

```bash
ssh <gaspar>@izar.hpc.epfl.ch
```

No more GASPAR password (possibly the key passphrase) → it works. `exit` to leave.

### 2.6 (Comfort) `~/.ssh/config` + macOS keychain

```bash
nano ~/.ssh/config
```

Content:

```sshconfig
Host *
    AddKeysToAgent yes
    UseKeychain yes
    IdentityFile ~/.ssh/id_ed25519

Host izar
    HostName izar.hpc.epfl.ch
    User <gaspar>
    ForwardAgent yes

Host jed
    HostName jed.hpc.epfl.ch
    User <gaspar>
```

Then:

```bash
chmod 600 ~/.ssh/config
ssh-add --apple-use-keychain ~/.ssh/id_ed25519   # (older macOS: ssh-add -K)
```

From now on:

```bash
ssh izar
```

- `UseKeychain` / `AddKeysToAgent`: macOS stores the passphrase in the Keychain.
- `ForwardAgent yes`: according to the SCITAS docs, on Izar without agent forwarding a password
  is asked again when you `ssh` from the frontend to a compute node. Command-line equivalent:
  `ssh -A <gaspar>@izar.hpc.epfl.ch`.

### 2.7 File transfer (optional)

- Command line: `scp` / `rsync` (examples in §5 and §6).
- GUI: **FileZilla** (recommended by SCITAS), protocol **SFTP**, host `izar.hpc.epfl.ch`,
  port 22, your GASPAR user, authentication type "key file" → `~/.ssh/id_ed25519`.

### 2.8 Option B: reuse your existing key

On the old Mac, copy `~/.ssh/id_ed25519` and `~/.ssh/id_ed25519.pub` to the new one through a
safe channel (direct AirDrop, a USB stick you wipe afterwards). **Never** by email, chat or
shared cloud, and **never** onto the cluster. Then on the new Mac:

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
# put both files in ~/.ssh, then:
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
```

No `ssh-copy-id` needed: the public key is already on the cluster.

### 2.9 Revoke the key of a lost or sold computer

On the cluster:

```bash
nano ~/.ssh/authorized_keys
```

Delete the matching line (recognisable by the comment at the end of the line) and save.

---

## 3. SSH keys: what they are and how to handle them

### 3.1 On your Mac (`~/.ssh/`)

| File | Role | Secret? |
|---|---|---|
| `id_ed25519` | **Private key**. Proves your identity. | **YES, absolutely.** Never leaves your Mac. |
| `id_ed25519.pub` | **Public key**. Added to the cluster. | No, shareable. |
| `known_hosts` | Fingerprints of servers already visited (including Izar). Protects against server spoofing. | No |
| `known_hosts.old` | Automatic backup of the previous `known_hosts`. | No |

Useful commands:

```bash
ssh-keygen -lf ~/.ssh/id_ed25519.pub      # show the fingerprint
cat ~/.ssh/id_ed25519.pub                 # show the public key
ssh-keygen -y -f ~/.ssh/id_ed25519        # asks for the passphrase if there is one
ssh-keygen -p -f ~/.ssh/id_ed25519        # add / change the passphrase
ssh-keygen -c -C "<new comment>" -f ~/.ssh/id_ed25519   # change the comment
```

> If you pressed Enter at the passphrase prompts, your key is **not protected**: anyone with
> access to your Mac session can connect to the cluster. Add one with `ssh-keygen -p`.

### 3.2 On the cluster (`~/.ssh/`)

| File | Role |
|---|---|
| `authorized_keys` | Public keys allowed to log into your account. `ssh-copy-id` adds one line per key. |

Since `/home` is shared, this authorisation applies to every SCITAS cluster you have access to.

**SCITAS golden rule**: **never put a private key** in your cluster home. After a security
incident in 2026, SCITAS disabled the private keys it found stored in user homes; the people
concerned had to log in again with GASPAR and add a clean public key (see the SCITAS blog).

### 3.3 The Izar server fingerprint (`known_hosts`)

On the first connection SSH prints the server's `ED25519 key fingerprint is SHA256:...`.
Compare it with the one published on the Izar page of the SCITAS docs. The frontend host keys
were changed after the 2026 incident, and the docs page may lag behind: if the two differ,
**have the current fingerprint confirmed** by support (1234@epfl.ch, subject "HPC") before
answering `yes`.

If you ever see `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!`:

1. Do not force it. Check the fingerprint with SCITAS.
2. If it is legitimate, remove the old entry and reconnect:

```bash
ssh-keygen -R izar.hpc.epfl.ch
```

---

## 4. Python environment for this repo

The Slurm scripts in this repo activate `${VENV:-$HOME/venvs/dash-v2}` (override with
`VENV=...`), and put `~/.local/bin` on the `PATH`. The training stack is JAX + MuJoCo MJX
(`RLframework/requirements.txt`, Python ≥ 3.12). On the cluster, install **`jax[cuda12]`**
instead of plain `jax`: the plain wheel only runs on the CPU.

Build it **once**, in an interactive session on a GPU node (never on the frontend):

```bash
# on the frontend
cd ~ && git clone https://github.com/Dash-organisation/dash-sim2real.git
Sinteract -q debug -t 00:30:00 -m 16G -c 4 -g gpu:1

# now on a compute node
module purge
module load gcc python           # check the version: python3 --version (needs >= 3.12)
python3 -m venv ~/venvs/dash-v2
source ~/venvs/dash-v2/bin/activate
pip install --upgrade pip
pip install -r ~/dash-sim2real/RLframework/requirements.txt
pip install "jax[cuda12]==$(pip show jax | awk '/^Version/{print $2}')"   # same version, CUDA wheels
python -c "import jax; print(jax.devices())"   # must list CudaDevice entries
exit
```

If the `python` module is older than 3.12, use `uv` instead (it installs into `~/.local/bin`,
which the Slurm scripts already put on the `PATH`):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python 3.12 ~/venvs/dash-v2
source ~/venvs/dash-v2/bin/activate
uv pip install -r ~/dash-sim2real/RLframework/requirements.txt "jax[cuda12]"
```

Notes:

- `module spider python` lists the available Python versions.
- If the repo is private, clone it with an HTTPS personal access token, or `rsync` it from
  your Mac (`rsync -avz --exclude .git ./ izar:~/dash-sim2real/`). Do **not** copy a private
  SSH key onto the cluster for GitHub (§3.2).

---

## 5. Running a DASH-01 training

### 5.1 What is in the repo

| Script | What it does |
|---|---|
| `BalanceRL/slurm/izar_balance.sh` | Standing / balance campaign: `SEEDS × LINKS` chained 4.5 h jobs on 2 × V100. Each link resumes the previous one. |
| `BalanceRL/slurm/izar_balance.sbatch` | The job itself. Launch it only through `izar_balance.sh` (it sets `REPO`, the chain and the seeds). |
| `BalanceRL/slurm/izar_*.sbatch` | Other standing series (quiet tracking, recovery, three-plant comparison). |
| `RLframework/slurm/izar_dash.sh` | Locomotion curriculum, three seeds on 2 GPUs each. |
| `RLframework/slurm/izar_train.sbatch` | Generic locomotion job (`PRESET`, `NAME`, `SEED`, `WARM`, …, passed with `--export`). |

The launchers locate the repo from their own path, so they work from wherever you cloned it.
Logs go to `BalanceRL/logs/` or `RLframework/logs/` (`<job-name>-<jobid>.out`); runs go to
`BalanceRL/runs/<name>/` or `RLframework/results/`.

### 5.2 First run: a short check

```bash
ssh izar
cd ~/dash-sim2real && git switch train/<topic> && git pull
SEEDS=0 LINKS=1 BalanceRL/slurm/izar_balance.sh    # one link of one seed, to watch it start
Squeue
tail -f BalanceRL/logs/bal_s0-<jobid>.out
```

The first lines of the log must show `nvidia-smi` with V100s and `jax ... [CudaDevice(id=0), ...]`.
If JAX lists only CPU devices, stop the job (`scancel <jobid>`) and fix the venv (§4).

### 5.3 Full campaign

```bash
BalanceRL/slurm/izar_balance.sh                   # seeds 0 1 2, 4 links of 4.5 h each
PREFIX=bal6 SEEDS="0 1" BalanceRL/slurm/izar_balance.sh
RLframework/slurm/izar_dash.sh                    # locomotion, three seeds
SEEDS=0 PRESET=dash_planar RLframework/slurm/izar_dash.sh
```

Then evaluate, export and check the bundle on your Mac, as described in
[`BalanceRL/README.md`](../../BalanceRL/README.md) ("Workflow") and section 7 of the
[main README](../../README.md).

### 5.4 Resources, partitions and QOS on Izar

Every job **must** request GPUs with `--gres=gpu:N`; without it the job runs on CPU only.

| QOS | Use | Time limit |
|---|---|---|
| `gpu` | Default, normal jobs (up to 8 nodes) | 3 days |
| `long` | Long jobs (up to 8 nodes) | 7 days |
| `debug` | Tests, max 2 GPUs, high priority, **not for production** | 1 hour |
| `build` | Compiling, 20 cores, 0 GPU | 4 hours |

Partitions: `gpu` (default), `gpu-xl` (nodes with 4 GPUs), `test`.

Sizing rule used in this repo: one Izar node = 2 V100 + 40 cores. With **1 GPU**, ask for about
half the node's cores (`--cpus-per-task=10` to `20`); asking for more strands the node's other
GPU, and SCITAS asked users not to do that. With **2 GPUs**, you can take the whole node.

### 5.5 Following and managing jobs

```bash
Squeue                     # your jobs (capital S = SCITAS wrapper)
scancel <jobid>            # cancel one job
scancel -n bal_s0          # cancel every job with that name (e.g. a whole chain)
Sjob <jobid>               # details of a finished job (time, memory, exit code)
```

Column `ST`: `PD` = pending, `R` = running. Waiting is normal: do not cancel and resubmit in a
loop, it puts you back at the end of the queue.

### 5.6 Fetching results and tracking usage

```bash
# from the Mac
rsync -avz izar:~/dash-sim2real/BalanceRL/runs/bal_s0/ BalanceRL/runs/bal_s0/
rsync -avz izar:~/dash-sim2real/BalanceRL/logs/ BalanceRL/logs/

# on the cluster
sausage                        # CPU/GPU hours this month
sausage -s 2026-09-01          # since a date
sausage quotas --unit GiB      # storage quotas
```

`runs/`, `results/`, logs and videos stay untracked in git (see the main README, "What stays
untracked"). Only the exported bundle and the `docs/experiments/` record are committed.

---

## 6. Minimal example from scratch

To check a new account or a new venv without the repo: a script that prints the GPU, run in
the `debug` QOS.

```bash
# on the Mac
mkdir -p ~/demo-izar && cd ~/demo-izar
cat > check.py <<'EOF'
import socket
print("node:", socket.gethostname())
import jax, jax.numpy as jnp
print("devices:", jax.devices())
x = jnp.ones((4000, 4000))
print("matmul ok:", float((x @ x)[0, 0]))
EOF

cat > job.sh <<'EOF'
#!/bin/bash -l
#SBATCH --job-name=demo-gpu
#SBATCH --output=demo-%j.out     # %j = job id
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=00:10:00          # short = shorter wait
#SBATCH --partition=gpu
#SBATCH --qos=debug              # quick test, high priority
#SBATCH --gres=gpu:1             # MANDATORY on Izar

module purge
source ~/venvs/dash-v2/bin/activate
echo "Start: $(date) on $(hostname)"
nvidia-smi
python check.py
echo "End: $(date)"
EOF

rsync -avz ~/demo-izar/ izar:~/demo-izar/
ssh izar
cd ~/demo-izar && sbatch job.sh    # → Submitted batch job 3167xxx
Squeue
cat demo-3167xxx.out
```

Interactive alternative, to test by hand on a GPU node:

```bash
Sinteract -q debug -t 00:30:00 -m 16G -c 4 -g gpu:1    # -g is mandatory on Izar
nvidia-smi
exit                                                   # frees the node
```

Ready-to-use official examples: `git clone https://github.com/epfl-scitas/scitas-examples`.

---

## 7. Command cheat sheet

**On the Mac**

| Command | Effect |
|---|---|
| `ssh izar` / `ssh <gaspar>@izar.hpc.epfl.ch` | Connect |
| `ssh-keygen -t ed25519 -C "comment"` | Create a key |
| `ssh-copy-id -i ~/.ssh/id_ed25519.pub <gaspar>@izar.hpc.epfl.ch` | Add the public key |
| `ssh-add --apple-use-keychain ~/.ssh/id_ed25519` | Store the passphrase |
| `rsync -avz folder/ izar:~/folder/` | Send a folder |
| `rsync -avz izar:~/folder/ folder/` | Fetch a folder |
| `scp izar:~/file .` | Fetch one file |
| `ssh-keygen -R izar.hpc.epfl.ch` | Forget the server fingerprint |

**On the cluster**

| Command | Effect |
|---|---|
| `sbatch job.sh` | Submit a job |
| `Squeue` | List your jobs |
| `scancel <jobid>` | Cancel a job |
| `Sjob <jobid>` | Details of a finished job |
| `Sinteract -q debug -t 00:30:00 -g gpu:1` | Interactive shell on a GPU |
| `module avail` / `module spider X` / `module load X` | Available software |
| `sausage` / `sausage quotas --unit GiB` | Usage / quotas |

**Where to store things**

| Location | Use |
|---|---|
| `/home/<gaspar>` | Code, venvs, scripts, small important files (backed up) |
| `/scratch/...` | Temporary compute data: fast, **deleted after 30 days** |

---

## 8. Official links

- SCITAS documentation: <https://scitas-doc.epfl.ch>
- Connecting to the clusters: <https://scitas-doc.epfl.ch/user-guide/using-clusters/connecting-to-the-clusters/>
- Izar page (QOS, partitions, hardware): <https://scitas-doc.epfl.ch/supercomputers/izar/>
- Running jobs: <https://scitas-doc.epfl.ch/user-guide/using-clusters/running-jobs/>
- File systems: <https://scitas-doc.epfl.ch/user-guide/data-management/how-to-use-filesystems/>
- FAQ: <https://scitas-doc.epfl.ch/faq/>
- Blog / incidents: <https://scitas-doc.epfl.ch/blog/>
- EPFL VPN: <https://network.epfl.ch/vpn>
- Examples: <https://github.com/epfl-scitas/scitas-examples>
- FileZilla: <https://filezilla-project.org>
- Support: **1234@epfl.ch** (subject: HPC)
