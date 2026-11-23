sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

to init CAN bus on raspberry pi

104 = abduction (polling 200Hz)
105 = cam
106 = hip


scp -r nemo@100.127.143.113:running_controller/fixed_gait* "C:\Users\Nathann\Downloads\running_robot\fixed_gait"                                      

## can_probe.py -- passive CAN / daemon probe (listens, never sends)

    cd ~/running_robot/robot && .venv/bin/python tools/can_probe.py --seconds 10 --bench --threads --json /tmp/probe.json

Per interface: frames/s, the daemon's commands (seen through socketcan loopback), tx_dropped and
txqueuelen. Per drive: status period and delay after the last command (tells a drive that emits
periodically at 200 Hz from one that replies to commands). `--bench`: per-frame read cost, python-can
vs raw AF_CAN socket. `--threads`: per-thread CPU of the running daemon. Run it before and after
each change to the CAN path.

## policy_cli.py -- run a policy from the terminal, no web UI

    ssh nemo@192.168.4.1
    cd ~/running_robot/robot
    python3 tools/policy_cli.py pose standing04_xshape_best.npz        # MANUAL + slow move to the start pose
    python3 tools/policy_cli.py run  standing04_xshape_best.npz --allow-uncalibrated-thermal
    python3 tools/policy_cli.py stop      # soft stop (--hard: gains to zero now)
    python3 tools/policy_cli.py limp      # release every motor (0 A), after a run
    python3 tools/policy_cli.py estop     # E-STOP
    python3 tools/policy_cli.py status

It sends the requests the panel's buttons send. Every safety check stays in the daemon. Runs last
at most 10 s. `run` asks for the harness confirmation, holds the dead-man at 2.5 Hz, and on the
first Ctrl+C sends a soft stop, on the second a hard stop. If the script dies, the daemon
soft-stops within 1.5 s. At the end it saves the run log and prints the timing summary.

Close the web UI tab first: an open panel makes the daemon re-read its cold state from the SD
card (35-68 ms each time) and holds the control token for 15 s after each click (run 5,
2026-11-20). Other options: `--seconds N` (N <= 10), `--no-worker`, `--hung`, `--takeover`,
`--url` (from the Mac: http://192.168.4.1:8081).
