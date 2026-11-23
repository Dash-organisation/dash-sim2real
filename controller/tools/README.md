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
