import sys, copy, numpy as np
sys.path.insert(0, "/Users/tk/Desktop/EPFL/Projets/Dash-01/code/DASH-01/controller/deploy")
from bundle import Bundle
from controller_balance import PolicyControllerBalance
S="/private/tmp/claude-501/-Users-tk-Desktop-EPFL-Projets-Dash-01/68a11da8-0202-40d1-b762-16524d993a28/scratchpad/"
b=Bundle.load(S+"standing04_xshape_best.npz")
L="/Users/tk/Desktop/EPFL/Projets/Dash-01/SimToReal/Logs/DASH01_runs6-9_2026-11-20/raw/policyrun_20260822_095556_standing04_xshape_best.npz"
d=np.load(L, allow_pickle=True)["data"].astype(float)
pos,vel,tau,grav,gyro,tgt=d[:,1:7],d[:,7:13],d[:,13:19],d[:,31:34],d[:,34:37],d[:,37:43]
# 1. replay
c=PolicyControllerBalance(b); c.start(pos[0],vel[0],tau[0],grav[0],gyro[0])
err=[]
for k in range(len(d)):
    o=c.step(pos[k],vel[k],tau[k],grav[k],gyro[k]); err.append(np.abs(np.asarray(o.target)-tgt[k]).max())
print("replay: max |target - logged| over 85 ticks = %.2e rad (k=0..), first ticks %s" % (max(err), np.round(err[:4],5)))
# 2. static sensitivity: constant pitch, stance joints, zero rates
default=np.asarray(b["default_motor_pos"],float)
def lean(theta_deg, n=150):
    th=np.radians(theta_deg); g=np.array([np.sin(th),0.0,-np.cos(th)])
    c=PolicyControllerBalance(b); c.start(default,np.zeros(6),np.zeros(6),g,np.zeros(3))
    for _ in range(n): o=c.step(default,np.zeros(6),np.zeros(6),g,np.zeros(3))
    return np.asarray(o.target)-np.asarray(b["nominal_ctrl"],float)
names="hip_roll_L cam_L thigh_L hip_roll_R cam_R thigh_R".split()
print("\nconstant pitch -> target - nominal (deg) after 1.5 s   [g_x = +sin(pitch): + = nose down/forward]")
for th in (-6,-3,0,3,6):
    print(f"  pitch {th:+d} deg: " + "  ".join(f"{n} {v:+6.2f}" for n,v in zip(names,np.degrees(lean(th)))))
print("\nreflex prior (designer's stabilising direction, disabled in this bundle): forward pitch g_x>0 ->"
      " thigh_L -, thigh_R +  (lean = -(0.3 g_x)) ; at +6 deg: thigh_L %+.2f thigh_R %+.2f deg" % (np.degrees(-0.3*np.sin(np.radians(6))), np.degrees(0.3*np.sin(np.radians(6)))))
# 3. what the real run commanded vs pitch
p=np.degrees(np.arctan2(grav[:,0],-grav[:,2])); nom=np.asarray(b["nominal_ctrl"],float)
print("\nrun 9: pitch vs commanded thigh (target - nominal) and measured thigh (pos - nominal), deg")
for k in range(0,85,12):
    print(f"  pitch {p[k]:+5.1f}  cmd thigh_L {np.degrees(tgt[k,2]-nom[2]):+5.2f} thigh_R {np.degrees(tgt[k,5]-nom[5]):+5.2f}   meas thigh_L {np.degrees(pos[k,2]-nom[2]):+5.2f} thigh_R {np.degrees(pos[k,5]-nom[5]):+5.2f}  cam_L {np.degrees(tgt[k,1]-nom[1]):+5.2f} cam_R {np.degrees(tgt[k,4]-nom[4]):+5.2f}")
