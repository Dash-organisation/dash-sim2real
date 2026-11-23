import os, time, collections
CLK = os.sysconf("SC_CLK_TCK")
def cpustat():
    out = {}
    for l in open("/proc/stat"):
        if l.startswith("cpu") and l[3].isdigit():
            f = l.split(); v = list(map(int, f[1:]))
            out[f[0]] = (sum(v), v[3] + v[4], v[5] + v[6])   # total, idle+iowait, irq+softirq
    return out
def tasks():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit(): continue
        try:
            for tid in os.listdir(f"/proc/{pid}/task"):
                s = open(f"/proc/{pid}/task/{tid}/stat").read()
                comm = s[s.index("(")+1:s.rindex(")")]
                r = s[s.rindex(")")+2:].split()
                out[(pid, tid)] = (comm, int(r[11]) + int(r[12]), int(r[36]))
        except OSError: pass
    return out
def irqs():
    out = {}
    lines = open("/proc/interrupts").read().splitlines()
    for l in lines[1:]:
        f = l.split()
        if len(f) > 5 and f[0].endswith(":"):
            try: out[" ".join(f[5:])[:40] or f[0]] = list(map(int, f[1:5]))
            except ValueError: pass
    return out
a, ta, ia, t0 = cpustat(), tasks(), irqs(), time.time()
time.sleep(10)
b, tb, ib, dt = cpustat(), tasks(), irqs(), time.time() - t0
print("== per core: busy % (of which irq/softirq %)")
for c in sorted(a):
    tot = b[c][0] - a[c][0]; idle = b[c][1] - a[c][1]; irq = b[c][2] - a[c][2]
    print(f"  {c}: busy {100*(tot-idle)/tot:5.1f} %   irq+softirq {100*irq/tot:4.1f} %")
names = {}
for pid in set(p for p, _ in tb):
    try: names[pid] = open(f"/proc/{pid}/cmdline").read().replace("\0", " ")[:60]
    except OSError: names[pid] = "?"
rows = []
for k, (comm, t, cpu) in tb.items():
    if k in ta:
        d = 100.0 * (t - ta[k][1]) / CLK / dt
        if d >= 0.3: rows.append((d, k, comm, cpu))
print("== threads >= 0.3 % of one core (last core it ran on)")
for d, (pid, tid), comm, cpu in sorted(rows, reverse=True):
    print(f"  {d:5.1f} %  core {cpu}  pid {pid:>6} tid {tid:>6}  {comm:<16} {names.get(pid,'')}")
print("== interrupts per second, per core (top)")
rr = []
for k in ib:
    if k in ia:
        d = [(x - y) / dt for x, y in zip(ib[k], ia[k])]
        if sum(d) > 20: rr.append((sum(d), k, d))
for s, k, d in sorted(rr, reverse=True)[:10]:
    print(f"  {s:7.0f}/s  " + "  ".join(f"c{i}:{v:6.0f}" for i, v in enumerate(d)) + f"   {k}")
