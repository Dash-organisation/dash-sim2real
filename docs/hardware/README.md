# docs/hardware — battery, power and charging

How DASH-01 is powered, what voltage the motor drives accept, and how to charge the pack safely.
Written on 2026-11-24, after the power cut of the Kd test
([`../sim2real/DASH01_journal_2026-11-24.md`](../sim2real/DASH01_journal_2026-11-24.md), §4.1).

## TL;DR

- **Never put more than 52 V on the bus.** The CubeMars AK drives accept 18–52 V.
- **Charge the 13S pack to 50–51 V, not to "full".** A full 13S pack (54.6 V) is above what the
  drives accept.
- **Limit the current first (2–3 A), then set the voltage (50.5 V).** Stop when the current drops
  below about 0.3 A.
- **Charge with the robot off** (or at least the motors idle), and **unplug the supply before any
  test**. The robot runs on the battery alone.

## 1. The battery pack

Source: Nathann Morand's thesis, §3.2.3, Table 3.4 (`Dash-01/Doc/Master_Thesis_Nathann_Morand_Dash-01-1.pdf`).

| | |
|---|---|
| Cells | EVE INR21700-50PL, 5.0 Ah (Li-ion, 4.2 V max per cell) |
| Configuration | **13S2P** |
| Nominal | 48.1 V, 10 Ah (468 Wh) |
| Full charge | 13 × 4.2 V = **54.6 V** (do not use, see §2) |
| Continuous / peak current | 100 A (cell-limited) / 360 A for 2 s |
| Internal resistance | about 65 mΩ for the pack (datasheet < 10 mΩ per cell): about 10 V of sag at 160 A |
| BMS | DALY, configured over Bluetooth with the DALY BMS app (a CAN port exists, unused) |
| Mass | 3.5 kg with BMS and casing |

The drives report no bus voltage in our stack, so the sag under load has never been measured on the
robot.

## 2. What the motor drives accept

| | AKE90-8 KV35 (cams, thighs) | AK60-39 V3.0 KV80 (hip rolls) |
|---|---|---|
| Rated voltage (motor datasheet) | 48 V | 48 V |
| Allowable working voltage (driver board) | **18–52 V** | **18–52 V** |

- The motor datasheets only give the rated 48 V
  ([AKE90-8 KV35](https://img.cubemars.com/products/cubemars-product-parameter/AKE90-8-KV35.pdf),
  [AK60-39 V3.0](https://img.cubemars.com/products/cubemars-product-parameter/AK60-39-V3.0-datasheets.pdf)).
- The range comes from the AK driver-board specifications, for both the small and the large board:
  "Rated working voltage 48V — Allowable working voltage 18-52V"
  ([AK Series Module Product Manual V3.0.0](https://www.cubemars.com/images/file/20250207/1738921575887499.pdf),
  [Driver Manual V1.0.17](https://www.cubemars.com/images/file/20250522/1747899365958473.pdf)).
  The Driver Manual's revision log says CubeMars replaced "maximum working voltage" with this
  "allowable working voltage range".
- **To confirm on our boards**: these are the generic AK driver specs. The CubeMars tool's
  "System Settings" page shows the voltage limits actually configured on each drive. Record them
  in the table of
  [`../sim2real/DASH01_tuto_retest_CubeMars.md`](../sim2real/DASH01_tuto_retest_CubeMars.md) §1.

**Why keep a margin below 52 V.** When the motors brake, they push current back into the bus, and
its voltage rises above the battery's for a moment. With the battery connected, the battery absorbs
it. A lab supply alone cannot absorb current, so the bus voltage would climb further.

| Pack voltage | Per cell | Approx. charge | Verdict |
|---|---|---|---|
| 54.6 V | 4.20 V | 100 % | **forbidden**: above the drives' limit |
| 52.0 V | 4.00 V | ~80 % | absolute limit, no margin for regeneration |
| **50–51 V** | **3.85–3.92 V** | ~60–70 % | **recommended** |
| 48 V | 3.69 V | ~20 % | measured 2026-11-24 before charging (19 % in the BMS app) |
| 18 V | — | — | drives' lower limit; the BMS cuts much earlier |

The "approx. charge" column is a rough Li-ion curve. Trust the BMS app's figure.

## 3. Charging with the lab power supply

The supply connects to the charge input that goes **through the BMS**, never straight onto the cells.

1. **Turn the supply on and set it before connecting the battery.** Some lab supplies do not like
   a battery on their output while they are off, because current flows back into them. A series
   diode protects the supply.
2. **Set the current limit first: 2–3 A** (0.2–0.3 C for the 10 Ah pack).
3. **Then set the voltage: 50.5 V.** Never above 52 V.
4. Connect. The supply starts in **constant current (CC)**: its output sits at about the battery
   voltage and rises slowly as the battery charges. A supply set to 50.5 V does **not** put
   50.5 V on the bus at the start. At the end it switches to **constant voltage (CV)**, the
   current tapers, and the bus is then at the set voltage.
5. **Stop when the current falls below about 0.3 A.** Disconnect the battery, then turn the supply
   off.
6. Stay nearby. Do not leave the pack charging unattended.

In the BMS app, during and after the charge:
- every cell must stay below 4.0 V (for 52 V), and the spread between cells should stay small
  (< 0.05 V). If one cell climbs faster than the others, lower the voltage and let the BMS balance;
- **suggested hard ceiling**: set the BMS cell over-voltage protection to **4.0 V**. The BMS then
  stops the charge before the bus reaches 52 V, even if the supply is set wrong. This is the
  operator's call: record the old value before changing it.

If the BMS opens (cell over-voltage), the bus sees the supply alone, at its full set voltage. That is
one more reason never to set it above 52 V.

## 4. Why not test while charging

- A charger or a lab supply is not designed to feed pulsed motor loads. Current peaks, and the
  current sent back by the motors when braking, make the voltage swing and can trip the BMS
  (over-voltage or over-current). The whole robot then switches off at once.
- It also pollutes the measurements: supply noise on the bus while we measure current ripple.
- On 2026-11-24 the first frozen run (A) cut the power to the whole robot after 0.87 s. It is not
  known whether the robot was charging at the time. The next runs, on battery alone, did not cut.

## 5. Open points

- The drives' configured voltage limits (CubeMars tool, "System Settings").
- The DALY BMS protection thresholds (cell over-voltage and under-voltage, over-current).
- Whether the robot was charging during run A on 2026-11-24.
- The power and connectors of the left-leg drives 104 and 106, which went silent together for
  120 ms on 2026-11-20 (`../sim2real/DASH01_journal_2026-11-20.md` §3.10).
