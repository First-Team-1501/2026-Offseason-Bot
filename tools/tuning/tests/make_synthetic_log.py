"""Write a synthetic WPILOG that follows the /Tuning data contract (build 2).

Contents (FPGA time base, 50 Hz motor signals written only when changed = ZOH skips):
  Shooter leader 12 + follower 11: VelocityVoltage plant with kS 0.3 V, kV 0.18 V/(rot/s), kA 0.02 V/(rot/s^2);
      setpoints 15, 20, 25, 30 rot/s; gains in Config kP 0.25, kS 0.2, kV 0.18 (FF deliberately off).
  ShooterHood 9: PositionVoltage step 8.5 -> 20 deg with first-order response.
  Heading: one activation with a 30 deg step and exponential convergence (250 Hz).
  Power: battery voltage, BrownedOut, Enabled, Mode (50 Hz).
  Interlock violation: hood held below 7.75 deg for 0.5 s while the shooter leader voltage is non-zero.
  systemTime: epoch microseconds; ShooterHood Config hoodDownThresholdDeg 7.75.
usage: py -3.14 -I make_synthetic_log.py <out.wpilog>
"""
import json
import math
import os
import random
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import wpilog  # noqa: E402

KS, KV, KA = 0.3, 0.18, 0.02
DT = 0.02
EPOCH0_US = 1792700000000000          # 2026-10-22 UTC, after any sign-off in tests
INTERLOCK = (40.0, 40.5)              # hood held down while the flywheel is powered


class ZohWriter:
    """Append only when the value changes (the robot telemetry skips unchanged values)."""

    def __init__(self, w):
        self.w = w
        self.last = {}

    def put(self, name, t, v, force=False):
        if not force and self.last.get(name) == v:
            return
        self.last[name] = v
        self.w.append(name, t, v)


def make(path, seed=1):
    rnd = random.Random(seed)
    w = wpilog.Writer(path)
    Z = ZohWriter(w)
    sig = ['Voltage', 'Velocity', 'Reference', 'StatorCurrent', 'SupplyCurrent', 'Temp']
    for s in sig:
        w.start('/Tuning/Shooter/12/' + s, 'double')
    w.start('/Tuning/Shooter/12/ControlMode', 'string')
    for s in ('Velocity', 'StatorCurrent', 'SupplyCurrent', 'Temp'):
        w.start('/Tuning/Shooter/11/' + s, 'double')
    for s in ('Voltage', 'Velocity', 'Position', 'Reference', 'StatorCurrent', 'SupplyCurrent'):
        w.start('/Tuning/ShooterHood/9/' + s, 'double')
    w.start('/Tuning/ShooterHood/9/ControlMode', 'string')
    for n in ('Shooter', 'ShooterHood'):
        w.start('/Tuning/%s/Config' % n, 'json')
        w.start('/Tuning/%s/MechanismOK' % n, 'boolean')
    for s in ('Target', 'Measured', 'Error', 'Output'):
        w.start('/Tuning/Heading/' + s, 'double')
    w.start('/Tuning/Heading/Active', 'boolean')
    w.start('/Tuning/Heading/Config', 'json')
    w.start('/Tuning/Power/BatteryVoltage', 'double')
    w.start('/Tuning/Power/BrownedOut', 'boolean')
    w.start('/Tuning/Power/Enabled', 'boolean')
    w.start('/Tuning/Power/Mode', 'string')
    w.start('/Robot/ShooterHood/Position (Deg)', 'double')
    w.start('/Robot/ShooterHood/Target Position (Deg)', 'double')
    w.start('/Robot/Shooter/Hood Interlock', 'boolean')
    w.start('systemTime', 'int64')
    shooter_cfg = {'mechanism': 'Shooter', 'loop': 'velocity', 'leaders': [12], 'followers': {'11': 'Aligned'},
                   'gains': {'kP': 0.25, 'kI': 0.0, 'kD': 0.0, 'kS': 0.2, 'kV': 0.18, 'kA': 0.0, 'kG': 0.0},
                   'sensorToMechanismRatio': 1.5, 'statorLimit': 80.0, 'statorLimitEnable': True, 'supplyLimit': 60.0,
                   'supplyLimitEnable': True, 'supplyLowerLimit': None, 'supplyLowerTime': None, 'inverted': False,
                   'neutralMode': 'Coast', 'tolerance': 1.6667}
    hood_cfg = {'mechanism': 'ShooterHood', 'loop': 'position', 'leaders': [9], 'followers': {},
                'gains': {'kP': 800.0, 'kI': 0.0, 'kD': 0.0, 'kS': 0.0, 'kV': 0.0, 'kA': 0.0, 'kG': 0.0},
                'gravityType': 'Arm_Cosine', 'sensorToMechanismRatio': 330.56, 'statorLimit': 50.0, 'statorLimitEnable': True,
                'tolerance': 0.3 / 360, 'hoodDownThresholdDeg': 7.75, 'stowAngleDeg': 8.5}
    w.append('/Tuning/Shooter/Config', 0.5, json.dumps(shooter_cfg))
    w.append('/Tuning/ShooterHood/Config', 0.5, json.dumps(hood_cfg))
    w.append('/Tuning/Heading/Config', 0.5, json.dumps({'kP': 15.0, 'kI': 0.0, 'kD': 0.6, 'maxRateRadPerSec': 9.42}))
    w.append('/Tuning/Shooter/MechanismOK', 0.5, True)
    w.append('/Tuning/ShooterHood/MechanismOK', 0.5, True)
    w.append('/Tuning/Power/Enabled', 1.0, True)
    w.append('/Tuning/Power/Mode', 1.0, 'teleop')
    w.append('/Tuning/Power/BrownedOut', 1.0, False)
    w.append('/Robot/Shooter/Hood Interlock', 1.0, False)
    # setpoint schedule (rot/s): hold each 6 s
    sched = [(1.0, 15.0), (7.0, 20.0), (13.0, 25.0), (19.0, 30.0), (25.0, 20.0), (31.0, 15.0), (37.0, 15.0)]
    T_END = 46.0
    kP, kS_c, kV_c = 0.25, 0.2, 0.18
    w_true = 0.0
    hood = 8.5
    t = 1.0
    temp = 25.0
    next_sys = 1.0
    while t < T_END:
        r = next(v for a, v in reversed(sched) if t >= a)
        down = INTERLOCK[0] <= t < INTERLOCK[1]
        # VelocityVoltage controller (Phoenix: kS*sign(r) + kV*r + kP*(r - w)), clamped to 12 V
        V = kS_c * (1 if r > 0 else 0) + kV_c * r + kP * (r - w_true)
        V = max(-12.0, min(12.0, V))
        # plant: V = kS*sign(w) + kV*w + kA*a
        a = (V - KS * (1 if w_true > 0 else 0) - KV * w_true) / KA
        w_true += a * DT
        w_meas = round(w_true + rnd.gauss(0, 0.01), 3)       # quantized: unchanged values are skipped
        v_meas = round(V + rnd.gauss(0, 0.003), 3)
        i_stator = round((V - KV * w_true) / 0.03 * 0.2, 1)
        vb = round(12.4 - 0.005 * abs(i_stator), 2)
        Z.put('/Tuning/Shooter/12/Velocity', t, w_meas)
        Z.put('/Tuning/Shooter/12/Voltage', t, v_meas)
        Z.put('/Tuning/Shooter/12/Reference', t, r)
        Z.put('/Tuning/Shooter/12/StatorCurrent', t, i_stator)
        Z.put('/Tuning/Shooter/12/SupplyCurrent', t, round(i_stator * abs(V) / vb, 1))
        Z.put('/Tuning/Shooter/12/ControlMode', t, 'VelocityVoltage')
        Z.put('/Tuning/Shooter/11/Velocity', t, round(w_true + rnd.gauss(0, 0.01), 3))
        Z.put('/Tuning/Shooter/11/StatorCurrent', t, i_stator)
        Z.put('/Tuning/Shooter/11/SupplyCurrent', t, round(i_stator * abs(V) / vb, 1))
        if abs(t - round(t)) < DT / 2:
            temp += 0.05
            Z.put('/Tuning/Shooter/12/Temp', t, round(temp))
            Z.put('/Tuning/Shooter/11/Temp', t, round(temp))
        # hood: step 8.5 -> 20 deg at t=10, back at 30; held down (6.8 deg) during the interlock window
        ref_deg = 20.0 if 10.0 <= t < 30.0 else 8.5
        if down:
            ref_deg = 6.8
        hood += (ref_deg - hood) * (1 - math.exp(-DT / 0.06))
        hq = round(hood / 0.0879) * 0.0879
        Z.put('/Tuning/ShooterHood/9/Position', t, round(hq / 360.0, 7))
        Z.put('/Tuning/ShooterHood/9/Reference', t, round(ref_deg / 360.0, 7))
        Z.put('/Tuning/ShooterHood/9/Voltage', t, round(max(-12, min(12, 800 * (ref_deg - hq) / 360.0)), 3))
        Z.put('/Tuning/ShooterHood/9/StatorCurrent', t, round(abs(ref_deg - hq) * 2, 1))
        Z.put('/Tuning/ShooterHood/9/ControlMode', t, 'PositionVoltage')
        Z.put('/Robot/ShooterHood/Position (Deg)', t, round(hq, 4))
        Z.put('/Robot/ShooterHood/Target Position (Deg)', t, round(ref_deg, 4))
        Z.put('/Tuning/Power/BatteryVoltage', t, vb)
        if t >= next_sys:
            w.append('systemTime', t, EPOCH0_US + int(t * 1e6))
            next_sys += 5.0
        t = round(t + DT, 6)
    # heading activation: 30 deg step at t=20, 250 Hz, exponential convergence with a small delay
    th = 0.0
    tgt = 30.0
    w.append('/Tuning/Heading/Active', 19.99, True)
    tt = 20.0
    rate = 0.0
    while tt < 22.0:
        e = ((tgt - th) + 180) % 360 - 180
        u = max(-9.42, min(9.42, 15.0 * math.radians(e)))
        rate += (0.8 * math.degrees(u) - rate) / 0.05 * 0.004
        th += rate * 0.004
        w.append('/Tuning/Heading/Target', tt, tgt)
        w.append('/Tuning/Heading/Measured', tt, round(th, 4))
        w.append('/Tuning/Heading/Error', tt, round(e, 4))
        w.append('/Tuning/Heading/Output', tt, round(u, 4))
        tt = round(tt + 0.004, 6)
    w.append('/Tuning/Heading/Active', 22.0, False)
    w.append('/Tuning/Power/Enabled', T_END, False)
    w.append('/Tuning/Power/Mode', T_END, 'disabled')
    w.close()
    return {'kS': KS, 'kV': KV, 'interlock': INTERLOCK}


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'synthetic.wpilog'
    print(make(out), '->', out)
