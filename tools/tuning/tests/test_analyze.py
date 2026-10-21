"""Unit tests for the tuning toolkit.

usage: py -3.14 -I tests\\test_analyze.py      (from tools\\tuning; writes only to a temporary directory)
"""
import json
import os
import struct
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import analyze  # noqa: E402
import make_synthetic_log  # noqa: E402
import recommend  # noqa: E402
import wpilog  # noqa: E402

ELIGIBLE = {'gain_tuning_eligible_after_utc': '2026-01-01T00:00:00Z', 'declared_by': 'test'}


class SyntheticLog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.tmp.name, 'FRC_20261022_120000.wpilog')
        cls.truth = make_synthetic_log.make(cls.path)
        targets = analyze.load_json(analyze.P.TARGETS_JSON, {})
        cls.ds, cls.report, cls.info, cls.rec_pass = analyze.analyze_log(cls.path, targets, ELIGIBLE, [])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_ff_fit_recovers_ks_kv(self):
        ff = self.report['mechanisms']['Shooter']['analysis']['motors']['12']['ff_fit']
        self.assertEqual(ff['status'], 'ok')
        self.assertTrue(ff['kV_identifiable'])
        self.assertAlmostEqual(ff['kS'], self.truth['kS'], delta=0.05 * self.truth['kS'])
        self.assertAlmostEqual(ff['kV'], self.truth['kV'], delta=0.05 * self.truth['kV'])
        self.assertGreater(ff['r2'], 0.99)

    def test_interlock_violation_flagged(self):
        il = self.report['interlock']
        self.assertEqual(il['status'], 'FAIL')
        self.assertGreaterEqual(il['violation_count'], 1)
        v = il['violations'][0]
        a, b = self.truth['interlock']
        self.assertLess(abs(v['t0'] - a), 0.1)
        self.assertLess(abs(v['t1'] - b), 0.1)

    def test_eligible_and_proposal_bounded(self):
        self.assertEqual(self.report['eligibility']['mode'], 'all')
        props = [p for p in self.report['recommendations']['proposals']
                 if p['mechanism'] == 'Shooter' and p['parameter'] == 'kS']
        self.assertEqual(len(props), 1)
        p = props[0]
        self.assertAlmostEqual(p['proposed'], 0.25, places=3)   # 0.2 + 25% bound toward the fitted 0.3

    def test_hood_and_heading(self):
        hood = self.report['mechanisms']['ShooterHood']['analysis']
        self.assertGreaterEqual(hood['step_count'], 2)
        h = self.report['heading']
        self.assertEqual(h['activations'], 1)
        self.assertAlmostEqual(h['steps'][0]['e0_deg'], 30.0, delta=0.5)
        self.assertIsNotNone(h['steps'][0]['t_lt1_s'])

    def test_ineligible_without_signoff(self):
        targets = analyze.load_json(analyze.P.TARGETS_JSON, {})
        _, rep, _, _ = analyze.analyze_log(self.path, targets, {'gain_tuning_eligible_after_utc': None}, [])
        self.assertEqual(rep['eligibility']['mode'], 'none')
        self.assertEqual(rep['recommendations']['proposals'], [])

    def test_exclusion_blocks_mechanism(self):
        targets = analyze.load_json(analyze.P.TARGETS_JSON, {})
        elig = dict(ELIGIBLE, excluded_mechanisms=[{'mechanism': 'Shooter', 'from_utc': '2026-01-01T00:00:00Z',
                                                    'to_utc': None, 'reason': 'test'}])
        _, rep, _, _ = analyze.analyze_log(self.path, targets, elig, [])
        self.assertFalse([p for p in rep['recommendations']['proposals'] if p['mechanism'] == 'Shooter'])

    def test_no_follower_events_when_coupled(self):
        h = self.report['mechanisms']['Shooter']['health']
        self.assertFalse([e for e in h['events'] if e['type'] in ('decoupled', 'follower')])


class DecoupledFollower(unittest.TestCase):
    """Follower 11 at 0.6 x the leader for most of the log: the whole-log median ratio is far from 1, which before
    1.1.0 was taken as a units difference and disabled the ratio check (intake motor 6, FRC_20261021_222140)."""
    WINDOW = (5.0, 40.0)

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        path = os.path.join(cls.tmp.name, 'FRC_20261022_130000.wpilog')
        make_synthetic_log.make(path, decouple=cls.WINDOW)
        targets = analyze.load_json(analyze.P.TARGETS_JSON, {})
        _, cls.report, _, _ = analyze.analyze_log(path, targets, ELIGIBLE, [])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_decoupled_follower_faulty_and_flagged(self):
        h = self.report['mechanisms']['Shooter']['health']
        self.assertEqual(h['verdict'], 'faulty')
        dec = [e for e in h['events'] if e['type'] == 'decoupled']
        self.assertTrue(dec)
        self.assertTrue(all(str(e['motor']) == '11' for e in dec))
        self.assertGreater(sum(e['duration_s'] for e in dec), 25.0)
        self.assertGreaterEqual(dec[0]['t0'], self.WINDOW[0] - 0.1)
        self.assertAlmostEqual(dec[0]['median_ratio'], 0.6, delta=0.02)
        flags = [f['text'] for f in self.report['recommendations']['flags'] if f['mechanism'] == 'Shooter']
        self.assertTrue(any('detached or slipping drive' in t for t in flags))
        self.assertFalse([p for p in self.report['recommendations']['proposals'] if p['mechanism'] == 'Shooter'])


class SwerveDriveProposal(unittest.TestCase):
    POLICY = analyze.load_json(analyze.P.TARGETS_JSON, {}).get('policy', {})

    @staticmethod
    def analysis(ks_ci):
        """Pooled drive fit shaped like FRC_20261021_222140 (kS 0.2358, kV 0.1111 per rotor rot/s)."""
        motors = {str(c): {'ff_fit': {'status': 'ok', 'kV_identifiable': True, 'kV': kv}}
                  for c, kv in ((1, 0.1140), (4, 0.1123), (7, 0.1088), (10, 0.1136))}
        pf = {'status': 'ok', 'kV_identifiable': True, 'windows': 42, 'steady_s': 16.8, 'r2': 0.9995, 'kS': 0.2358,
              'kV': 0.1111, 'kS_ci95': ks_ci, 'kV_ci95': 0.0044, 'kS_ci_rel': ks_ci / 0.2358, 'kV_ci_rel': 0.0044 / 0.1111,
              'setpoints_rpm': [-1237, -712, 398, 659, 1598], 'kV_per_mps': 2.098}
        return {'gains': {'kP': 0.1, 'kS': 0.0, 'kV': 0.124}, 'pooled_ff_fit': pf, 'motors': motors}

    def test_kv_step_and_initial_ks(self):
        R = recommend.Recs()
        recommend._swerve_drive_props(R, 'SwerveDrive', self.analysis(0.0444), self.POLICY, 0)
        props = {p['parameter']: p for p in R.proposals}
        self.assertAlmostEqual(props['kV']['proposed'], 0.1111, places=4)   # -10.4%, inside the 25% bound
        self.assertEqual(props['kV']['confidence'], 'medium')               # kS CI 19% > 10%: not high
        self.assertEqual(props['kV']['motor'], 'all')
        self.assertAlmostEqual(props['kS']['proposed'], 0.2358, places=4)   # current 0: initial value
        self.assertEqual(props['kS']['confidence'], 'low')

    def test_wide_ci_blocks_only_that_parameter(self):
        R = recommend.Recs()
        recommend._swerve_drive_props(R, 'SwerveDrive', self.analysis(0.1), self.POLICY, 0)   # kS CI 42%
        self.assertEqual([p['parameter'] for p in R.proposals], ['kV'])
        self.assertTrue(any(b['parameter'] == 'kS' for b in R.not_proposed))


class Reader(unittest.TestCase):
    def test_truncated_final_record_and_types(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 't.wpilog')
            with wpilog.Writer(p) as w:
                w.start('/Robot/DogLog/QueueRemainingCapacity', 'int64')
                w.start('d', 'double')
                w.start('s', 'string[]')
                w.start('/.schema/struct:Translation2d', 'structschema')
                w.start('pt', 'struct:Translation2d')
                w.append('/.schema/struct:Translation2d', 0.1, 'double x;double y')
                w.append('/Robot/DogLog/QueueRemainingCapacity', 0.2, 3609)
                w.append('s', 0.3, ['a', 'bc'])
                w.append('pt', 0.4, struct.pack('<dd', 1.5, -2.0))
                w.append('d', 0.5, 1.25)
                w.append('d', 0.6, 2.5)
            with open(p, 'ab') as f:   # truncated record: header promises 8 bytes, file ends
                f.write(bytes([0x20, 2, 8, 0x10, 0x27, 0x00]) + b'\x01\x02')
            L = wpilog.read(p)
            self.assertTrue(L.truncated)
            self.assertEqual(L.get('/Robot/DogLog/QueueRemainingCapacity').v[0], 3609)
            self.assertEqual(L.get('/Robot/DogLog/QueueRemainingCapacity').type, 'int64')
            self.assertEqual(list(L.get('d').v), [1.25, 2.5])
            self.assertEqual(L.get('s').v[0], ('a', 'bc'))
            self.assertEqual(L.get('pt').v[0], {'x': 1.5, 'y': -2.0})

    def test_zoh(self):
        z = wpilog.ZOH([1.0, 2.0, 4.0], [10, 20, 40])
        self.assertIsNone(z.at(0.5))
        self.assertEqual(z.at(3.9), 20)
        self.assertEqual(wpilog.resample([1.0, 2.0], [5, 6], [0.0, 1.0, 1.5, 3.0]), [None, 5, 5, 6])
        self.assertEqual(wpilog.intervals([0, 1, 2, 3], [0, 1, 1, 0], bool), [(1, 3)])


if __name__ == '__main__':
    unittest.main(verbosity=2)
