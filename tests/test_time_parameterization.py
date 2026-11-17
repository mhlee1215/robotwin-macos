"""Regression: almost-stationary IK previously made the EE API fail."""
import unittest
import numpy as np
from robotwin_macos.cpu_planner import CpuPlanner


class TimeParameterizationTests(unittest.TestCase):
    def test_stationary_and_submilliradian_corrections(self):
        planner = CpuPlanner.__new__(CpuPlanner)
        for delta in [0., 1e-8, 1e-5, 0.0009, 0.08]:
            with self.subTest(delta=delta):
                times, pos, vel, acc, duration = planner.TOPP(np.array([[0.]*6, [delta]*6]))
                self.assertTrue(np.isfinite(pos).all())
                np.testing.assert_allclose(pos[-1], delta, atol=1e-8)
                self.assertLessEqual(np.abs(vel).max(), 1.001)
                self.assertLessEqual(np.abs(acc).max(), 2.001)
                self.assertEqual(times[-1], duration)

    def test_fallback_preserves_corners_and_limits(self):
        planner = CpuPlanner.__new__(CpuPlanner)
        path = np.array([[0., 0.], [0.3, -0.1], [-0.2, 0.4]])
        times, pos, vel, acc, _ = planner._quintic_path(path, 0.004)
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertLess(np.min(np.linalg.norm(pos-path[1], axis=1)), 1e-8)
        np.testing.assert_allclose(pos[-1], path[-1])
        self.assertLessEqual(np.abs(vel).max(), 1.001)
        self.assertLessEqual(np.abs(acc).max(), 2.001)


if __name__ == '__main__':
    unittest.main()
