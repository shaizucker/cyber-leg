import math
import unittest
from ankle_trajectory import AnkleTrajectory


class AnkleTrajectoryTests(unittest.TestCase):
    def test_soft_start_common_axis_progress_and_speed_bound(self):
        planner = AnkleTrajectory([20, 20, 20])
        positions = [[20, 20, 20]]
        for _ in range(200):
            positions.append(planner.advance([30, 25, 15], 0.02))
        self.assertLess(positions[1][0] - 20, 0.02)
        for before, after in zip(positions, positions[1:]):
            self.assertLessEqual(max(abs(x-y) for x,y in zip(before, after)), 20 * 0.02 + 1e-10)
            self.assertAlmostEqual(after[0]-20, 2*(after[1]-20), places=9)
            self.assertAlmostEqual(after[0]-20, -2*(after[2]-20), places=9)
            self.assertTrue(20 <= after[0] <= 30)
        for actual, expected in zip(positions[-1], (30, 25, 15)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_measured_start_and_height_are_coordinated(self):
        planner = AnkleTrajectory([5, 10, 15])
        self.assertEqual(planner.advance([20, 20, 20], 0), [5, 10, 15])
        p = planner.advance([20, 20, 20], 0.02)
        self.assertAlmostEqual((p[0]-5)/15, (p[1]-10)/10)
        self.assertAlmostEqual((p[0]-5)/15, (p[2]-15)/5)
        planner.reset([20, 20, 20])
        p = planner.advance([30, 30, 30], 0.1)
        self.assertAlmostEqual(p[0], p[1])
        self.assertAlmostEqual(p[1], p[2])

    def test_small_noise_is_held_but_small_intentional_move_reaches_target(self):
        planner = AnkleTrajectory([20, 20, 20])
        for i in range(100):
            p = planner.advance([20 + (-1)**i * 0.01]*3, 0.02)
            self.assertEqual(p, [20, 20, 20])
        for _ in range(100):
            p = planner.advance([20.05]*3, 0.02)
        self.assertAlmostEqual(p[0], 20.05, places=6)

    def test_reversal_is_continuous_and_stays_in_travel(self):
        planner = AnkleTrajectory([20]*3)
        for _ in range(20):
            previous = list(planner.position)
            planner.advance([39, 0, 30], 0.02)
        old_velocity = [(x-y)/0.02 for x,y in zip(planner.position, previous)]
        before = list(planner.position)
        after = planner.advance([0, 39, 10], 0.02)
        new_velocity = [(x-y)/0.02 for x,y in zip(after, before)]
        self.assertLess(max(abs(x-y) for x,y in zip(old_velocity,new_velocity)), 4)
        for _ in range(300):
            before = list(planner.position)
            after = planner.advance([0, 39, 10], 0.02)
            self.assertTrue(all(0 <= x <= 39 for x in after))
            self.assertLessEqual(max(abs(x-y) for x,y in zip(before,after)), 0.4 + 1e-10)

    def test_stop_reset_removes_residual_motion_and_long_pause_is_bounded(self):
        planner = AnkleTrajectory([20]*3)
        for _ in range(20):
            planner.advance([35]*3, 0.02)
        held = list(planner.position)
        planner.reset(held)
        self.assertEqual(planner.advance(held, 0.02), held)
        before = list(planner.position)
        after = planner.advance([39]*3, 2.0)
        self.assertLessEqual(max(abs(x-y) for x,y in zip(before,after)), 20*0.04)

    def test_invalid_tuning_is_rejected(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                AnkleTrajectory([20]*3, max_speed=value)
            with self.assertRaises(ValueError):
                AnkleTrajectory([20]*3, response_time=value)


if __name__ == '__main__':
    unittest.main()
