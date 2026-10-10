"""The metric layer: reach envelope, stack heights, buffer slots."""

import unittest

from panda_block_planner import domain, geometry


class TestReachEnvelope(unittest.TestCase):
    """The surface measured by tools/measure_reach.py, interpolated."""

    # tools/measure_reach.py --x 0.62 --y 0.0 --step 0.002 --attempts 12
    MEASURED_AT_062 = 0.606

    def test_agrees_with_the_point_the_old_model_was_built_on(self):
        """(0.62, 0.633) was unplannable; (0.62, 0.603) planned.

        The sweep puts the IK ceiling there at 0.606, between the two, which
        is the one independent check available on the whole table.
        """
        self.assertLess(self.MEASURED_AT_062, 0.633)
        self.assertGreater(self.MEASURED_AT_062, 0.603)

    def test_the_interpolation_is_conservative_there_but_not_uselessly(self):
        """x = 0.62 falls between grid columns, so it is interpolated.

        Interpolating a convex surface across a 60 mm gap undershoots. That is
        the safe direction - a ceiling guessed too low refuses a pose the arm
        could have made, one guessed too high sends it somewhere it cannot go
        - but it should not undershoot so far that the table goes to waste.
        """
        limit = geometry.max_tcp_z(0.62)
        self.assertLessEqual(limit, self.MEASURED_AT_062)
        self.assertGreater(limit, self.MEASURED_AT_062 - 0.02)

    def test_reach_falls_off_with_distance(self):
        heights = [geometry.max_tcp_z(x) for x in (0.45, 0.50, 0.55, 0.62, 0.70)]
        self.assertEqual(heights, sorted(heights, reverse=True))

    def test_reach_falls_off_sideways_too(self):
        """The thing the old x-only model could not say.

        It claimed a flat 0.75 everywhere inboard of x = 0.50; the real
        ceiling at (0.48, 0.28) is 0.660.
        """
        heights = [geometry.max_tcp_z(0.48, y) for y in (0.0, 0.12, 0.20, 0.28)]
        self.assertEqual(heights, sorted(heights, reverse=True))
        self.assertGreater(heights[0] - heights[-1], 0.05)

    def test_it_is_symmetric_about_the_centre_line(self):
        for x in geometry.REACH_X[:-1]:
            for y in (0.12, 0.20, 0.28):
                self.assertAlmostEqual(geometry.max_tcp_z(x, y),
                                       geometry.max_tcp_z(x, -y))

    def test_y_defaults_to_the_most_generous_row(self):
        self.assertEqual(geometry.max_tcp_z(0.48), geometry.max_tcp_z(0.48, 0.0))

    def test_clamped_close_in(self):
        """Inboard of the grid the ceiling is held, not extrapolated upward."""
        self.assertEqual(geometry.max_tcp_z(0.30),
                         geometry.max_tcp_z(geometry.REACH_X[0]))

    def test_past_the_end_of_the_arm_nothing_is_allowed(self):
        """A cell with no IK solution is refused outright, not interpolated."""
        self.assertEqual(geometry.max_tcp_z(0.72, 0.28), geometry.UNREACHABLE)

    def test_far_out_keeps_falling_rather_than_flattening(self):
        """Extrapolation past the far edge must not wave placements through."""
        self.assertLess(geometry.max_tcp_z(0.80),
                        geometry.max_tcp_z(geometry.REACH_X[-1]))


class TestHeights(unittest.TestCase):
    def test_block_on_table(self):
        self.assertAlmostEqual(geometry.block_z(0), 0.4225)

    def test_three_high_tower(self):
        # Matches the verified result in panda_block_stack's README.
        self.assertAlmostEqual(geometry.block_z(1), 0.4675)
        self.assertAlmostEqual(geometry.block_z(2), 0.5125)

    def test_place_z_is_one_edge_up(self):
        self.assertAlmostEqual(geometry.place_z_on(geometry.block_z(1)),
                               geometry.block_z(2))


class TestBaseAndXY(unittest.TestCase):
    def setUp(self):
        self.table_xy = {"A": (0.45, -0.20), "B": (0.45, 0.20),
                         "C": (0.62, 0.0)}

    def test_stacked_block_inherits_its_base_position(self):
        state = domain.state_from_towers([["C", "B", "A"]])
        self.assertEqual(geometry.base_of(state, "A"), "C")
        self.assertEqual(geometry.xy_of(state, "A", self.table_xy, (0.5, 0.0)),
                         (0.62, 0.0))

    def test_held_block_has_no_position(self):
        state = domain.state_from_towers([["A"], ["B"], ["C"]])
        state = domain.apply_action(state, domain.pickup("A", domain.TABLE))
        self.assertIsNone(
            geometry.xy_of(state, "A", self.table_xy, (0.5, 0.0)))


class TestFreeSlot(unittest.TestCase):
    def test_keeps_clear_of_existing_blocks(self):
        occupied = [(0.45, -0.20), (0.45, 0.20), (0.62, 0.0)]
        slot = geometry.free_slot(occupied)
        for xy in occupied:
            self.assertGreaterEqual(geometry.distance(slot, xy),
                                    geometry.SLOT_CLEARANCE)

    def test_avoids_the_tower_footprint(self):
        """A buffer slot under the future tower costs a second transfer."""
        tower = (0.48, 0.0)
        slot = geometry.free_slot([(0.45, -0.20)], avoid=[tower])
        self.assertGreaterEqual(geometry.distance(slot, tower),
                                geometry.SLOT_CLEARANCE)

    def test_prefers_slots_close_to_the_base(self):
        slot = geometry.free_slot([])
        self.assertEqual(slot, (min(geometry.SLOT_X), 0.0))

    def test_raises_when_the_table_is_full(self):
        crowded = [(x, y) for x in geometry.SLOT_X for y in geometry.SLOT_Y]
        with self.assertRaises(RuntimeError):
            geometry.free_slot(crowded)


if __name__ == "__main__":
    unittest.main()
