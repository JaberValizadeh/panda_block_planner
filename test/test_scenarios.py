"""The scenario layer: layouts, and the blocks a scenario is not about.

The world spawns five blocks whatever the problem is. A scenario that wants
three has to (a) put the other two somewhere harmless and (b) keep them out of
the state, and both halves have to work or the problem quietly changes shape -
`sussman` with a spare cube on the table is not the Sussman anomaly. These
tests hold both halves down.
"""

import unittest

from geometry_msgs.msg import Pose

from panda_block_planner import domain, geometry, scenarios, world_model


class TestStowedBlocks(unittest.TestCase):
    def test_every_scenario_accounts_for_every_block_in_the_world(self):
        for name, scenario in sorted(scenarios.SCENARIOS.items()):
            named = scenario.blocks | set(scenario.stowed)
            self.assertEqual(named, set(scenarios.WORLD_BLOCKS), name)

    def test_setup_places_the_whole_world_not_just_the_cast(self):
        """A block left where the last run dropped it is one the arm can hit."""
        for name, scenario in sorted(scenarios.SCENARIOS.items()):
            self.assertEqual(set(scenario.block_poses()),
                             set(scenarios.WORLD_BLOCKS), name)

    def test_stowed_blocks_sit_on_the_table_at_the_back(self):
        scenario = scenarios.SCENARIOS["sussman"]
        # Whatever the world grew to, everything sussman does not name.
        self.assertEqual(set(scenario.stowed),
                         set(scenarios.WORLD_BLOCKS) - set("ABC"))
        self.assertTrue(scenario.stowed)
        for block in scenario.stowed:
            x, y, z = scenario.block_poses()[block]
            self.assertEqual(x, geometry.STOW_X)
            self.assertAlmostEqual(z, geometry.block_z(0))
            # On the table top, not over an edge - a block below the table is
            # how a fallen one is recognised, and that is fatal to a run.
            half_x, half_y = geometry.TABLE_SIZE[0] / 2, geometry.TABLE_SIZE[1] / 2
            self.assertLess(abs(x - geometry.TABLE_CENTER[0]),
                            half_x - geometry.BLOCK / 2)
            self.assertLess(abs(y - geometry.TABLE_CENTER[1]),
                            half_y - geometry.BLOCK / 2)

    def test_there_is_a_stow_slot_for_every_block_in_the_world(self):
        """A scenario using one block would stow all the rest."""
        self.assertGreaterEqual(len(geometry.STOW_Y),
                                len(scenarios.WORLD_BLOCKS))

    def test_stow_slots_are_a_block_apart(self):
        for a, b in zip(geometry.STOW_Y, geometry.STOW_Y[1:]):
            self.assertGreater(b - a, geometry.BLOCK)

    def test_the_stow_strip_is_clear_of_every_buffer_slot(self):
        """Nothing put down can land on a stowed block, which is unperceived."""
        for index in range(len(scenarios.WORLD_BLOCKS)):
            stowed = geometry.stow_xy(index)
            for x in geometry.SLOT_X:
                for y in geometry.SLOT_Y:
                    self.assertGreaterEqual(
                        geometry.distance(stowed, (x, y)),
                        geometry.SLOT_CLEARANCE,
                        "stow %s is within a block of slot %s" % (stowed, (x, y)))

    def test_towers_still_stack_one_edge_at_a_time(self):
        poses = scenarios.SCENARIOS["five_blocks"].block_poses()
        self.assertAlmostEqual(poses["A"][2], geometry.block_z(0))
        self.assertAlmostEqual(poses["D"][2], geometry.block_z(1))
        self.assertAlmostEqual(poses["B"][2], geometry.block_z(2))
        self.assertEqual(poses["A"][:2], poses["B"][:2])

    def test_a_scenario_the_world_cannot_supply_is_not_physical(self):
        one_too_many = list(scenarios.WORLD_BLOCKS) + ["Z"]
        extra = scenarios.Scenario("extra", "one block past the scene",
                                   [(one_too_many, (0.5, 0.0))], [], "")
        self.assertFalse(extra.runnable)
        self.assertNotIn("extra", scenarios.PHYSICAL)

    def test_the_scenarios_that_exist_can_all_be_run(self):
        self.assertEqual(set(scenarios.PHYSICAL), set(scenarios.SCENARIOS))


class TestPerceptionIgnoresStowedBlocks(unittest.TestCase):
    """The other half: a stowed block must not reach the state.

    Without this, sussman's spare cubes would give C somewhere to go that is
    not the table, and the putdown the scenario exists to force would stop
    being forced.
    """

    def model_states(self):
        class States(object):
            pass
        states = States()
        states.name = ["ground_plane", "panda", "table",
                       "block_A", "block_B", "block_C", "block_D", "block_E"]
        states.pose = []
        for x, y, z in [(0, 0, 0), (0, 0, 0), (0.55, 0, 0.2),
                        (0.45, -0.20, 0.4225), (0.45, 0.20, 0.4225),
                        (0.45, -0.20, 0.4675),
                        (geometry.STOW_X, -0.32, 0.4225),
                        (geometry.STOW_X, -0.16, 0.4225)]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            states.pose.append(pose)
        return states

    def world(self, ignore):
        world = world_model.WorldModel(ignore=ignore)
        world._raw_states = self.model_states
        return world

    def test_ignored_blocks_are_not_in_the_poses(self):
        self.assertEqual(sorted(self.world(("D", "E")).poses()),
                         ["A", "B", "C"])

    def test_sussman_stays_the_sussman_anomaly(self):
        obs = self.world(("D", "E")).perceive()
        self.assertEqual(obs.state,
                         scenarios.SCENARIOS["sussman"].initial)
        self.assertEqual(obs.problems, [])

    def test_left_in_they_would_join_the_state(self):
        """Why the filter is not cosmetic."""
        obs = self.world(()).poses()
        self.assertIn("D", obs)
        state = world_model.abstract(obs).state
        self.assertIn(domain.on("D", domain.TABLE), state)


if __name__ == "__main__":
    unittest.main()
