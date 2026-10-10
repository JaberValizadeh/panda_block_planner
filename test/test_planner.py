"""Search over E: does it find r, and does it find the runs r cannot express."""

import collections
import unittest

from panda_block_planner import domain, geometry, planner, scenarios
from panda_block_planner.domain import TABLE


class TestModelScenario(unittest.TestCase):
    """The planner should rediscover the run that was written by hand."""

    def setUp(self):
        self.scenario = scenarios.SCENARIOS["model"]

    def test_finds_the_hand_written_run(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertIsNotNone(result)
        self.assertEqual([str(a) for a in result.actions],
                         ["pickup(B)", "stack(B,C)", "pickup(A)", "stack(A,B)"])

    def test_the_run_ends_in_s4(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertEqual(domain.format_state(result.final),
                         "{on(A,B), on(B,C), on(C,T), handfree}")

    def test_four_actions_is_optimal(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertEqual(len(result), 4)

    def test_states_and_actions_line_up(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertEqual(len(result.states), len(result.actions) + 1)
        for state, action, nxt in result:
            self.assertEqual(domain.apply_action(state, action), nxt)


class TestSussman(unittest.TestCase):
    """The case a fixed SEQUENCE cannot do: it needs the table as a buffer."""

    def setUp(self):
        self.scenario = scenarios.SCENARIOS["sussman"]

    def test_six_actions(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 6)

    def test_it_puts_C_down_first(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertEqual([str(a) for a in result.actions],
                         ["pickup(C)", "stack(C,T)", "pickup(B)",
                          "stack(B,C)", "pickup(A)", "stack(A,B)"])

    def test_a_putdown_is_required(self):
        """stack(x,T) is not optional here - no plan avoids it.

        Only because C is the one spare block. The scene has five now, and if
        the two this scenario does not name were left in the state instead of
        stowed, C could be parked on one of them and the anomaly would stop
        being one. See the note in scenarios.py.
        """
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertTrue(any(a.kind == "stack" and a.support == TABLE
                            for a in result.actions))

    def test_greedy_single_goal_ordering_would_be_wrong(self):
        """Satisfying on(A,B) first is a trap; the optimal plan does it last."""
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertEqual(str(result.actions[-1]), "stack(A,B)")


class TestTrivialAndUnreachable(unittest.TestCase):
    def test_already_satisfied(self):
        state = domain.state_from_towers([["C", "B", "A"]])
        result = planner.plan(state, scenarios.GOAL_ABC)
        self.assertEqual(len(result), 0)
        self.assertEqual(result.initial, result.final)

    def test_malformed_initial_state_is_rejected(self):
        bad = frozenset([domain.on("A", "B"), domain.on("B", "A"),
                         domain.HANDFREE])
        with self.assertRaises(ValueError):
            planner.plan(bad, scenarios.GOAL_ABC)

    def test_inverted_tower(self):
        scenario = scenarios.SCENARIOS["inverted"]
        result = planner.plan(scenario.initial, scenario.goal)
        self.assertIsNotNone(result)
        # A off, B off, then rebuild the other way up: 3 transfers.
        self.assertEqual(len(result), 6)


class TestReachFilter(unittest.TestCase):
    """Geometrically impossible actions must not appear in a plan."""

    def test_far_base_is_not_used_as_a_base(self):
        scenario = scenarios.SCENARIOS["far_base"]
        feasible = geometry.reach_filter(scenario.table_xy, (0.48, -0.28))
        result = planner.plan(scenario.initial, scenario.goal,
                              feasible=feasible)
        self.assertIsNotNone(result, "should build the tower somewhere else")
        # Whatever it built, nothing ended up stacked on C at x = 0.70.
        self.assertFalse(any(a.kind == "stack" and a.support == "C"
                             for a in result.actions),
                         [str(a) for a in result.actions])
        self.assertEqual(scenarios.tallest_tower(result.final), 3)

    def test_the_filter_rejects_the_action_the_model_allows(self):
        """The two layers disagree, which is the whole reason the hook exists.

        Symbolically, stacking A onto B once B sits on C is textbook legal.
        Geometrically, at x = 0.70 that puts the wrist somewhere it cannot
        plan to - so the edge is real in E and absent from the search graph.
        """
        scenario = scenarios.SCENARIOS["far_base"]
        feasible = geometry.reach_filter(scenario.table_xy, (0.48, -0.28))
        state = domain.state_from_towers([["C", "B"], ["A"]])
        state = domain.apply_action(state, domain.pickup("A", TABLE))
        action = domain.stack("A", "B")
        self.assertTrue(domain.applicable(state, action))
        self.assertFalse(feasible(state, action))

    def test_a_four_high_tower_avoids_the_far_side(self):
        scenario = scenarios.SCENARIOS["four_blocks"]
        feasible = geometry.reach_filter(scenario.table_xy, (0.48, -0.28))
        result = planner.plan(scenario.initial, scenario.goal,
                              feasible=feasible)
        self.assertIsNotNone(result)
        base = geometry.base_of(result.final, "A")
        # It must pick a base whose column can actually service a four-high
        # tower; C, out at x = 0.62, cannot.
        top = geometry.place_z_on(geometry.block_z(2)) + geometry.MIN_CLEARANCE
        self.assertGreaterEqual(geometry.max_tcp_z(*scenario.table_xy[base]),
                                top)
        self.assertLess(geometry.max_tcp_z(*scenario.table_xy["C"]), top)

    def test_named_far_tower_is_reported_as_a_reach_failure(self):
        """Insisting on the unreachable base fails, and says why."""
        scenario = scenarios.SCENARIOS["far_base"]
        feasible = geometry.reach_filter(scenario.table_xy, (0.48, -0.28))
        goal = [domain.on("A", "B"), domain.on("B", "C")]
        self.assertIsNone(planner.plan(scenario.initial, goal,
                                       feasible=feasible))
        why = planner.explain_failure(scenario.initial, goal, feasible)
        self.assertIn("reach envelope", why)

    def test_the_original_demo_is_still_feasible(self):
        """The real two-transfer run must not be filtered out by accident."""
        scenario = scenarios.SCENARIOS["model"]
        feasible = geometry.reach_filter(scenario.table_xy, (0.48, -0.28))
        result = planner.plan(scenario.initial, scenario.goal,
                              feasible=feasible)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)


class TestFiveBlocks(unittest.TestCase):
    """B on D on A, E on C, and a goal tower that reverses the alphabet."""

    def setUp(self):
        self.scenario = scenarios.SCENARIOS["five_blocks"]

    def feasible(self):
        buffer_xy = geometry.free_slot(list(self.scenario.table_xy.values()))
        return geometry.reach_filter(self.scenario.table_xy, buffer_xy)

    def test_the_initial_state_is_the_one_asked_for(self):
        self.assertEqual(domain.format_state(self.scenario.initial),
                         "{on(A,T), on(B,D), on(C,T), on(D,A), on(E,C), "
                         "handfree}")

    def test_twelve_actions(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 12)

    def test_B_is_the_one_block_that_moves_twice(self):
        """Ten transfers would do it if every block could go straight home.

        B cannot: it is on top at the start, so it comes off first, and the
        support it belongs on - C - is not on D yet and will not be for four
        more actions. So B goes to the table and is fetched again, and that
        second transfer is the whole difference between 10 and 12.
        """
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        moved = collections.Counter(a.block for a in result.actions
                                    if a.kind == "pickup")
        self.assertEqual(moved["B"], 2)
        for block in "ACDE":
            self.assertEqual(moved[block], 1, block)

    def test_the_tower_is_built_from_the_table_up(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        stacks = [a.support for a in result.actions if a.kind == "stack"]
        # ... after the two blocks that had to be got out of the way first.
        self.assertEqual(stacks[-4:], ["E", "D", "C", "B"])

    def test_a_five_high_tower_still_fits_under_the_reach_envelope(self):
        """Only just, and only near in: the top goes on at z = 0.6025."""
        result = planner.plan(self.scenario.initial, self.scenario.goal,
                              feasible=self.feasible())
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 12)

    def test_it_would_not_fit_out_at_the_far_edge(self):
        """The same goal with the tower forced to x = 0.62 is unreachable."""
        far = geometry.reach_filter(self.scenario.table_xy, (0.62, 0.0))
        self.assertIsNone(planner.plan(self.scenario.initial,
                                       self.scenario.goal, feasible=far))
        self.assertIn("reach envelope",
                      planner.explain_failure(self.scenario.initial,
                                              self.scenario.goal, far))


class TestSixBlocks(unittest.TestCase):
    """Two inverted three-towers merged into one six-high tower."""

    def setUp(self):
        self.scenario = scenarios.SCENARIOS["six_blocks"]

    def feasible(self):
        buffer_xy = geometry.free_slot(list(self.scenario.table_xy.values()))
        return geometry.reach_filter(self.scenario.table_xy, buffer_xy)

    def test_the_initial_state_is_two_inverted_towers(self):
        self.assertEqual(domain.format_state(self.scenario.initial),
                         "{on(A,T), on(B,A), on(C,B), on(D,T), on(E,D), "
                         "on(F,E), handfree}")

    def test_twelve_actions_which_is_the_floor(self):
        """Six blocks all out of place cannot be done in fewer than 12."""
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 12)

    def test_every_block_moves_exactly_once(self):
        """Unlike five_blocks, nothing needs the table twice.

        Unstacking the second tower onto the table *is* the bottom of the goal
        tower, so each block goes straight from where it starts to where it
        belongs.
        """
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        moved = collections.Counter(a.block for a in result.actions
                                    if a.kind == "pickup")
        for block in "ABCDEF":
            self.assertEqual(moved[block], 1, block)

    def test_the_tower_is_built_from_the_table_up(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal)
        stacks = [a.support for a in result.actions if a.kind == "stack"]
        self.assertEqual(stacks, [domain.TABLE, "F", "E", "D", "C", "B"])

    def test_a_six_high_tower_fits_only_on_the_near_slots(self):
        """The top goes on at z = 0.6475, which not every slot can service."""
        top = geometry.place_z_on(geometry.block_z(4)) + geometry.MIN_CLEARANCE
        usable = [x for x in geometry.SLOT_X if top <= geometry.max_tcp_z(x)]
        self.assertTrue(usable, "no slot can build a six-high tower")
        self.assertNotEqual(usable, list(geometry.SLOT_X),
                            "if every slot works, the envelope has gone soft")

    def test_the_planner_finds_it_under_the_reach_filter(self):
        result = planner.plan(self.scenario.initial, self.scenario.goal,
                              feasible=self.feasible())
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 12)

    def test_it_would_not_fit_out_at_the_far_edge(self):
        far = geometry.reach_filter(self.scenario.table_xy, (0.62, 0.0))
        self.assertIsNone(planner.plan(self.scenario.initial,
                                       self.scenario.goal, feasible=far))


class TestRunFormatting(unittest.TestCase):
    def test_run_is_printed_in_the_model_notation(self):
        scenario = scenarios.SCENARIOS["model"]
        text = planner.plan(scenario.initial, scenario.goal).format_run()
        self.assertIn("r: (s0, a0, s1, a1, s2, a2, s3, a3, s4)", text)
        self.assertIn("s0 = {on(A,T), on(B,T), on(C,T), handfree}", text)
        self.assertIn("a1 = stack(B,C)", text)


if __name__ == "__main__":
    unittest.main()
