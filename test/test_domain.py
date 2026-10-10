"""The model, checked against itself: E, A, gamma and the run r."""

import unittest

from panda_block_planner import domain
from panda_block_planner.domain import on, holding, HANDFREE, TABLE


# The states exactly as written in the model, with the model's D for the
# table spelled TABLE - the constant is "T" now that D is a block.
S0 = frozenset([on("A", TABLE), on("B", TABLE), on("C", TABLE), HANDFREE])
S1 = frozenset([on("A", TABLE), on("C", TABLE), holding("B")])
S2 = frozenset([on("A", TABLE), on("B", "C"), on("C", TABLE), HANDFREE])
S3 = frozenset([on("B", "C"), on("C", TABLE), holding("A")])
S4 = frozenset([on("A", "B"), on("B", "C"), on("C", TABLE), HANDFREE])

A0 = domain.pickup("B", TABLE)
A1 = domain.stack("B", "C")
A2 = domain.pickup("A", TABLE)
A3 = domain.stack("A", "B")


class TestRun(unittest.TestCase):
    def test_r_is_a_valid_run(self):
        """r: (s0, a0, s1, a1, s2, a2, s3, a3, s4) - every step must check out."""
        state = S0
        for action, expected in [(A0, S1), (A1, S2), (A2, S3), (A3, S4)]:
            self.assertTrue(domain.applicable(state, action),
                            "%s not applicable in %s"
                            % (action, domain.format_state(state)))
            state = domain.apply_action(state, action)
            self.assertEqual(state, expected,
                             "after %s got %s" % (action, domain.format_state(state)))

    def test_states_are_well_formed(self):
        for name, state in [("s0", S0), ("s1", S1), ("s2", S2),
                            ("s3", S3), ("s4", S4)]:
            self.assertEqual(domain.check_state(state), [], name)

    def test_action_notation(self):
        self.assertEqual(str(A0), "pickup(B)")
        self.assertEqual(str(A1), "stack(B,C)")
        self.assertEqual(str(domain.stack("C", TABLE)), "stack(C,T)")

    def test_state_notation(self):
        self.assertEqual(domain.format_state(S1),
                         "{on(A,T), on(C,T), holding(B)}")


class TestOperators(unittest.TestCase):
    def test_cannot_pick_up_with_a_full_hand(self):
        self.assertFalse(domain.applicable(S1, domain.pickup("A", TABLE)))

    def test_cannot_pick_up_a_covered_block(self):
        # C is on A, so A is not clear.
        state = domain.state_from_towers([["A", "C"], ["B"]])
        self.assertFalse(domain.applicable(state, domain.pickup("A", TABLE)))
        self.assertTrue(domain.applicable(state, domain.pickup("C", "A")))

    def test_cannot_stack_onto_a_covered_block(self):
        state = domain.state_from_towers([["A", "C"], ["B"]])
        state = domain.apply_action(state, domain.pickup("B", TABLE))
        self.assertFalse(domain.applicable(state, domain.stack("B", "A")))
        self.assertTrue(domain.applicable(state, domain.stack("B", "C")))

    def test_table_is_always_clear(self):
        state = domain.state_from_towers([["A"], ["B"], ["C"]])
        self.assertTrue(domain.is_clear(state, TABLE))
        state = domain.apply_action(state, domain.pickup("A", TABLE))
        self.assertTrue(domain.applicable(state, domain.stack("A", TABLE)))

    def test_pickup_effect_depends_on_the_source(self):
        """The reason pickup carries a support it does not print."""
        state = domain.state_from_towers([["A", "C"], ["B"]])
        after = domain.apply_action(state, domain.pickup("C", "A"))
        self.assertTrue(domain.is_clear(after, "A"))

    def test_inapplicable_action_raises(self):
        with self.assertRaises(ValueError):
            domain.apply_action(S1, domain.pickup("A", TABLE))


class TestStateSpace(unittest.TestCase):
    def test_E_has_22_states(self):
        """13 handfree arrangements of 3 blocks + 3 x 3 holding states."""
        self.assertEqual(len(domain.reachable_states(S0)), 22)

    def test_18_ground_actions(self):
        self.assertEqual(len(domain.ground_actions(["A", "B", "C"])), 18)

    def test_every_state_in_the_model_is_reachable(self):
        space = domain.reachable_states(S0)
        for state in [S1, S2, S3, S4]:
            self.assertIn(state, space)

    def test_the_space_is_strongly_connected(self):
        """Anything undone can be redone: every state reaches every other."""
        space = domain.reachable_states(S0)
        for state in space:
            self.assertEqual(len(domain.reachable_states(state)), 22)

    def test_four_blocks(self):
        s = domain.state_from_towers([["A"], ["B"], ["C"], ["E"]])
        # 73 handfree arrangements + 4 * 13 holding states.
        self.assertEqual(len(domain.reachable_states(s)), 73 + 4 * 13)


class TestTowers(unittest.TestCase):
    def test_round_trip(self):
        towers = [["A", "C"], ["B"]]
        self.assertEqual(domain.towers_of(domain.state_from_towers(towers)),
                         towers)

    def test_height(self):
        state = S4  # A on B on C on the table
        self.assertEqual(domain.height_of(state, "C"), 0)
        self.assertEqual(domain.height_of(state, "B"), 1)
        self.assertEqual(domain.height_of(state, "A"), 2)


class TestMalformedStates(unittest.TestCase):
    def test_two_blocks_on_one(self):
        bad = frozenset([on("A", "C"), on("B", "C"), on("C", TABLE), HANDFREE])
        self.assertTrue(any("several blocks" in p
                            for p in domain.check_state(bad)))

    def test_held_and_supported(self):
        bad = frozenset([on("A", TABLE), holding("A")])
        self.assertTrue(domain.check_state(bad))

    def test_cycle(self):
        bad = frozenset([on("A", "B"), on("B", "A"), HANDFREE])
        self.assertTrue(any("cycle" in p for p in domain.check_state(bad)))

    def test_floating_block(self):
        bad = frozenset([on("A", TABLE), on("B", TABLE), HANDFREE])
        self.assertEqual(domain.check_state(bad), [])  # only A and B exist


if __name__ == "__main__":
    unittest.main()
