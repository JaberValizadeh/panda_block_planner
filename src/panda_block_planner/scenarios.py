"""Named (initial state, goal) pairs to plan and run.

A scenario is a *physical* layout - towers with the table position of each
base - plus a goal. The layout is what `setup_scenario.py` teleports the
blocks into; the symbolic initial state is derived from it, so the two can
never drift apart.

The world holds six blocks, A through F, and the table is T. A scenario that
wants fewer says so by simply not naming them: `stowed` is the rest, which
setup parks on the far strip of the table and `WorldModel` is told to filter
out of perception by name.

That filter is not tidiness. Perception reads every block_* model in Gazebo,
so without it a spare block would join the state of every scenario - and
`sussman` would stop being the Sussman anomaly, because with a spare cube
about, C no longer has to go down on the table and the putdown the scenario
exists to force is not forced. A three-block problem has to stay a three-block
problem even when five cubes are on the table.
"""

from . import domain
from .geometry import block_z, stow_xy

# What the scene actually spawns. blocks.world has A-C, blocks5.world A-E and
# blocks6.world A-F; this names the largest of them, which is the one
# demo.launch loads. A scenario naming anything outside it can be planned but
# not run.
WORLD_BLOCKS = ("A", "B", "C", "D", "E", "F")


class Scenario(object):
    def __init__(self, name, description, towers, goal, goal_text):
        self.name = name
        self.description = description
        self.towers = towers          # [(["A", "C"], (x, y)), ...] bottom-first
        self.goal = goal
        self.goal_text = goal_text

    @property
    def initial(self):
        return domain.state_from_towers([blocks for blocks, _ in self.towers])

    @property
    def blocks(self):
        """The blocks this scenario is about."""
        return {b for blocks, _ in self.towers for b in blocks}

    @property
    def stowed(self):
        """World blocks it is not about, parked out of the way and unperceived."""
        return tuple(b for b in WORLD_BLOCKS if b not in self.blocks)

    @property
    def runnable(self):
        """Can the real scene supply the blocks this needs?"""
        return self.blocks <= set(WORLD_BLOCKS)

    @property
    def table_xy(self):
        """Where each tower's base block sits - the anchor for all metric reasoning."""
        return {blocks[0]: xy for blocks, xy in self.towers}

    def block_poses(self):
        """(x, y, z) for every block in the *world*, for teleporting the scene.

        The stowed ones are in here too, and have to be: setup that placed only
        the blocks a scenario names would leave the others wherever the last
        run dropped them, which for a block that ended up mid-table is a
        collision waiting to happen.
        """
        poses = {}
        for blocks, (x, y) in self.towers:
            for level, block in enumerate(blocks):
                poses[block] = (x, y, block_z(level))
        for index, block in enumerate(self.stowed):
            x, y = stow_xy(index)
            poses[block] = (x, y, block_z(0))
        return poses


def tallest_tower(state):
    """Height in blocks of the tallest tower in `state`."""
    return max((len(t) for t in domain.towers_of(state)), default=0)


def any_tower_of(n):
    """Goal: a tower n blocks high, anywhere it fits.

    No fixed set of atoms says this - `{on(A,B), on(B,C)}` names a specific
    tower - so it goes in as a callable. This is the goal that lets the robot
    choose its own base, which is the point of the `far_base` scenario.
    """
    def goal(state):
        return domain.HANDFREE in state and tallest_tower(state) >= n
    return goal


GOAL_ABC = [domain.on("A", "B"), domain.on("B", "C")]

# The five-block goal, bottom-first: E on the table, then D, C, B, A.
GOAL_EDCBA = [domain.on("A", "B"), domain.on("B", "C"), domain.on("C", "D"),
              domain.on("D", "E"), domain.on("E", domain.TABLE)]

# The six-block one, the same idea one block taller.
GOAL_FEDCBA = [domain.on("A", "B"), domain.on("B", "C"), domain.on("C", "D"),
               domain.on("D", "E"), domain.on("E", "F"),
               domain.on("F", domain.TABLE)]


SCENARIOS = {
    "model": Scenario(
        "model",
        "s0 from the model: A, B, C side by side on the table.",
        [(["A"], (0.45, -0.20)), (["B"], (0.45, 0.20)), (["C"], (0.62, 0.00))],
        GOAL_ABC,
        "{on(A,B), on(B,C)}"),

    "sussman": Scenario(
        "sussman",
        "The Sussman anomaly: C on A, with A and B on the table. Needs C "
        "moved out of the way first, so it forces a putdown - stack(C,T) - "
        "which no run of the original demo ever performs.",
        [(["A", "C"], (0.45, -0.20)), (["B"], (0.45, 0.20))],
        GOAL_ABC,
        "{on(A,B), on(B,C)}"),

    "inverted": Scenario(
        "inverted",
        "The finished tower, upside down: A on B on C, goal C on B on A. "
        "Every block has to move and the table is the only buffer.",
        [(["C", "B", "A"], (0.50, 0.00))],
        [domain.on("C", "B"), domain.on("B", "A")],
        "{on(C,B), on(B,A)}"),

    "far_base": Scenario(
        "far_base",
        "C parked at x = 0.70, past where the arm can service the top of a "
        "three-high tower. The goal names no base, so the planner has to "
        "pick one it can actually reach.",
        [(["A"], (0.45, -0.20)), (["B"], (0.45, 0.20)), (["C"], (0.70, 0.00))],
        any_tower_of(3),
        "any three-block tower"),

    "four_blocks": Scenario(
        "four_blocks",
        "Four blocks side by side, and a goal that names no tower. A "
        "four-high tower does not fit under the reach envelope at x = 0.62, "
        "so the filter forces the near side of the table.",
        [(["A"], (0.45, -0.20)), (["B"], (0.45, 0.20)),
         (["C"], (0.62, 0.00)), (["D"], (0.55, -0.28))],
        any_tower_of(4),
        "any four-block tower"),

    "five_blocks": Scenario(
        "five_blocks",
        "All five blocks, none of them where the goal wants it: B on D on A, "
        "and E on C. The goal tower reads E, D, C, B, A from the table up, so "
        "every block moves - and B, which comes off first, has nowhere to go "
        "but the table, because the support it belongs on is not built yet. "
        "That second transfer is what makes this twelve actions rather than "
        "the ten that five moved blocks would suggest.",
        [(["A", "D", "B"], (0.45, -0.20)), (["C", "E"], (0.45, 0.20))],
        GOAL_EDCBA,
        "{on(A,B), on(B,C), on(C,D), on(D,E), on(E,T)}"),

    "six_blocks": Scenario(
        "six_blocks",
        "Two three-block towers, both stacked the wrong way up, merged into "
        "one six-high tower: C on B on A, F on E on D, goal F, E, D, C, B, A "
        "from the table up. Every block moves exactly once - twelve actions "
        "for six blocks, which is the floor - because unstacking the second "
        "tower onto the table builds the bottom of the goal tower as a side "
        "effect, and the first tower is then peeled onto it in order. The "
        "cost of that tidiness is height: the top block goes on at z = 0.6475 "
        "and only the near half of the table can service it.",
        [(["A", "B", "C"], (0.45, -0.20)), (["D", "E", "F"], (0.45, 0.20))],
        GOAL_FEDCBA,
        "{on(A,B), on(B,C), on(C,D), on(D,E), on(E,F), on(F,T)}"),
}

# Everything the real scene can supply blocks for. Derived rather than written
# out, so a scenario added with a sixth block is excluded by arithmetic instead
# of by somebody remembering to leave it off a list.
PHYSICAL = tuple(sorted(name for name, s in SCENARIOS.items() if s.runnable))
