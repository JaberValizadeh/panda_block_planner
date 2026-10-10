"""The metric half of the world: where the symbolic state actually is in space.

`domain` knows that A is on B. It does not know that the top of that tower is
0.5125 m up and 0.62 m out, which is past the point where the Panda can still
plan a hover pose. This module holds everything metric - constants shared with
the Gazebo world, the reach envelope, the arithmetic that turns a symbolic
state into block positions, and the choice of where to park a block that has
to go on the table.

Constants must match panda_block_stack/worlds/blocks.world.
"""

from . import domain

BLOCK = 0.045                 # cube edge [m]
TABLE_TOP = 0.40              # table surface height [m]
BLOCK_Z = TABLE_TOP + BLOCK / 2.0   # centre height of a block on the table

TABLE_SIZE = (0.60, 0.90, 0.40)
TABLE_CENTER = (0.55, 0.0, 0.20)

# A placed block must end up this close to where it was aimed, measured
# against Gazebo ground truth rather than the arm's own account of itself.
# Half a block: enough slack for settle and roll, tight enough that a dropped
# block can never pass.
PLACE_TOLERANCE = 0.0225      # [m]

APPROACH = 0.12               # preferred hover clearance above a block
APPROACH_FALLBACKS = (APPROACH, 0.09, 0.06)
MIN_CLEARANCE = APPROACH_FALLBACKS[-1]

# The reach envelope, measured rather than assumed.
#
# tools/measure_reach.py asks MoveIt for IK at the only orientation the
# executor ever commands (DOWNWARD_RPY) over a grid on the table, and reports
# the highest z that still solves. These numbers are that sweep's output, at a
# 10 mm resolution in z with four IK seeds per pose.
#
# This replaces one measured point and a straight line drawn through it, and
# the line was wrong in a way the point could not show: reach depends on y as
# much as on x. The old model said a flat 0.75 anywhere inboard of x = 0.50,
# and at (0.48, 0.28) the real ceiling is 0.660 - it was waving through
# placements 90 mm above what the arm can do. Out at (0.62, 0.00) it agrees:
# the sweep puts the ceiling at 0.606, and the original note recorded 0.603 as
# planning and 0.633 as not.
#
# None means no IK solution at any height, i.e. off the end of the arm.
REACH_X = (0.36, 0.42, 0.48, 0.54, 0.60, 0.66, 0.72)
REACH_Y = (-0.28, -0.20, -0.12, 0.00, 0.12, 0.20, 0.28)
REACH_Z = (
    (0.740, 0.760, 0.780, 0.780, 0.780, 0.760, 0.740),   # x = 0.36
    (0.700, 0.730, 0.750, 0.760, 0.750, 0.730, 0.700),   # x = 0.42
    (0.660, 0.690, 0.710, 0.720, 0.710, 0.690, 0.660),   # x = 0.48
    (0.610, 0.650, 0.670, 0.680, 0.670, 0.650, 0.610),   # x = 0.54
    (0.550, 0.590, 0.610, 0.620, 0.610, 0.590, 0.550),   # x = 0.60
    (0.470, 0.510, 0.540, 0.550, 0.540, 0.510, 0.470),   # x = 0.66
    (None,  None,  0.440, 0.460, 0.440, None,  None),    # x = 0.72
)

UNREACHABLE = 0.0             # below the table, so every placement is refused


def _span(grid, v, clamp_low):
    """(i, t) with v = grid[i] + t*(grid[i+1] - grid[i]).

    t leaves [0, 1] past the far end of the grid on purpose: the samples are
    still falling there, so extrapolating along the last pair keeps rejecting
    poses further out. `clamp_low` pins the near end instead, for x, where
    extrapolating inboard would raise the ceiling on the strength of nothing.
    """
    if v <= grid[0]:
        return 0, (0.0 if clamp_low else (v - grid[0]) / (grid[1] - grid[0]))
    if v >= grid[-2]:
        i = len(grid) - 2
    else:
        i = max(j for j in range(len(grid) - 1) if grid[j] <= v)
    return i, (v - grid[i]) / (grid[i + 1] - grid[i])


def max_tcp_z(x, y=0.0):
    """Highest TCP z with an IK solution above (x, y), by interpolation.

    `y` defaults to 0, the centre line and the most generous row, so a caller
    that has only an x gets the optimistic answer. Every caller that knows its
    y should pass it - see `reach_filter` and `Executor.move_above`.
    """
    i, tx = _span(REACH_X, x, clamp_low=True)
    j, ty = _span(REACH_Y, y, clamp_low=False)
    weights = (((1 - tx) * (1 - ty), REACH_Z[i][j]),
               ((1 - tx) * ty,       REACH_Z[i][j + 1]),
               (tx * (1 - ty),       REACH_Z[i + 1][j]),
               (tx * ty,             REACH_Z[i + 1][j + 1]))
    # A corner off the end of the arm refuses the whole query - interpolating
    # across a None would invent a height for a pose with no solution at all.
    # Only corners that actually carry weight count, though: a query sitting
    # exactly on a good sample must not be refused because the cell next to it
    # happens to run off the arm, and it lands on a grid line from either
    # side, so letting that matter would make the surface asymmetric.
    if any(z is None and w != 0.0 for w, z in weights):
        return UNREACHABLE
    return sum(w * z for w, z in weights if w != 0.0)


def block_z(height):
    """Centre height of a block sitting `height` blocks above the table."""
    return BLOCK_Z + height * BLOCK


def place_z_on(support_z):
    """Where the centre of a block goes when stacked on a block at `support_z`."""
    return support_z + BLOCK


def base_of(state, block):
    """The table-resting block at the bottom of `block`'s tower."""
    current = block
    while True:
        support = domain.support_of(state, current)
        if support is None or support == domain.TABLE:
            return current
        current = support


def xy_of(state, block, table_xy, buffer_xy):
    """Planar position of `block` in `state`.

    Stacking never moves the block underneath, so a block's xy is its tower
    base's xy, and `table_xy` (from perception) supplies those. The one thing
    that cannot be known at planning time is where a block put *down* will
    end up - the executor picks that slot when it gets there - so `buffer_xy`
    stands in for it. That is an approximation, and it is why the executor
    re-perceives and re-plans after every action instead of trusting the plan
    it started with.
    """
    if domain.holding(block) in state:
        return None
    base = base_of(state, block)
    return table_xy.get(base, buffer_xy)


def reach_filter(table_xy, buffer_xy):
    """A `feasible(state, action)` hook enforcing the reach envelope.

    Both operators put the TCP above a block with some clearance: pickup above
    the block it takes, stack above the tower it adds to. If even the smallest
    usable clearance would put the wrist past what plans at that reach, the
    action is rejected rather than attempted. Attempting it is not merely slow
    (~10 s to time out) - it was twice seen to jostle the carried block loose.
    """
    def feasible(state, action):
        if action.kind == "pickup":
            xy = xy_of(state, action.block, table_xy, buffer_xy)
            z = block_z(domain.height_of(state, action.block))
        else:
            if action.support == domain.TABLE:
                xy, z = buffer_xy, BLOCK_Z
            else:
                xy = xy_of(state, action.support, table_xy, buffer_xy)
                z = place_z_on(block_z(domain.height_of(state, action.support)))
        if xy is None:
            return True
        return z + MIN_CLEARANCE <= max_tcp_z(*xy)
    return feasible


# --- picking a spot on the table -----------------------------------------

# Candidate slots are a grid over the reachable part of the table top, not the
# whole 0.60 x 0.90 surface: the far and near edges are outside a comfortable
# top-down grasp even though blocks would sit there happily.
SLOT_X = (0.42, 0.48, 0.54, 0.60)
SLOT_Y = (-0.28, -0.20, -0.12, 0.0, 0.12, 0.20, 0.28)
SLOT_CLEARANCE = 0.09         # keep this far from any other block centre [m]


def free_slot(occupied, avoid=(), clearance=SLOT_CLEARANCE):
    """Choose an empty (x, y) on the table for a block that must be put down.

    `occupied` are the xy of blocks already on the table; `avoid` are places
    nothing should be left, which in practice means the footprint of the tower
    about to be built. Parking a block where the tower is going costs a second
    transfer to move it out again, so it is worth the check.

    Slots are scored by distance from the robot base - nearer is easier to
    reach and easier to plan - and the first that clears everything wins.
    """
    blocked = list(occupied) + list(avoid)
    best = None
    for x in SLOT_X:
        for y in SLOT_Y:
            if any((x - bx) ** 2 + (y - by) ** 2 < clearance ** 2
                   for bx, by in blocked):
                continue
            cost = x ** 2 + y ** 2
            if best is None or cost < best[0]:
                best = (cost, (x, y))
    if best is None:
        raise RuntimeError("no free slot on the table for a putdown "
                           "(occupied: %s)" % (blocked,))
    return best[1]


def distance(a, b):
    return sum((p - q) ** 2 for p, q in zip(a, b)) ** 0.5


# --- parking the blocks a scenario does not use ---------------------------

# The world holds five blocks; a three-block scenario has to put the other two
# somewhere. They go on the far strip of the table, past SLOT_X and past every
# pose the arm is ever sent to, and perception is told to ignore them by name.
#
# On the table rather than on the floor beside it, deliberately: a block below
# the table top is how `world_model.abstract` recognises one that has fallen
# off, which is fatal to a run. If the name filter ever failed, a stowed block
# should show up as a harmless extra on the table, not as a crash.
STOW_X = 0.78                 # table top spans x in [0.25, 0.85]
# One y per block in the world, so the largest possible stow still fits.
STOW_Y = (-0.32, -0.19, -0.06, 0.06, 0.19, 0.32)


def stow_xy(index):
    """Where the `index`-th unused block is parked."""
    return (STOW_X, STOW_Y[index])
