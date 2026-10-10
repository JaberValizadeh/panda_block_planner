"""The blocks-world state-transition system: E, A and gamma.

This is the model written down directly, in the notation it was specified in:

    s0 = {on(A,T), on(B,T), on(C,T), handfree}
    s1 = {on(A,T), on(C,T), holding(B)}
    ...
    a0 = pickup(B),  a1 = stack(B,C),  ...
    r  = (s0, a0, s1, a1, s2, a2, s3, a3, s4)

`T` is the table: the support that is always there, always has room, and is
never picked up. That one convention is what lets two operators cover four
textbook ones - `stack(x,T)` *is* putdown, and `pickup(x,C)` *is* unstack.

The model writes the table as D. It is T here because D is a block: the scene
grew to five of them, and one symbol cannot be both a block and the table.

Nothing in this module imports ROS. The whole point is that the reasoning can
be developed and tested without Gazebo, which takes a minute to start and
several to run a task through.
"""

TABLE = "T"

HANDFREE = ("handfree",)


def on(x, y):
    return ("on", x, y)


def holding(x):
    return ("holding", x)


def is_clear(state, x):
    """clear(x) is derived, not stored.

    Under the closed-world assumption a block is clear exactly when nothing
    rests on it and it is not in the hand, so storing it would only be a
    second copy of what `on` already says - and a second thing to get wrong.
    The table is clear by definition: it always has room for one more block.
    """
    if x == TABLE:
        return True
    if holding(x) in state:
        return False
    return not any(a[0] == "on" and a[2] == x for a in state)


def support_of(state, x):
    """What x is resting on, or None if it is held (or absent)."""
    for a in state:
        if a[0] == "on" and a[1] == x:
            return a[2]
    return None


def height_of(state, x):
    """How many blocks are underneath x. A block on the table is at height 0."""
    n = 0
    support = support_of(state, x)
    while support is not None and support != TABLE:
        n += 1
        support = support_of(state, support)
    return n


def blocks_in(state):
    """Every block named anywhere in the state, table excluded."""
    names = set()
    for a in state:
        if a[0] == "on":
            names.add(a[1])
            if a[2] != TABLE:
                names.add(a[2])
        elif a[0] == "holding":
            names.add(a[1])
    return names


# --- Actions --------------------------------------------------------------

class Action(object):
    """A ground action.

    `pickup` is written with one argument in the model - pickup(B), not
    pickup(B,T) - and prints that way here. The source is still carried
    internally because the *effect* needs it: taking B off C makes C clear,
    taking B off the table makes nothing clear. It is recoverable from the
    state anyway, since `on` is functional (a block rests on exactly one
    thing), so the one-argument name is unambiguous - just not self-contained.
    """

    __slots__ = ("kind", "block", "support")

    def __init__(self, kind, block, support):
        self.kind = kind
        self.block = block
        self.support = support

    def __str__(self):
        if self.kind == "pickup":
            return "pickup(%s)" % self.block
        return "stack(%s,%s)" % (self.block, self.support)

    def __repr__(self):
        return "%s(%s,%s)" % (self.kind, self.block, self.support)

    def __eq__(self, other):
        return (isinstance(other, Action)
                and (self.kind, self.block, self.support)
                == (other.kind, other.block, other.support))

    def __hash__(self):
        return hash((self.kind, self.block, self.support))

    def verbose(self):
        """Spelled out, for logs: 'pickup(B) [off T]' / 'stack(B,C)'."""
        if self.kind == "pickup":
            return "pickup(%s) [off %s]" % (self.block, self.support)
        return str(self)


def pickup(x, y):
    return Action("pickup", x, y)


def stack(x, y):
    return Action("stack", x, y)


# --- gamma ----------------------------------------------------------------
#
#   pickup(x, y)                        stack(x, y)
#     pre:  on(x,y), clear(x),            pre:  holding(x), clear(y)
#           handfree                      del:  holding(x), clear(y)
#     del:  on(x,y), clear(x), handfree   add:  on(x,y), clear(x), handfree
#     add:  holding(x), clear(y)
#
# The clear() lines are implicit here because clear is derived - deleting
# on(x,y) is what makes y clear.

def applicable(state, action):
    if action.block == action.support:
        return False
    if action.kind == "pickup":
        return (on(action.block, action.support) in state
                and is_clear(state, action.block)
                and HANDFREE in state)
    if action.kind == "stack":
        return (holding(action.block) in state
                and is_clear(state, action.support))
    raise ValueError("unknown action kind: %r" % action.kind)


def apply_action(state, action):
    """gamma(s, a) -> s'. Raises if a is not applicable in s."""
    if not applicable(state, action):
        raise ValueError("%s is not applicable in %s" % (action, format_state(state)))
    s = set(state)
    if action.kind == "pickup":
        s.discard(on(action.block, action.support))
        s.discard(HANDFREE)
        s.add(holding(action.block))
    else:
        s.discard(holding(action.block))
        s.add(on(action.block, action.support))
        s.add(HANDFREE)
    return frozenset(s)


def ground_actions(blocks):
    """Every ground action over `blocks`. 3 blocks -> 18 of them."""
    supports = list(blocks) + [TABLE]
    out = []
    for x in blocks:
        for y in supports:
            if x == y:
                continue
            out.append(pickup(x, y))
            out.append(stack(x, y))
    return out


def successors(state):
    """(action, next_state) for every action applicable in `state`."""
    for action in ground_actions(sorted(blocks_in(state))):
        if applicable(state, action):
            yield action, apply_action(state, action)


def reachable_states(state):
    """Every state reachable from `state`. For 3 blocks this is all 22 of E."""
    seen = {state}
    frontier = [state]
    while frontier:
        current = frontier.pop()
        for _, nxt in successors(current):
            if nxt not in seen:
                seen.add(nxt)
                frontier.append(nxt)
    return seen


# --- states ---------------------------------------------------------------

def state_from_towers(towers):
    """Build a state from towers listed bottom-first.

        state_from_towers([["A"], ["B"], ["C"]])  -> s0
        state_from_towers([["A", "C"], ["B"]])    -> C on A, A and B on table
    """
    atoms = {HANDFREE}
    for tower in towers:
        support = TABLE
        for block in tower:
            atoms.add(on(block, support))
            support = block
    return frozenset(atoms)


def towers_of(state):
    """The inverse: towers bottom-first, sorted by their base block."""
    bases = sorted(a[1] for a in state if a[0] == "on" and a[2] == TABLE)
    towers = []
    for base in bases:
        tower = [base]
        while True:
            above = [a[1] for a in state if a[0] == "on" and a[2] == tower[-1]]
            if not above:
                break
            tower.append(above[0])
        towers.append(tower)
    return towers


def check_state(state):
    """Return a list of problems with `state`, empty if it is well-formed.

    Perception can hand us nonsense - two blocks read as resting on the same
    one, a block that is both held and supported - and it is much easier to
    debug that here than three actions into a plan.
    """
    problems = []
    held = [a[1] for a in state if a[0] == "holding"]
    if len(held) > 1:
        problems.append("holding more than one block: %s" % ", ".join(sorted(held)))
    if held and HANDFREE in state:
        problems.append("holding(%s) together with handfree" % held[0])
    if not held and HANDFREE not in state:
        problems.append("neither holding(x) nor handfree")

    for block in sorted(blocks_in(state)):
        supports = [a[2] for a in state if a[0] == "on" and a[1] == block]
        if len(supports) > 1:
            problems.append("%s rests on several things: %s"
                            % (block, ", ".join(sorted(supports))))
        if block in held and supports:
            problems.append("%s is held but also on %s" % (block, supports[0]))
        if not supports and block not in held:
            problems.append("%s is neither on anything nor held" % block)

    for support in sorted(blocks_in(state)):
        resting = [a[1] for a in state if a[0] == "on" and a[2] == support]
        if len(resting) > 1:
            problems.append("%s has several blocks on it: %s"
                            % (support, ", ".join(sorted(resting))))

    # Follow each block down to the table; a cycle never gets there.
    for block in sorted(blocks_in(state)):
        seen = {block}
        support = support_of(state, block)
        while support is not None and support != TABLE:
            if support in seen:
                problems.append("support cycle through %s" % block)
                break
            seen.add(support)
            support = support_of(state, support)
    return problems


def format_state(state):
    """'{on(A,T), on(B,T), on(C,T), handfree}' - the notation from the model."""
    def key(atom):
        return (atom[0] != "on", atom)
    parts = []
    for atom in sorted(state, key=key):
        if atom[0] == "on":
            parts.append("on(%s,%s)" % (atom[1], atom[2]))
        elif atom[0] == "holding":
            parts.append("holding(%s)" % atom[1])
        else:
            parts.append("handfree")
    return "{" + ", ".join(parts) + "}"
