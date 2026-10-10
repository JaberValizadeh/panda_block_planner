"""Search over E for a run from s0 into the goal.

Breadth-first, and deliberately so. Three blocks plus a table give 22 states
and 18 ground actions - the entire reachable space fits in a set with room to
spare, so BFS returns an optimal-length plan instantly and there is nothing
for a heuristic to buy. Add a fourth block and it is 73 handfree states plus
the holding ones; still trivial. If this ever grows past ~6 blocks, replace
the frontier with A* over "blocks not yet in their goal position" rather than
reaching for an external PDDL solver.

The `feasible` hook is what keeps this honest. A symbolically applicable
action can still be geometrically impossible - stacking onto a two-block tower
at the far edge of the Panda's reach is the case that actually bites here -
and an edge the hook rejects is simply not expanded. Backtracking then falls
out of BFS for free: the search takes another branch, and only reports failure
if every branch is blocked.
"""

import collections

from . import domain


class Plan(object):
    """A run: (s0, a0, s1, a1, ..., sn), which is what the model calls r.

    `ok` is None for a plan the search produced - every step of one of those
    is a gamma transition by construction. An *executed* run is different: if
    the world declined to do what an action said, the step is still part of
    what happened and still belongs in the trace, but it is not an edge in E.
    Those steps are flagged rather than dropped, because a report that
    silently omitted them would describe a run that never occurred.
    """

    def __init__(self, states, actions, ok=None):
        self.states = states      # len(actions) + 1 of them
        self.actions = actions
        self.ok = ok

    def __len__(self):
        return len(self.actions)

    def __iter__(self):
        """Yield (state, action, next_state) for each step."""
        for i, action in enumerate(self.actions):
            yield self.states[i], action, self.states[i + 1]

    @property
    def initial(self):
        return self.states[0]

    @property
    def final(self):
        return self.states[-1]

    def format_run(self):
        """'r: (s0, a0, s1, ...)' plus the expansion of each symbol."""
        symbols = []
        lines = []
        for i, state in enumerate(self.states):
            symbols.append("s%d" % i)
            if i < len(self.actions):
                symbols.append("a%d" % i)
        lines.append("r: (%s)" % ", ".join(symbols))
        lines.append("")
        for i, state in enumerate(self.states):
            lines.append("  s%d = %s" % (i, domain.format_state(state)))
            if i < len(self.actions):
                note = ""
                if self.ok is not None and not self.ok[i]:
                    note = "   <- did not have its modelled effect"
                lines.append("  a%d = %s%s"
                             % (i, self.actions[i].verbose(), note))
        return "\n".join(lines)

    def failures(self):
        """Steps whose observed successor was not gamma(s, a)."""
        if self.ok is None:
            return []
        return [(i, self.actions[i]) for i in range(len(self.actions))
                if not self.ok[i]]


def satisfies(state, goal):
    """A goal is a *partial* state - a set of atoms that must all hold.

    Also accepts a callable for goals that no fixed atom set can express,
    such as "any three-block tower, wherever it fits".
    """
    if callable(goal):
        return goal(state)
    return set(goal) <= set(state)


def plan(initial, goal, feasible=None, max_actions=32):
    """Shortest sequence of actions from `initial` to a state satisfying `goal`.

    Returns a Plan, or None if the goal is unreachable - which, with a
    `feasible` filter in play, genuinely happens and is not a bug.

    `feasible(state, action) -> bool` is consulted before an edge is expanded.
    """
    problems = domain.check_state(initial)
    if problems:
        raise ValueError("initial state is malformed: " + "; ".join(problems))

    if satisfies(initial, goal):
        return Plan([initial], [])

    seen = {initial}
    # (state, actions_so_far, states_so_far)
    frontier = collections.deque([(initial, [], [initial])])
    while frontier:
        state, actions, states = frontier.popleft()
        if len(actions) >= max_actions:
            continue
        for action, nxt in domain.successors(state):
            if nxt in seen:
                continue
            if feasible is not None and not feasible(state, action):
                continue
            seen.add(nxt)
            path = actions + [action]
            trace = states + [nxt]
            if satisfies(nxt, goal):
                return Plan(trace, path)
            frontier.append((nxt, path, trace))
    return None


def explain_failure(initial, goal, feasible):
    """Why did planning fail - no route at all, or only a geometric block?

    Re-runs the search without the feasibility filter. If a plan exists then,
    the task is symbolically solvable and it is the robot's reach that is in
    the way, which is worth saying out loud rather than reporting a bare
    'no plan found'.
    """
    if feasible is None:
        return "no sequence of actions reaches the goal"
    relaxed = plan(initial, goal, feasible=None)
    if relaxed is None:
        return "no sequence of actions reaches the goal, even ignoring reach"
    return ("a %d-action plan exists symbolically (%s) but every branch is "
            "blocked by the reach envelope"
            % (len(relaxed), ", ".join(str(a) for a in relaxed.actions)))
