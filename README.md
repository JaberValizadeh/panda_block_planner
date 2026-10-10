# panda_block_planner

The blocks-world model — `E`, `A`, `γ`, and runs `r` — implemented, searched,
and driven onto the Panda in Gazebo.

`panda_block_stack` executes a run. This package *produces* one. It is given a
goal, not a sequence:

```python
SEQUENCE = [("B", "C"), ("A", "B")]      # panda_block_stack: the answer, written down
GOAL     = [on("A", "B"), on("B", "C")]  # here: the question
```

The scene is not duplicated. The world, the URDF with the grasp plugin and the
whole bring-up sequence belong to `panda_block_stack` and are included from it;
this package replaces only what drives the arm.

## The model

```
s0 = {on(A,T), on(B,T), on(C,T), handfree}
s1 = {on(A,T), on(C,T), holding(B)}
s2 = {on(A,T), on(B,C), on(C,T), handfree}
s3 = {on(B,C), on(C,T), holding(A)}
s4 = {on(A,B), on(B,C), on(C,T), handfree}

a0 = pickup(B)   a1 = stack(B,C)   a2 = pickup(A)   a3 = stack(A,B)

r: (s0, a0, s1, a1, s2, a2, s3, a3, s4)
```

`T` is the table: always present, always with room, never picked up. That one
convention lets two operators cover the four textbook ones — `stack(x,T)` *is*
putdown, `pickup(x,C)` *is* unstack.

> The model writes the table as `D`. It is `T` here because the scene grew to
> five blocks and D is now one of them; one symbol cannot be both. Nothing else
> about the notation changed.

```
pickup(x, y)                        stack(x, y)
  pre:  on(x,y), clear(x), handfree   pre:  holding(x), clear(y)
  del:  on(x,y), clear(x), handfree   del:  holding(x), clear(y)
  add:  holding(x), clear(y)          add:  on(x,y), clear(x), handfree
```

`clear` is derived rather than stored — under the closed-world assumption it is
exactly "nothing on it and not in the hand", so storing it would only be a
second copy of what `on` already says, and a second thing to get wrong.

**`|E| = 22`**, for three blocks and a table: 13 arrangements with the hand
free, plus 3 × 3 with a block held. `|A| = 18` ground actions. The test suite
asserts both by enumeration, and that the space is strongly connected.

The scene holds six blocks now. Five is `|E| = 866` — 501 arrangements with
the hand free, plus 5 × 73 with a block held — and six is `7057`, still
enumerated from `s0` in milliseconds. A scenario that only wants three says
so, and the rest are stowed; see *Five blocks, and stowing the rest*. A space
this small is still why the planner is breadth-first and there is no
heuristic:
optimal plans come back in microseconds, and every alternative would be
premature.

`pickup` prints with one argument, as in the model, but carries its source
internally — the *effect* needs it, since taking B off C makes C clear while
taking B off the table makes nothing clear.

## Where the model meets the robot

Three places the formalism does not survive contact with Gazebo. All three are
the reason this package is shaped the way it is.

**1. γ is not a function.** Executing `a0` in `s0` can yield `s1`, or leave the
world in `s0` because the grasp closed on nothing, or put the block somewhere
no state predicted. So `γ: E × A → 2^E`, and a plan cannot be a script. The
node executes the **first action only**, re-perceives, and re-plans. Searching
`E` costs microseconds; a wrong assumption about where a block is costs the
run.

**2. The abstraction is blind to metric drift.** A block that slips out of the
gripper and lands flat on the table is symbolically indistinguishable from one
that was never picked up — `on(B,T)` either way. Every observation therefore
carries both the symbolic state *and* the poses behind it, and placements are
checked against `PLACE_TOLERANCE` (22.5 mm) in metres, not in predicates.

**3. Some preconditions are not symbolic.** `stack(A,B)` onto a two-high tower
is textbook-legal and can still be unplannable, because the wrist ends up
outside what the arm can reach. `geometry.reach_filter` is passed to the search
as a `feasible(state, action)` hook, and a rejected edge is simply never
expanded — so backtracking falls out of BFS for free, and the planner reports
failure only when *every* branch is blocked.

**4. An observation has to wait for the world to stop.** `γ` is instantaneous;
Gazebo is not. A block released onto a tower bounces a few mm before it comes
to rest, and abstracting it mid-bounce reads a height it is only passing
through. `WorldModel.settle` therefore waits for every free block to be still
— and to *stay* still for `SETTLE_DWELL`, since a single quiet sample is not
proof of rest — before any observation is taken. The held block is excluded: it
jitters against the fingers for as long as it is carried.

This was found the hard way, via a much worse bug underneath it. `sussman`
died reproducibly at `stack(C,T)` with "no feasible plan", because C was being
perceived ~120 mm in the air and vanishing from the state. C was not misplaced;
it was still *welded to the palm*, riding up with the retreat, and was dropped
from that height a fraction of a second later. The cause was
`gazebo_grasp_fix`'s release latency, and the fix belongs in the URDF rather
than here — see `panda_block_stack/urdf/panda_grasp.urdf.xacro`. Settling is
the right thing to do regardless, but it is not what made this work: a settle
long enough to outlast a 1.5 s weld would only have hidden a robot that throws
its blocks.

## Scenarios

```bash
roslaunch panda_block_planner demo.launch scenario:=sussman
```

| scenario | start | goal | what it exercises |
|---|---|---|---|
| `model` | A, B, C side by side | `{on(A,B), on(B,C)}` | reproduces `r` exactly — 4 actions |
| `sussman` | C on A; A, B on the table | `{on(A,B), on(B,C)}` | needs the table as a buffer: 6 actions, including a `stack(C,T)` that no run of the original demo performs |
| `inverted` | A on B on C | `{on(C,B), on(B,A)}` | every block moves; the table is the only buffer |
| `far_base` | C at x = 0.70 | any three-block tower | the goal names no base, so the planner must pick one it can reach |
| `four_blocks` | A, B, C, D side by side | any four-block tower | a four-high tower does not fit under the reach envelope at x = 0.62, so the filter forces the near side |
| `five_blocks` | B on D on A; E on C | `{on(A,B), on(B,C), on(C,D), on(D,E), on(E,T)}` | all five blocks, a named five-high tower, and a block that has to be moved twice: 12 actions |
| `six_blocks` | C on B on A; F on E on D | `{on(A,B), …, on(E,F), on(F,T)}` | two inverted towers merged into one six-high tower; every block moves exactly once, and only the near slots can service the top |

The Sussman plan is worth looking at, because its tail is the hand-written run:

```
s0 = {on(A,T), on(B,T), on(C,A), handfree}
a0 = pickup(C) [off A]      s1 = {on(A,T), on(B,T), holding(C)}
a1 = stack(C,T)             s2 = {on(A,T), on(B,T), on(C,T), handfree}   <- the model's s0
a2 = pickup(B) [off T]      s3 = {on(A,T), on(C,T), holding(B)}
a3 = stack(B,C)             s4 = {on(A,T), on(B,C), on(C,T), handfree}
a4 = pickup(A) [off T]      s5 = {on(B,C), on(C,T), holding(A)}
a5 = stack(A,B)             s6 = {on(A,B), on(B,C), on(C,T), handfree}
```

It solves its way into `r` and then follows it.

## Five blocks, and stowing the rest

`five_blocks` is the long one: `B` on `D` on `A`, `E` on `C`, and a goal that
names the whole tower bottom-up as `E, D, C, B, A`. Nothing starts on the right
support, so every block moves — and `B`, which is on top at the start and so
comes off first, has nowhere to go. The support it belongs on is `C`, and `C`
will not be on `D` for another four actions. So `B` goes down on the table and
is fetched again later, and that one extra transfer is the difference between
the ten actions five moved blocks would suggest and the twelve the planner
returns:

```
a0  = pickup(B) [off D]     a1  = stack(B,T)      <- the second transfer starts here
a2  = pickup(E) [off C]     a3  = stack(E,T)
a4  = pickup(D) [off A]     a5  = stack(D,E)
a6  = pickup(C) [off T]     a7  = stack(C,D)
a8  = pickup(B) [off T]     a9  = stack(B,C)      <- ... and ends here
a10 = pickup(A) [off T]     a11 = stack(A,B)
```

A five-high tower is close to the ceiling: the top block goes on at z = 0.6025
and the wrist needs 60 mm above that, which `max_tcp_z` allows out to about
x = 0.55 and refuses at x = 0.62. The tower therefore gets built on a near-side
buffer slot, and asking for it at the far edge is reported as a reach failure
rather than attempted. Both are asserted in `test_planner.py`.

Setting that layout up is itself worth a look: two stacked towers teleported
into place is the first scenario here that reliably provokes ODE into shoving
one block off another on the first contact. `scene_setup` reads the result
back and places the layout again if anything moved — see pitfall 5.

## Six blocks, and measuring the ceiling

`six_blocks` starts from two three-block towers stacked the wrong way up — C
on B on A, F on E on D — and wants one six-high tower reading F, E, D, C, B, A
from the table up. It takes **12 actions, and twelve is the floor**: six blocks
all out of place cannot be done in fewer than six pickups and six stacks, and
unlike `five_blocks` nothing has to be moved twice. Unstacking the second tower
onto the table *is* the bottom of the goal tower, so the first tower can then
be peeled onto it in order:

```
a0  = pickup(F) [off E]    a1  = stack(F,T)      F down: the tower's base
a2  = pickup(E) [off D]    a3  = stack(E,F)
a4  = pickup(D) [off T]    a5  = stack(D,E)
a6  = pickup(C) [off B]    a7  = stack(C,D)
a8  = pickup(B) [off A]    a9  = stack(B,C)
a10 = pickup(A) [off T]    a11 = stack(A,B)
```

The price of that tidiness is height. The top block goes on at z = 0.6475, and
the wrist needs clearance above that, so only the near half of the table can
service it. That is what forced the reach envelope to stop being a guess —
see pitfall 2 — and `tools/measure_reach.py` is the sweep that replaced it.

**Stowing.** Gazebo spawns every block in the world whatever the scenario is
about, and
perception reads every `block_*` model it can see. Left alone, that would put
spare cubes into the state of every smaller scenario — and `sussman`
would stop being the Sussman anomaly, because `C` could be parked on a spare
block instead of on the table, and the putdown the whole scenario exists to
force would not be forced.

So a scenario names the blocks it is about, by using them, and the rest are
`scenario.stowed`. Setup teleports those to a strip at the back of the table
(`geometry.stow_xy`, x = 0.78 — past every slot a putdown can choose), and
`WorldModel(ignore=...)` drops them by name before abstraction. Both halves
have to hold, so both are tested in `test_scenarios.py`, including the negative
case: with the filter off, the spare block does appear in the state.

They are stowed *on* the table rather than beside it on purpose. A block below
the table top is how `world_model.abstract` recognises one that has fallen off,
which is fatal to a run — so if the name filter ever fails, a stowed block
should show up as a harmless extra, not as a crash.

## Running it

Load the Panda environment first, exactly as for `panda_block_stack`:

```bash
panda      # or: source ~/moveit_ws/setup_env.bash
```

### Without a robot

The reasoning needs neither ROS nor Gazebo, and this is how you should
actually work on it — a second, against a minute of bring-up:

```bash
./scripts/plan_only.py                  # every scenario
./scripts/plan_only.py sussman          # one
./scripts/plan_only.py far_base --naive # ... ignoring the reach envelope

PYTHONPATH=src python3 -m unittest discover -s test
```

### With the robot

All in one:

```bash
roslaunch panda_block_planner demo.launch scenario:=sussman
```

Or against a long-lived environment, re-running as often as you like:

```bash
roslaunch panda_block_stack environment.launch \
    world:=$(rospack find panda_block_stack)/worlds/blocks5.world    # terminal 1
roslaunch panda_block_planner plan_and_stack.launch scenario:=five_blocks  # terminal 2
```

The world matters: `blocks.world` has three blocks and `blocks5.world` has
five, and `four_blocks` and `five_blocks` need the second one. `demo.launch`
asks for it for every scenario, since the three-block ones stow what they do
not use. Both worlds live in `panda_block_stack` — the scene is still not
duplicated here.

`setup:=true` (the default) teleports the blocks into the scenario's layout
first. That happens *inside* the task node rather than as a second `<node>`
tag, because roslaunch does not order nodes and a task that plans while the
blocks are still being placed plans against the wrong state.

### Watching it recover

The point of re-planning every action is only visible when something goes
wrong, and a working simulator will not go wrong on demand. Injecting a
disturbance by hand —

```bash
rosservice call /gazebo/set_model_state '{model_state: {model_name: block_B,
  pose: {position: {x: 0.45, y: -0.30, z: 0.4225}, orientation: {w: 1.0}},
  reference_frame: world}}'
```

— works (the teleport sticks), but landing it *inside* the window that matters
is a race against the task, and two attempts here missed it: one fired late
because `roslaunch`'s output is block-buffered, so grepping the log for the
trigger lagged reality by seconds.

So the recovery path is tested against a fake world instead, in
`test/test_task.py`, where the fault is scripted rather than raced:
`stack(B,C)` lets go early, B lands on the table, and the loop is asserted to
notice, re-plan, re-grasp B from where it actually landed, and finish. A
second fault (`pickup(B)` that never takes effect) asserts it gives up after
`MAX_RETRIES` rather than grinding. `Task` takes its world and arm as
constructor arguments for exactly this reason.

Recording that trace correctly turned out to be subtle, and the fake caught a
real bug: the loop originally appended only *successful* actions, so a run
with a retry in it reported a sequence that was not a path through `E` at all.
Failed attempts are now kept and flagged — dropping them would describe a run
that never happened.

## Layout

| path | what it is |
|---|---|
| `src/panda_block_planner/domain.py` | `E`, `A`, `γ` — the model, with no ROS imports |
| `src/panda_block_planner/planner.py` | BFS over `E`, with the feasibility hook and run formatting |
| `src/panda_block_planner/geometry.py` | the metric half: reach envelope, stack heights, buffer slots |
| `src/panda_block_planner/scenarios.py` | named (layout, goal) pairs; layout and initial state derived from one source |
| `src/panda_block_planner/world_model.py` | `/gazebo/model_states` → a state in `E`, plus the poses behind it |
| `src/panda_block_planner/executor.py` | one action → arm motion (MoveIt + `franka_gripper` actions) |
| `src/panda_block_planner/task.py` | the perceive-plan-act-verify loop; world and arm injected so it can be faked |
| `src/panda_block_planner/scene_setup.py` | teleport blocks into a scenario layout, and check they stayed |
| `tools/measure_reach.py` | sweep IK over the table to measure the reach envelope; prints the table `geometry.py` carries |
| `scripts/plan_and_stack.py` | node wiring: pick a scenario, build the real world and arm, run the loop |
| `scripts/plan_only.py` | plan and print, no ROS |
| `scripts/setup_scenario.py` | set the scene up on its own |
| `test/` | 103 tests; only `test_task.py` and `test_scenarios.py` need `rospy` importable, and none need a simulator |

## Building

```bash
cd ~/moveit_ws
catkin build panda_block_planner --no-deps
```

`--no-deps` is no longer required: the MoveIt source checkout moved to
`~/ridgeback_ws` when the workspace was split, so a plain `catkin build` here
only builds the Panda demos and `gazebo-pkgs`. It remains harmless, and MoveIt
still comes from the `~/arm_underlay` binaries.

Re-source after building — `catkin_python_setup()` puts the module on
`PYTHONPATH` through `devel/setup.bash`.

## Things that will bite you

Everything in `panda_block_stack`'s list still applies — sourcing order, the
`.catkin` marker, `palm_link` having to be `panda_link7`,
`load_robot_description:=false`, `GAZEBO_RESOURCE_PATH`. Plus:

1. **Do not run both task nodes.** `demo.launch` includes
   `stack_blocks.launch` with `run_demo:=false` deliberately. Two nodes
   commanding one `move_group` interleave their trajectories and neither plan
   survives.

2. **The reach envelope is measured now, and the measurement changed the
   answer.** It used to be one point — `(0.62, 0.633)` unplannable,
   `(0.62, 0.603)` plannable — with a straight line drawn through it on the
   assumption that reach improves as the arm comes inboard. `tools/measure_reach.py`
   swept IK over the table instead, and the line was wrong in a way one point
   could never show: **reach depends on y as much as on x.** The old model
   claimed a flat 0.75 anywhere inboard of x = 0.50; the real ceiling at
   (0.48, 0.28) is 0.660. It was waving through placements 90 mm above what
   the arm can do, and only the fact that nothing had yet tried to build
   anything tall near the table edge kept that from biting.

   Out at (0.62, 0.00) the sweep agrees with the original point — 0.606,
   between the 0.603 that planned and the 0.633 that did not — which is the
   only independent check available.

   What to watch now: `max_tcp_z` interpolates over a 60 mm grid, and a convex
   surface interpolated across a gap reads low. That is the safe direction,
   but it means the filter refuses some poses the arm could make. Re-run the
   sweep with a finer `--x` if the table starts feeling smaller than it is.
   IK is also not planning: a pose with a solution can still be unplannable
   because of what is in the way, which is what the fallback ladder in
   `move_above` is for.

3. **The planner does not know where a putdown will land.** A block's `xy` is
   its tower base's `xy`, which is known — except for `stack(x,T)`, whose slot
   the executor chooses when it gets there. Planning uses the slot the
   executor *would* pick as a stand-in. It is an approximation, and it is
   survivable only because the loop re-plans against reality immediately
   afterwards.

4. **Buffer slots avoid the goal footprint on purpose.** Parking a block where
   the tower is about to go costs a second transfer to move it out again.
   `free_slot` takes an `avoid` list for exactly this, and a goal given as a
   callable (`any three-block tower`) has no footprint to avoid — which is
   correct, since the planner has not committed to a base yet.

5. **A teleport onto another block is not a placement.**
   `set_model_state` puts the two faces exactly in contact, and ODE resolves
   that first contact by pushing — usually by nothing, occasionally by enough
   to slide the upper block off. Measured on the five-block layout: the first
   pass after a fresh world load left E 54-71 mm off C and C 17 mm off its own
   target, while every later pass — and every retry — was exact to 0.1 mm. So
   `place_scenario` reads ground truth back, and repeats the whole layout if
   anything moved.

   It is worth repeating the *layout*, not the block: a block that slid off
   has usually nudged the one under it too, and a second pass over a scene
   that is already nearly right lands exactly. Three passes, then it gives up
   and says which block will not stay put — which is much easier to read than
   a run that quietly starts from a state nobody asked for.

6. **A block in the hand is not standing on the table.** Perception reports
   an xy for it like any other — wherever the gripper has carried it, which is
   directly over the tower being built as often as not. `Task.choose_slot`
   used to count that as occupied space, so it ruled out the nearest buffer
   slot and pushed the putdown outboard. That is why the five-block tower got
   built at x = 0.54 instead of 0.42, and it made `six_blocks` unplannable
   outright, since 0.54 is past the ceiling for a six-high tower. The fix is
   one clause; the lesson is that `Observation.poses` answers "where is it",
   not "what is occupied", and the two differ for exactly one block.

7. **A scenario that does not use every block must stow the rest.** The
   world spawns five whatever the problem is. `Scenario.stowed`,
   `scene_setup.place_scenario` and `WorldModel(ignore=...)` are three halves
   of one mechanism — a new scenario gets stowing for free by simply not
   naming a block, but a new *entry point* that builds a `WorldModel` without
   passing `ignore` will silently plan a different problem than the one on the
   table. `scripts/plan_and_stack.py` is the one place that wiring lives.

8. **Perception can hand you nonsense.** A block mid-topple reads as "a level
   up but resting on nothing"; two blocks can read as resting on the same one.
   `domain.check_state` catches these and `planner.plan` refuses a malformed
   initial state, because the alternative is a plan that fails three actions
   in for reasons that look like a motion bug.

## Status

103 tests pass: the model (`r` validated step by step, `|E| = 22` by
enumeration), the search (the Sussman plan, the five-block reversal, the
six-block merge, the reach filter rejecting an action the model permits), the
measured reach surface, the geometry, the stow mechanism, the buffer-slot
choice, and the loop's failure handling against a fake world.

Every scenario has been run headless against Gazebo and reached its goal, with
no adjustment to the perception tolerances or the buffer-slot geometry. The
first three are against the three-block world, as originally recorded:

```
model      4 actions   A on B on C at x=0.62      placements 1-8 mm from target
inverted   6 actions   tower rebuilt upside down  placements 7 mm
far_base   4 actions   built on B at x=0.45       placements 1-3 mm
```

`far_base` is the one worth watching: C sat at x = 0.70, the filter refused it
as a base, and the planner built on B instead and folded C in as the *top*
block — `pickup(A), stack(A,B), pickup(C), stack(C,A)`.

And against the five- and six-block worlds:

```
six_blocks  12 actions  F,E,D,C,B,A built at (0.42, 0.00)  placements 1-8 mm
five_blocks 12 actions  E,D,C,B,A built at (0.54, 0.00)    placements 2-6 mm
four_blocks  6 actions  B,A,C,D built on B at y=0.20       placements 6-8 mm
sussman      6 actions  stack(C,T) still forced            placements 4-8 mm
```

The lower three predate the slot fix in pitfall 6, which is why `five_blocks`
built at x = 0.54 rather than the x = 0.42 it now chooses — that run is still
a valid run, just not where it would land today. They have not been repeated
since.

`six_blocks` is the one that pushed hardest. Twelve actions, no retries, and
the finished tower measured z = 0.4225 / 0.4675 / 0.5125 / 0.5575 / 0.6025 /
0.6475 — six levels exactly one block apart, leaning about 10 mm over the full
height. `move_above` took its preferred 120 mm of clearance for every
placement but the last, where 0.6475 + 0.12 = 0.7675 is above the measured
ceiling of 0.760 at that slot, and dropped to the 90 mm rung: z = 0.737, with
22 mm to spare. No reach warnings, and setup placed both towers first time.

It also only worked because of two things fixed on the way there — the slot
bug in pitfall 6, which made it unplannable, and the reach sweep in pitfall 2,
without which the filter would have been happy to build it somewhere the arm
could not reach.

Four things those runs settle:

* **A five-high tower holds, and its top is reachable.** The finished tower
  measured z = 0.4225 / 0.4675 / 0.5125 / 0.5575 / 0.6025 — five levels exactly
  one block apart, with every placement 2-6 mm from where it was aimed against
  a 22.5 mm tolerance. Putting A on at 0.6025 is the highest the wrist has been
  asked to go here; `move_above` refused its preferred 120 mm of clearance at
  that reach and backed off to 90 mm, which is the fallback ladder doing its
  job rather than a near miss.
* **Stowing holds.** `sussman` against the five-block world perceived exactly
  `{on(A,T), on(B,T), on(C,A), handfree}` with D and E sitting on the same
  table, and still performed `stack(C,T)` — the putdown, not a park on a spare
  cube. D and E were at (0.780, -0.320) and (0.780, -0.160) before the run and
  at the same values to 0.1 mm after it.
* **A six-high tower is at the limit, and the limit is now known.** Nothing
  taller fits: a seven-high tower needs the TCP at 0.7525, and the highest the
  measured surface goes anywhere on the table is 0.780, directly in front of
  the base at x = 0.36 where no slot exists. Six is the last one.
* **Setup needed fixing, and the first run found it.** The first `five_blocks`
  attempt reported *success in ten actions*: E had been pushed off C during the
  teleport, so the robot correctly solved the easier problem it could see. The
  placement check in `scene_setup` (pitfall 5) is the fix, and the next run
  logged `pass 1: C (17 mm out), E (71 mm out) ... placing the layout again`,
  started from the right `s0`, and took the full twelve.

Still not verified: recovery from a real mid-run disturbance. The
path is covered by scripted-fault tests (see *Watching it recover*), but no
live failure has been observed, because none occurred on its own and the
injected ones raced the task.
