# Room priors: which cloud model to ask (2026-11-23)

The record behind `PLAN-ros-alignment.md` 3.190 step B, criterion 7, and
step G: which cloud model should answer the find's opening question, "in
which room types is X usually found?". The shareable version is the
artifact "Room-prior models"
(https://claude.ai/artifact/C8bZtqHV8BoiPmoV3J2wbA, private to its owner). The full method, bars and amendments are in
`docs/plans/ros-alignment/3.190-prior-at-start.md` (amendments 5-8); the
replies and scores in `evaluations/room-prior-3190/b/` and `b7/`.

## The question

At the start of a find, the brain can ask a cloud model where the target
usually lives (a prior over room types) and use it to pick its first goals.
The default is Claude Opus 5.5. Criterion 7 asks whether another model does
better: it replaces the default only if it meets criteria 4-6 on the same
40 pinned targets AND beats Opus on criterion 1's simulator arm (paired
reward over 588 ProcTHOR houses, 95% interval above 0). Price is weighed
too.

## Results

| Model | Valid (of 40, bar 38) | Right room (of 20, bar 18) | Informative (of 40, bar 30) | Found (of 588) | Mean reward | vs no hint | vs Opus | $ a call |
|---|---|---|---|---|---|---|---|---|
| Opus 5.5 (default) | 40 | 20 | 39 | 574 | 0.524 | +0.101 [+0.055, +0.149] | -- | $0.0018 |
| Fable 5.1 | 40 | 20 | 40 | 576 | 0.495 | +0.071 [+0.025, +0.119] | **-0.030 [-0.053, -0.007]** | $0.0046 |
| Sonnet 5 | running (3.190.G) | | | | | | | |
| No hint | -- | -- | -- | 575 | 0.423 | -- | -- | $0 |
| Sonnet 5.5 | not reachable: not yet on the account's Bedrock (one pilot call answered, the next refused) | | | | | | | |
| Fable 5.2 | not reachable: not yet on the account's Bedrock (one pilot call answered, the next refused) | | | | | | | |

Arms on one commit (`56d44ef`); the brackets are 95% intervals of the
paired reward difference over houses.

## What it says

* **Every model reached helps.** A prior from Opus or Fable 5.1 finds
  targets with less driving than no prior (reward +0.10 and +0.07).
* **Opus 5.5 stays the default.** Fable 5.1 answers the 40 targets as well
  as Opus does, but its priors steer the simulator robot measurably worse
  (the interval is wholly below 0), and it costs 2.5x a call. It loses on
  plunger, basketball, garbage can, laundry hamper and house plant (its
  rooms are less sharply ordered: garbage can kitchen 0.93 vs bathroom
  0.85); it gains on box and garbage bag.
* **Answer quality on the 40 targets does not separate the models;** the
  simulator arm does. Criteria 4-6 are a floor, not a ranking.
* **Caveat:** the 40 targets and their expected rooms were written by a
  Claude session and are scored against Claude models.

## Cost

The whole of 3.190 B spent $0.31 of its $2 cap through Fable 5.1's run
(Opus's 40 calls $0.074; Fable 5.1's 49 $0.227; two refused-model pilots
$0.005). Sonnet 5's run is capped at $0.50.
