# ExecPlans

An ExecPlan is a self-contained, living specification for work too large for a single pass. An agent should be able to read only the plan and the current repository, then finish the work.

## When a plan is required

Write an ExecPlan in `plans/planned/` before implementation when any of these holds:

- The work will likely take more than an hour or spans several modules.
- It changes simulation semantics, spatial-query results or default settings.
- It changes dependencies or the toolchain.
- It needs measurement (benchmarks, profiling) or research before the approach is known.

Small, low-risk fixes (a typo, a contained bug with an obvious fix and a test) need no plan.

## Required sections

Use `plans/TEMPLATE.md`. Every plan has these sections, in this order:

1. **Purpose**: why the work matters and what someone can observe when it is done.
2. **Progress**: checkboxes with UTC timestamps. Always reflects reality; partial items say what is done and what remains.
3. **Surprises & Discoveries**: unexpected behavior or failed assumptions, with short evidence.
4. **Decision Log**: `Decision / Rationale / Date` entries for each important choice, including rejected alternatives.
5. **Assumptions and Open Questions**: each assumption with why it is reasonable, how to validate it and what to do if wrong.
6. **Outcomes & Retrospective**: delivered results, remaining gaps, lessons. Filled at milestones and completion.
7. **Context and Orientation**: the relevant code in plain language, with repository-relative paths and function names.
8. **Plan of Work**: the sequence of edits and why.
9. **Concrete Steps**: exact commands from the repository root and their expected short output.
10. **Validation and Acceptance**: observable behavior and passing tests, not "it compiles".
11. **Idempotence and Recovery**: how to rerun safely and how to roll back.
12. **Artifacts and Notes**: short transcripts or snippets that prove progress. Large logs stay outside the repository; note their path.
13. **Interfaces and Dependencies**: new or changed functions, types, resources and crates.

## Rules

- Name files `YYYY-MM-DD-short-name.md`, one initiative per file. State near the top that the plan follows `PLANS.md`.
- Keep Progress, Surprises & Discoveries, Decision Log and Outcomes & Retrospective current at every milestone, pause and handoff.
- Prefer the order: correctness, reversibility, consistency with existing code, simplicity, performance. Record it in the Decision Log when a lower priority wins.
- Bug fixes come with a regression test where practical. Behavior or workflow changes update `README.md` in the same change.
- Measurements state the scenario, build profile, seed and machine; a number without them is not evidence.
- Verification follows the resource rules in `AGENTS.md`.

## Lifecycle

- Open plans live in `plans/planned/`. Move a plan to `plans/completed/` when it is implemented, superseded or cancelled.
- Before moving a plan, close or explicitly defer every Progress item, with a reason and where the deferred work is tracked, and summarize residual risk in Outcomes & Retrospective.
- A superseded plan names its replacement.
- Update `plans/STATUS.md` whenever a plan is created, moved or reclassified.
- When revising a plan materially, append a revision note at the bottom: what changed, why, date.
