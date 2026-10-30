# <Short, action-oriented title>

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

What changes for someone running the simulation, and how to observe it.

## Progress

- [ ] (YYYY-MM-DD HH:MMZ) Plan drafted.
- [ ] First milestone.
- [ ] Validation gate run.

## Surprises & Discoveries

- Observation: ...
  Evidence: ...

## Decision Log

- Decision: ...
  Rationale: ...
  Date: ...

## Assumptions and Open Questions

- Assumption: ...
  Why reasonable: ...
  Validation: ...
  If wrong: ...

## Outcomes & Retrospective

Delivered results, remaining gaps and lessons.

## Context and Orientation

Relevant modules, functions and paths.

## Plan of Work

Sequence of edits and why.

## Concrete Steps

From the repository root, sequentially:

1. `CARGO_BUILD_JOBS=4 cargo test --release --locked`

## Validation and Acceptance

Observable behavior and tests that must pass.

## Idempotence and Recovery

How to rerun safely and roll back.

## Artifacts and Notes

Short evidence; paths to large logs outside the repository.

## Interfaces and Dependencies

New or changed functions, types, resources and crates.

---

Revision notes:

- YYYY-MM-DD: Initial plan.
