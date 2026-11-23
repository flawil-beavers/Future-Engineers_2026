# Engineering history archive

Snapshots preserved during the documentation consolidation on 2026-11-17.
Use the root [current handoff](../../AGENT_DOCUMENTATION.md) and
[active checklist](../validation/OBSTACLE_CHALLENGE_TEST_PLAN.md) for ongoing work.
Historical "current", "next", test flags and upload requests refer to the
build discussed in that entry. They do not authorize a new test sequence.

| Archive | Contents |
| --- | --- |
| [Agent documentation](AGENT_DOCUMENTATION.md) | Complete prior engineering history, including original firmware hashes, physical reports, limitations and rejected approaches. |
| [Obstacle test plan](OBSTACLE_CHALLENGE_TEST_PLAN.md) | Superseded ordered checklists and batch-specific validation procedures. |
| [Connector guide](CONNECTOR_NEXT_TEST.md) | Previous connector candidates, first-lap trials and acceptance procedures. |
| [Rules reference](WRO_2026_RULES.md) | Original dated verification notes and page-inspection context. |
| [Camera clock development](CAMERA_24MHZ_DEVELOPMENT.md) and [asynchronous image capture](CAMERA_ASYNC_BUFFERING.md) | Earlier clock, frame timing and memory experiments. |
| [Parking entry handoff](PARKING_ENTRY_HANDOFF.md) | Early parking design notes and validation plans from September and October. |
| [Alternate parking bay proposal](SURPRISE_CHALLENGE_PREPARATION.md) | Earlier concept for an alternate bay. |

Search the archived agent document by run number, revision or date. Its entries
are not consistently chronological. Paths inside code spans are relative to
the repository root. The snapshots preserve original wording; the obstacle
plan's one relative Markdown link was adjusted for its new directory.

For current investigations, prefer the detailed review reports linked from
the handoff. The newest firmware reports cover the November 11 Surprise Rule,
first-lap retry, parking recheck and checked CCW view changes. Earlier reports
describe their own firmware and must not be used as acceptance of later builds.

Original logs, image approval status and evidence metadata stay in
[simulation/evidence](../../simulation/evidence/parking_exit_diagnostics/README.md).
This archive contains documentation, not replacement telemetry.

- November 20 parking entry before the fixed-only change. M7 `20261120_ccw_rail_approach`, SHA-256 `adf7e6a302815692d5bb7caef6c5f125d3f1fe2cbf71ec7deaae411810da8bee`, preferred the four-stroke normal entry and used measured-radius replanning or a checked diagonal retrace where necessary. On that version 396 full approach/entry scenarios and 38 measured entry variants passed in the host fixture, including 36 cases which allowed replanning. The earlier build with SHA-256 `a1a165ce1e56b9a008aa9a5a57d690444a9aad807b882a2b388db698389c9771` used the same adaptive fallback before the progressive CCW approach. Tests that call adaptive planning internals or legacy entry segments apply to those historical implementations. The current fixture verifies the fixed-only sequence. [Current handoff](../../AGENT_DOCUMENTATION.md)

- [November19 height-camera intake and pre-correction replay](HEIGHT_CAPTURE_INTAKE_20261119.md): original acquisition/approval history and initial failure evidence. Current correction/validation is in the active handoff and measured-height review.

- [November 19 handoff before CW logs 3–6](AGENT_DOCUMENTATION_20261119_BEFORE_CW_BATCH.md) preserves the prior active summary, build identities, saved USB and camera evidence, and limitations. Its links follow the current directory layout. Current status is in the root handoff and linked CW review.

- November 20 superseded later-lap clearance experiments. The `wall_room` build used a 195 mm isolated extreme offset (M7 SHA-256 `2e0d15eb3031ff73639d295543438711b744a3a9edaa95162006ed089a476270`). The `pillar_clearance` build also used a 205 mm normal end offset (SHA-256 `892e2711a6e27b9f21a437e302bc4e3b94302a7dd8ef055a04756928113f727a`). The two 2,809-case polygon checks gave minima of 55.20 and 53.35 mm respectively. There was no physical acceptance of either offset change. Both were reverted at the user's clarification. The faster CW scout was retained. The historical log 570 view fixture still gave −27.5° bearing and no selected extension at its first perturbed pose, including when replayed with the preceding geometry. [Current handoff](../../AGENT_DOCUMENTATION.md)
