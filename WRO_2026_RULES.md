# WRO 2026 Future Engineers rules reference

## Canonical sources and verification

- [Official season page](https://wro-association.org/competition/2026-season/).
- [General and Game Rules](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf), January 15, 2026, 55 pages.
- [Documentation Rubric](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Documentation-Rubric.pdf).
- [Modified Traffic Sign Surprise Rule](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf), November 9, 2026.
- [Official game mat](https://wro-association.org/wp-content/uploads/WRO-2026_FutureEngineers_Playfield.pdf).
- [Official Questions and Answers](https://wro-association.org/competition/questions-answers/), part of the international rules and able to clarify or override the PDF.

The recorded General Rules SHA-256 is
`c23ba063c548e74812ffd9ed580931b2d4264541261b5f8eaff643a1e3201043`.
Follow AGENTS.md for downloading, caching and verifying official PDFs.
National competitions may adapt the rules; use the applicable Swiss organizer's
rules and recheck official Q&A before competition.

| Verification date | Result |
| --- | --- |
| 2026-11-22 | Checked the official season page, Documentation Rubric and Future Engineers Q&A for documentation requirements and clarifications. |
| 2026-11-19 | Season page/November9 Surprise PDF/Q&A rechecked; first encountered sign excluded, original passing colour on all3laps, no newer Surprise clarification found. |
| 2026-11-17 | Season page still links the November 9 Surprise Rule. Its full text and score sheet were read; Q&A has no additional Surprise Rule clarification. |
| 2026-11-11 | General Rules remain the January 15 edition. First-encountered-sign exclusion confirmed for the Surprise Rule. |
| 2026-11-08 | General Rules version/hash and Q&A checked for inner enclosure geometry. |
| 2026-10-18 and 2026-10-27 | General Rules and Q&A checked for parking-row restrictions and passing-side preservation. |

Earlier checks, page inspections and engineering context remain in the
[preserved reference history](docs/history/WRO_2026_RULES.md).

## Modified Traffic Sign Surprise Rule

Participation is optional and declared before each attempt, with no change
after starting. Exactly one sign changes to the opposite colour and a height
of 50 or 150 mm instead of 100 mm. Its position and 50 x 50 mm base stay fixed.
The first sign encountered in the required direction is excluded; a later sign
in the starting straight remains eligible. Judges record the original colour
and position; selected position and modified height are shared by participating
teams in the same official round. Non-participants retain the original sign.

Pass according to the original colour on all three laps: modified green is
passed right, modified red left. Other signs keep their normal rules.
Incorrect passage ends the attempt under the existing rules. Each correct
passage earns 5 points, up to 15, retained even if all laps are not completed.
Repairing halves the total including the bonus. The three-minute limit remains.
Maximum Obstacle Challenge and overall scores are 77 and 137 respectively.

## Passing, layout and lap completion

Rules 9.16 and 9.19 retain one direction and RED-right/GREEN-left for all
three laps; no mandatory turn follows lap 2. Rule 9.19 and Appendix A.5
(pages 18 and 39-40) assess passage when the complete vehicle crosses the
sign's transverse line. A sign already behind the start need not be approached
just to pass it initially; a new wrong-side crossing has no such exception.

Figure 8c (page 15) has middle-station signs only as solitary signs and pairs
at the longitudinal ends. Inferring empty ends from a middle sign, or an empty
middle from both ends, assumes this official layout and does not establish
camera evidence on an arbitrary practice field. Figure 8e (page 16) places
parking-section signs in the inner row. The October 16 Q&A excludes its three
outer-row positions at the International Final; confirmed signs on a different
practice layout must still be respected. Route shortcuts retain passing sides.

Scoring item 1.2 (page 21) completes a lap once the whole vehicle leaves the
last corner, without another full start-straight traversal. Rule 9.22 requires
the three-lap return; 9.24.4 ends Obstacle on a stop in the finish section or
parking lot. Appendix A (page 41) relaxes passing sides on travel to parking
after three laps. Q&A allows opposite-direction final parking.
The project's 250 mm handover localization reserve is an engineering assumption,
not a rule or independent pose measurement.

## Field dimensions and visualization assumptions

Rule 13.1 (page 26) specifies a 3000 x 3000 mm track. Page 11 gives 1000 or
600 mm corridors, with +/-100 mm tolerance at the International Final.
Rule 13.16 permits square or rectangular inner walls. Appendix B (page 43)
lists 1000, 1400 and 1800 mm segments and thickness-adjusted short pieces.
The filmed MVI_1318 enclosure is consistent with 1000 x 1800 mm, giving
600 mm end corridors and 1000 mm side corridors. This is a filmed-layout
interpretation; Open layouts vary with the draw.

The official mat PDF has 5 mm bleed per side and a 3200 mm square TrimBox.
Inspector trims the artwork and separately draws nominal walls and logged
objects. Its source hash is in tools/robot-run-inspector/assets/fe-2026-mat.json.
