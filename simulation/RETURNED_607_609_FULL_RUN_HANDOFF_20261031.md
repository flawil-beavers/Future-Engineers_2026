# Successful CW entry practice and return to full runs

607--609 were copied FIRST from USB and verified against source-before,
source-after and local SHA-256; immediate unplug notice sent. Complete unchanged
saved originals and full hashes are in
[evidence index](evidence/parking_exit_diagnostics/README.md). All three are CW
entry-only practice, revision20261031_rear_space, no overflow, completed:

| Run | Duration | Replans | Accepted rear X corrections | Longest logged search |
| --- | ---: | ---: | --- | ---: |
|607|24.168s|5|+20.76mm|7201ms|
|608|22.489s|4|+11.19mm|7122ms|
|609|20.481s|3|-3.42/+4.85mm|5604ms|

User reports that parking worked but requests more front steering space and
less rear space. Individual contacts were not specified by log number; do not
claim three independently measured contact-free runs. Rear corrections are
onboard reference estimates, not independent pose truth. Installed binary not
read back. This batch does not validate CCW or automatic post-lap localization.
Search times show that the optimized bounded search can still take5--7seconds
in difficult partial entries. No unrelated planner retuning in this follow-up.

Minimal change: preferred local axle X65->60mm, i.e. another5mm rearward.
At nominal247.5mm gap and165mm bounding length, preferred rear/front clearances
are20/62.5mm. At the smallest modeled232.5mm gap they are20/47.5mm. At2deg heading
the125mm-wide bounding footprint reduces those nominal end gaps by about2.2mm.
The entire steered body/wheel sweep remains checked, not just these straight
end dimensions. Completion seeks at least30mm modeled front reserve instead
of25mm; modeled rear minimum remains10mm. Positive entry margins, rear-reference
quality/fan gates, run-local radius/brake adaptation and speed remain unchanged.
These reserves depend on the existing physical geometry and pose estimate;
no new overhang measurement or guaranteed true clearance is asserted.

Restore STARTUP_ROBOT_MODE=MODE_OBSTACLE_CHALLENGE. Normal parking-lot start,
unparking, three laps and final parking now run after reset. The separate
park cw/park ccw commands remain available while drive disabled, with their
unchanged surveyed capture marks; reset returns to full challenge.
[Practice instructions](../docs/guides/PARKING_ENTRY_PRACTICE.md) updated accordingly.

Checks:full parking396/396 +5registration,tight36/36,entry-only4CW/CCW completions,
wall/rail shortcut1440/12168,603replans2,604retreat27+3negative,rearreference
6accepted/36rejected+2front-space controls pass. Adaptive350/350 +8health pass.
Host models do not establish physical success of this additional5mm change.
Final IDE-managed M7 build and binary hash are recorded in AGENT_DOCUMENTATION.md.
M4 unchanged; no upload, motion, commit or push.

Next:upload M7 only and place robot at the normal original parking-lot start,
not at isolated-practice capture marks. Normal CW/CCW full runs and normal saves;
secure originals FIRST and notify. Record final front/rear room, steering/contact,
rear corrections and search_ms. No extra ToF sweep or sensor pauses requested.

Final M7 SUCCESS:RAM453712/523624,flash555728/786432,binary559280bytes,
SHA256 `a50cd4611b70318ebd941abd81e5eb563fbb713c2171b9dbaba7c1fd93121d91`. Parking revision20261031_front_swing_space.
