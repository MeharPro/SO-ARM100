# Robot Arm Parts Audit

Date checked: 2026-02-28

## Confirmed
- SO-101 leader gearing remains 3x C046, 2x C044, 1x C001 from the upstream repo.
- Standard follower is 6x C001; your upgraded follower remains 6x 12V STS3215 on a single 12V rail.
- LeRobot setup requires one USB cable and a serial 3-pin cable path per arm, with board jumpers on B-channel for Waveshare-style boards during setup.
- The workbook formulas and cached totals are valid and display correctly.

## Corrections Applied
- Replaced the leader PSU row with the repo's exact Amazon.ca ASIN B087LY41PV at live CAD pricing.
- Updated workbook notes to flag camera compatibility and servo accessory uncertainty instead of treating them as fully verified.
- Added an Audit sheet to the main workbook.

## Remaining Blockers
- The SO-101 STEP assembly includes 4x M2.5x4 screws and 4x M2.5 spacers for the controller-board mount per arm, but the board listing only advertises Bus Servo Adapter (A) x1. These are not sourced in the workbook yet.
- LeKiwi assembly explicitly calls for M2x5 tap screws on the wheel modules. Those are not sourced in the workbook yet.
- The current arm M3 screw row is not a verified pan-head Amazon.ca listing. The docs specify M3x6 and the STEP file names pan-head screws.
- The current Future sheet uses SO-101 wrist-camera modules, not a fully verified LeKiwi base-camera mount pairing.
- Upstream sources conflict on leader power: the repo BOM uses a 5V PSU, while the board listing describes 9-12.6V input.

## Current Displayed Totals
- Lowest: 572.11 CAD
- Sufficient: 617.39 CAD
- Great: 737.57 CAD
- Future: 1436.12 CAD

These totals exclude the unresolved hardware above.
