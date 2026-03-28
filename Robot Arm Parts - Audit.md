# Robot Arm Parts Audit

Date checked: 2026-03-01

## Current State
- The workbook is now kit-only.
- Granular BOM rows were removed.
- The sheet now presents just two purchase options:
  - `Arm Kit`
  - `Complete Mobile Robot Kit`
- Totals are formula-backed again, so changing quantity or unit price will recalculate totals when the workbook is opened in Excel or Google Sheets.

## Included Kits
- `Arm Kit`
  - Exact PartaBot `SO-ARM101` variant from the user-provided link:
    - `Electronics only / Unassembled`
    - Variant ID `43074328920179`
  - PartaBot product text says this includes:
    - `1` leader arm
    - `1` follower arm
    - `6x 12V` follower motors
    - `6x 7.4V` leader motors
    - both power supplies
    - `2x` motor control boards
    - `2x` USB-C cables
    - `4x` table clamps
  - This variant excludes `3D printed parts`

- `Complete Mobile Robot Kit`
  - Exact PartaBot `LeKiwi` default unassembled variant:
    - `Unassembled / None`
    - Variant ID `42948144791667`
  - PartaBot product text says the full kit includes:
    - all `3D printed parts`
    - electronics
    - power supply
    - hardware
    - integrated `SO-ARM101`

## Pricing
- PartaBot storefront prices are listed in `USD`
- Workbook `CAD` values use `Bank of Canada FXUSDCAD` for `2026-02-27`
- Exchange rate used: `1.3642`

## Current Workbook Prices
- `Arm Kit`
  - `299.00 USD`
  - `407.90 CAD`
  - `460.93 CAD` including HST

- `Complete Mobile Robot Kit`
  - `699.00 USD`
  - `953.58 CAD`
  - `1077.55 CAD` including HST

## Notes
- The `SO-ARM101` row still satisfies the stronger follower requirement because the PartaBot product description explicitly says the follower side uses `6x 12V` motors.
- The `LeKiwi` storefront also exposes a `+12V motors` option of `None` or `2`, but the user-provided link did not pin that option, so the workbook uses the default unassembled variant.
- I have not committed these changes.
