# SO-101 Leader + Follower (12V Follower Upgrade) BOM

Location target: Milton, Ontario, Canada  
Currency: CAD  
Snapshot date: 2026-02-15

## 1) Exact Required Parts (Spec-Accurate)

| Section | Part category | Exact spec / variant | Qty required |
|---|---|---|---:|
| Leader | Servo | STS3215 7.4V `C046` (1/147) | 3 |
| Leader | Servo | STS3215 7.4V `C044` (1/191) | 2 |
| Leader | Servo | STS3215 7.4V `C001` (1/345) | 1 |
| Follower (12V) | Servo | STS3215 **12V** high-torque (follower chain must be all 12V) | 6 |
| Shared | Motor control board | ST/SC serial bus adapter board compatible with STS3215 | 2 |
| Leader | Power | 5V class DC supply (5.5x2.1mm) | 1 |
| Follower (12V) | Power | **12V, 5A+ minimum** DC supply (5.5x2.1mm) | 1 |
| Shared | USB data cable | USB cable for each control board to host | 2 |
| Shared | Fasteners | M2x6 + M3x6 + horn screws (if not included in servo packs) | 1 kit |

## 2) Live Listing Prices I Was Able To Verify

| Section | Item | Exact AliExpress URL | Unit price (CAD) | Shipping to Milton (CAD) | Tax (CAD, est. 13%) | Line total (CAD) | Notes |
|---|---|---|---:|---:|---:|---:|---|
| Shared | SO-ARM101 Arm Motor Kit (contains mixed leader ratios + boards/cables per seller listing) | https://www.aliexpress.com/item/1005008883199466.html | 305.01 | 51.04 | 46.29 | 402.34 | Price verified for `Arm Motor Kit` SKU. Shipping returned from live listing response for default-selected SKU context; checkout may vary by SKU/route. |
| Follower (12V) | STS3215 12V 30KG 6PCS set | https://www.aliexpress.com/item/1005011566463861.html | 153.55 | 0.00 | 19.96 | 173.51 | Listing title and SKU indicate 6PCS 12V high-torque set. |

### Resolved subtotal (verified lines only)

- Subtotal: `CAD 458.56`
- Shipping: `CAD 51.04`
- Tax (estimated): `CAD 66.25`
- Current total: `CAD 575.85`

## 3) Required But Not Fully Resolved In This Run

| Section | Required item | Status |
|---|---|---|
| Follower (12V) | 12V 5A+ power supply (5.5x2.1mm) | Required, but exact cheapest verified AliExpress line was blocked by anti-bot challenge during this session. |
| Shared | USB cables / clamps / screwdriver / screw assortment | Optional/recommended lines not fully price-verified in this session due anti-bot challenge. |

## 4) Electrical Verification Checklist

- [x] Leader motor distribution is `C046x3 + C044x2 + C001x1` (7.4V).
- [x] Follower upgrade uses `6x 12V` servos on follower chain.
- [x] No 7.4V + 12V mixing on the same powered follower daisy-chain rail.
- [x] Leader and follower are separate arms with separate controller boards and separate power supplies.
- [x] Follower supply requirement is 12V with 5A+ margin.
- [x] STS3215 uses serial bus protocol; controller must support ST/SC bus servos.

## 5) Source References

- SO-ARM100 repository BOM and notes: https://github.com/TheRobotStudio/SO-ARM100/tree/main
- LeRobot SO-101 guide: https://huggingface.co/docs/lerobot/so101

