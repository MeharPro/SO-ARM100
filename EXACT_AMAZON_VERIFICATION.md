# Exact Amazon Verification for SO-101 + LeKiwi Hybrid Build

Date checked: 2026-02-28
Scope: SO-101 leader arm, SO-101 follower arm (12V follower), LeKiwi base for follower, vision add-ons.

## Repo-verified BOM facts
- Leader arm motors are 6 total: 3x C046, 2x C044, 1x C001. Source: `/Users/meharkhanna/robot-arm/README.md:86`, `/Users/meharkhanna/robot-arm/README.md:87`
- Two-arm SO-101 BOM totals 7x C001, 2x C044, 3x C046. Source: `/Users/meharkhanna/robot-arm/README.md:76`, `/Users/meharkhanna/robot-arm/README.md:77`, `/Users/meharkhanna/robot-arm/README.md:78`
- Follower-only SO-101 arm uses 6x C001 in the standard 7.4V BOM. Source: `/Users/meharkhanna/robot-arm/README.md:89`, `/Users/meharkhanna/robot-arm/README.md:93`
- 12V follower upgrade requires 12V 5A+ power and leader stays 7.4V. Source: `/Users/meharkhanna/robot-arm/README.md:70`
- Repo exact Amazon links exist for the bus-servo board, USB cable, power supplies, clamp, screwdriver, and optional cameras. Source: `/Users/meharkhanna/robot-arm/README.md:79`, `/Users/meharkhanna/robot-arm/README.md:80`, `/Users/meharkhanna/robot-arm/README.md:81`, `/Users/meharkhanna/robot-arm/README.md:82`, `/Users/meharkhanna/robot-arm/README.md:83`, `/Users/meharkhanna/robot-arm/Optional/Wrist_Cam_Mount_32x32_UVC_Module/README.md:20`, `/Users/meharkhanna/robot-arm/Optional/Overhead_Cam_Mount_Webcam/README.md:22`
- LeKiwi exact Amazon links exist for screw assortment, battery, lever connector, DC plug connector, converter, USB hub, USB-C extension, and microSD. Source: https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md

## Exact Amazon.ca rows I could verify
| Section | Product | Qty | Exact Amazon.ca URL | Live price now (CAD) | Notes |
|---|---|---:|---|---:|---|
| Leader | Motor Control Board (ST/SC serial bus compatible) | 1 | https://www.amazon.ca/dp/B0CTMM4LWK | 54.68 | Live CAD price recovered from Amazon.ca indexed result |
| Leader | USB-C Cable 2 pcs / USB Data Cable | 1 | https://www.amazon.ca/dp/B0B8NWLLW2 | 11.99 | Exact ASIN from SO-101 repo |
| Leader | 5V Power Supply (5.5x2.1mm) | 1 | https://www.amazon.ca/dp/B087LY41PV | 20.49 | Exact ASIN from SO-101 repo |
| Follower | Motor Control Board (ST/SC serial bus compatible) | 1 | https://www.amazon.ca/dp/B0CTMM4LWK | 54.68 | Same board as leader |
| Follower | USB-C Cable 2 pcs / USB Data Cable | 1 | https://www.amazon.ca/dp/B0B8NWLLW2 | 11.99 | Exact ASIN from SO-101 repo |
| Follower | 12V 5A+ Power Supply (5.5x2.1mm) | 1 | https://www.amazon.ca/dp/B01GEA8PQA | 18.99 | Exact 12V supply ASIN from LeKiwi 12V arm BOM |
| Base for Follower | 12V 5A Li-ion Battery Pack | 1 | https://www.amazon.ca/dp/B0C242DYT1 | 61.87 | Exact ASIN from LeKiwi BOM |
| Base for Follower | Lever Wire Connector | 1 | https://www.amazon.ca/dp/B06XGYXVXR | 11.29 | Exact ASIN from LeKiwi BOM |
| Base for Follower | 12V 5A DC Plug Connector Set | 1 | https://www.amazon.ca/dp/B072BXB2Y8 | 8.99 | Exact ASIN from LeKiwi BOM |
| Base for Follower | 12V to 5V 5A USB-C Converter | 1 | https://www.amazon.ca/dp/B0CRVW7N2J | 13.99 | Exact ASIN from LeKiwi BOM |
| Base for Follower | microSD Card | 1 | https://www.amazon.ca/dp/B09X7C7LL1 | 22.97 | Exact ASIN from LeKiwi BOM |
| Base for Follower | M2/M3/M4 Assorted Screw Set | 1 | https://www.amazon.ca/dp/B0BMQGJP3F | 15.99 | Exact ASIN from LeKiwi BOM |
| Base for Follower | USB Hub | 1 | https://www.amazon.ca/dp/B0BR3M8XHK | 26.98 | Exact ASIN from LeKiwi wired BOM |
| Base for Follower | USB-C Extension Cable | 1 | https://www.amazon.ca/dp/B0BHHK1W95 | 22.09 | Exact ASIN from LeKiwi wired BOM |
| Base for Follower | USB-C to DC Cable | 1 | https://www.amazon.ca/dp/B0B9G1KFL3 | 17.99 | Exact ASIN from LeKiwi wired/5V BOM |
| Other | Table Clamp (4pcs) | 1 | https://www.amazon.ca/dp/B0DJNXF8WH | 19.99 | Exact ASIN from SO-101 repo |
| Other | Screwdriver Set | 1 | https://www.amazon.ca/dp/B0DB227RTH | 8.98 | Exact ASIN from SO-101 repo |
| Other | Wide-Angle USB Wrist Camera | 2 | https://www.amazon.ca/dp/B0CNCSFQC1 | 51.47 | Exact ASIN from optional wrist camera docs |
| Other | Overhead Webcam | 1 | https://www.amazon.ca/dp/B082X91MPP | 43.49 | Exact ASIN from optional webcam mount docs |

## Blockers: exact Amazon listing not verified
| Section | Product | Qty | Status | Why blocked |
|---|---|---:|---|---|
| Leader | STS3215 Servo 7.4V, 1/147 gear (C046) | 3 | Not exact on Amazon | SO-101 repo points this exact SKU to Alibaba/Taobao/Akizuki, not Amazon. I did not find an exact Amazon listing for C046. |
| Leader | STS3215 Servo 7.4V, 1/191 gear (C044) | 2 | Not exact on Amazon | Same issue. Exact C044 listing did not surface on Amazon. |
| Leader | STS3215 Servo 7.4V, 1/345 gear (C001) | 1 | Not exact on Amazon | Same issue. Exact C001 listing did not surface on Amazon. |
| Follower | STS3215 Servo 12V, 1/345 gear | 6 | Not exact on Amazon | Exact 12V STS3215 follower servo is sourced in the repo from Alibaba, not Amazon. |
| Base for Follower | 4in Omni Wheels | 3 | Not exact on Amazon | LeKiwi BOM points to VEX Robotics exact wheel listing, not Amazon. |
| Base for Follower | STS3215 Servo 12V (base drive) | 3 | Not exact on Amazon | LeKiwi BOM points to Alibaba exact listing, not Amazon. |
| Base for Follower | Raspberry Pi 5 (4GB) | 1 | Amazon.ca price found, exact ASIN not locked | Indexed Amazon.ca result showed a live CAD listing, but I was not able to recover a defensible exact product URL/ASIN from Amazon.ca. |
| Other | Gripper Tape | 1 | Optional; exact Amazon row not locked | Easy to source, but not present in repo BOM and I did not lock a single exact ASIN before writing this file. |

## Bottom line
The current Amazon-only workbook is not exact end to end.

What is exact right now:
- The BOM quantities from the SO-101 repo are exact.
- The Amazon.ca rows above with explicit `/dp/ASIN` links and CAD prices are exact enough to use.

What is not exact right now:
- The servo rows.
- The omni wheel row.
- The Raspberry Pi 5 row.
- The gripper tape row.

If you want a fully exact purchase sheet, one of these must change:
1. Allow non-Amazon exact links for the servo and omni-wheel rows.
2. Keep Amazon-only, but accept blocker rows where no exact Amazon listing exists.
