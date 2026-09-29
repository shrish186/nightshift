# Watchman hardware BOM (December 2026 pilots)

**Target: ≤ ₹15,000 per machine node, installed.** Prices are Indian retail ranges from
memory and **every line is [verify]**. Put real quotes in `PLAN.md` → Cost tracker (vendor,
date, link) before ordering. See `docs/watchman-architecture.md` for how the parts are used.

## Machine node (one per machine)

| # | Part | Suggested | Qty | ₹ each (range) | Notes |
|---|---|---|---|---|---|
| 1 | MCU | ESP32-S3-DevKitC-1 (N8R8) | 1 | 1,000–1,400 [verify] | Wi-Fi, FPU, I2S; 8 MB PSRAM for the audio buffer |
| 2 | Current ADC | ADS1115 16-bit module | 1 | 250–400 [verify] | Up to 860 SPS, differential input; I2C |
| 3 | Current clamp | SCT-013-030 (30 A, 1 V out) | 1 | 600–900 [verify] | Split core on one spindle-motor phase. **VERIFY** the phase current range per machine; use SCT-013-050/100 for bigger spindles. Clamp on the drive *input* phase if the motor side is inaccessible |
| 4 | Accelerometer | ADXL345 module (SPI, 3.2 kHz ODR) | 1 | 200–350 [verify] | Baseline. **Upgrade:** ADXL355 (low noise, ~₹4–5k) if M1 data shows ADXL345 noise hides tool events |
| 5 | Accel mount | M6 stud / magnetic base + potted housing | 1 | 200–400 [verify] | Rigid mount near the spindle nose; sealed against coolant |
| 6 | Microphone | INMP441 I2S MEMS | 1 | 200–350 [verify] | Behind a coolant-proof acoustic membrane vent |
| 7 | Feed-hold relay | Opto-isolated DIN-rail interface relay (e.g. Omron G2RV class, 24 V coil) | 1 | 800–1,200 [verify] | **Energised = hold** (dead node → machine keeps running). **VERIFY** the feed-hold input, polarity and contact rating per machine |
| 8 | Relay driver | Logic-level MOSFET or driver board | 1 | 100–200 [verify] | If the relay coil isn't driven directly |
| 9 | Power | 24 V → 5 V DIN-rail PSU (e.g. Mean Well HDR-15-5) | 1 | 900–1,300 [verify] | Fed from the machine's 24 V control supply, fused |
| 10 | Enclosure | IP65 ABS box with DIN rail | 1 | 400–800 [verify] | Mounted outside the machining area |
| 11 | Button + LED | Illuminated push button (PAUSE/RESET), status LED | 1 | 150–300 [verify] | The **only** way to clear a latched hold |
| 12 | Cabling | Shielded cable, glands, GX16/M12 connectors, ferrules, terminal blocks, fuse | 1 lot | 800–1,500 [verify] | Shielded runs for the accelerometer and clamp |
| 13 | Board | Perfboard (pilot) → custom 2-layer PCB (after pilots) | 1 | 150–600 [verify] | |
| 14 | Install | Mounting, wiring, commissioning (~2 h) | 1 | 1,000–2,000 [verify] | Founder-installed for pilots |
| | **Total** | | | **≈ 6,950–11,850** | Under the ₹15k target, with margin for the ADXL355 upgrade |

## Shop hub (one per shop)

| # | Part | Suggested | Qty | ₹ each (range) | Notes |
|---|---|---|---|---|---|
| 1 | Computer | Raspberry Pi 5, 4 GB | 1 | 6,000–7,000 [verify] | |
| 2 | Power | Official 27 W USB-C PSU | 1 | 900–1,200 [verify] | |
| 3 | Storage | NVMe HAT + 256 GB SSD (or 64 GB high-endurance microSD) | 1 | 3,000–4,000 (800) [verify] | SSD strongly preferred for the telemetry queue |
| 4 | Case | Case with active cooling | 1 | 600–1,200 [verify] | Shop floors are hot and dusty |
| 5 | UPS | Mini DC UPS (12 V / 5 V) | 1 | 1,500–2,500 [verify] | Rides through power cuts; clean shutdown |
| 6 | Uplink | 4G router with SIM (if shop Wi-Fi is unreliable) | 0–1 | 1,500–3,500 + ~₹300/month data [verify] | Outbound only; no port forwarding |
| | **Total** | | | **≈ 12,500–19,400** | Once per shop, not per machine |

## Recording kit (makerspace sessions)

The same node hardware in **record mode**, plus a laptop running `tools/record`, and a USB
cable. Buy two node kits early (October) so recording can start before the pilots.

## Open hardware questions (for the hardware cofounder)

1. Spindle current ranges on the pilot machines, to choose the SCT-013 variant.
2. Feed-hold input type and polarity on each pilot controller (Fanuc `*SP`, Siemens,
   Mitsubishi, local brands), and whether a relay contact can be wired in without voiding
   warranty or service contracts.
3. ADXL345 vs ADXL355: decide from the first makerspace recordings.
4. Coolant and chip protection for the accelerometer and microphone.
5. Mains noise on the ADS1115 current signal near VFD-driven spindles (shielding, filtering).
