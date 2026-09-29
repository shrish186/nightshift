# NightShift — Plan (draft v0, 2026-09-29)

> Status: first draft. Every number marked **[verify]** is an assumption we have not checked yet.
> Owners: Shrish (backend/cloud/AI/business), CS cofounder (controller/vision/watchman), HW cofounder (arm/gripper/fixtures/guarding).
> Team is based in Amherst, MA; pilots and installs happen in India. Pilots via founders' contacts at large factories.
> Decision (2026-09-29): **we are building the arm + watchman together, done properly, and raising funding for it.** The watchman ships first as the safety/trust layer the arm depends on.
> Decision (2026-09-29, later): **watchman-first for December shop pilots.** The watchman ships standalone (node per machine + shop hub + owner app), **alert-only by default**, auto-stop opt-in per machine. The arm/cell code is frozen as-is (M0 done, sim only) until January. Architecture: `docs/watchman-architecture.md`; hardware: `hardware/bom.md`.

---

## 1. Problem & customer

**Customer:** small/medium Indian job shops (5–40 CNC machines) in clusters like Pune (Bhosari/Chakan), Coimbatore, Rajkot, Bengaluru (Peenya), Ludhiana, Chennai (Ambattur). They make repeat parts for auto/tractor/pump/valve OEMs and Tier-1s.

**Pilot customers (first):** large factories run by founders' contacts. Good fit for the arm: high-volume repeat parts, fewer changeovers, budget, and a maintenance team who can help. Risk: they may already run 2–3 shifts, so the pitch there is labor/consistency/scrap, not "idle night hours" **[verify per factory]**.

**Pain:**
- Most run 1–1.5 shifts. Expensive machines (₹15–60 lakh each **[verify]**) sit idle ~12 hrs/night.
- Night shifts are hard to staff and supervise; operators run 1 machine each doing mostly load/unload.
- Running unattended is scary: a broken tool or chip jam at 2am can scrap parts, crash the spindle, or start a fire.
- Owners won't buy a ₹20–40 lakh automation cell with uncertain payback.

**Our bet:** if we remove (a) the capex and (b) the "what if something breaks at night" fear, owners will run lights-out on repeat jobs.

## 2. Product

A retrofit cell parked next to an existing lathe or VMC:

| Part | What it does | Owner |
|---|---|---|
| Arm (low-cost cobot) + gripper | Pick raw part from infeed tray → load chuck/vise → unload finished part to outfeed | HW |
| Part trays / fixtures | Grid trays sized for a family of parts; quick changeover | HW |
| CNC I/O interface | Door open/close, clamp/unclamp, cycle start, M-code handshake, feed hold | HW + CS |
| Guarding + e-stop + interlocks | Hardware safety — never software | HW |
| Cell controller | Explicit state machine sequencing the cycle | CS |
| **Watchman** | Spindle current (CT clamp), vibration (accelerometer), sound (mic) → detects tool breakage, jams, chip buildup → feed hold + safe stop + alert | CS |
| Telemetry agent | Logs everything, works offline, syncs later | Shrish |
| Cloud + alerts | Dashboard, WhatsApp/phone alerts to owner, billing by machine-hour | Shrish |

## 3. Business model

**Decided (2026-09-29): rental.**
- **Monthly base fee** + **fee per extra machine-hour** the cell produces.
- **12-month minimum** term.
- **2–3 month deposit** up front (refundable at end of term, minus damage).

Pricing to test in interviews (**all [verify]**):
- Shop's own machine-hour rate: ₹400–900/hr (VMC), ₹250–600/hr (CNC lathe).
- Base fee: ~₹15–25k/month. Per-hour fee: ~₹60–120 per extra hour.
- At 150 extra hrs/month: ~₹24k–43k/month per cell.
- Payback on a ₹9 lakh cell: ~21–37 months. **A 12-month minimum does not pay back the cell by itself.** We need renewals or redeploying the cell to another customer. Track renewal rate from the first pilot.

### Cell cost target: **under ₹10 lakh all-in** (hardware + guarding + electronics + install)

Cheap cobot cell vs. simple gantry loader. Ranges are placeholders until real quotes land in the cost tracker below.

| Line item | Cobot cell (₹) | Gantry loader (₹) | Notes |
|---|---|---|---|
| Arm / gantry axes + drives | 3.5–6 L | 1.5–3.5 L | Cobot: low-cost 6-axis brands. Gantry: 2–3 axis linear modules + servos/steppers |
| Gripper | 0.5–1.5 L | 0.5–1.5 L | Pneumatic parallel gripper; part-specific jaws |
| Guarding (fence, doors, light curtain) | 0.8–1.5 L | 0.5–1.2 L | Gantry can often use a smaller enclosure |
| Safety relay + e-stops + interlocks | 0.3–0.6 L | 0.3–0.6 L | Certified relay (see CLAUDE.md) |
| Edge computer + watchman sensors + I/O | 0.3–0.6 L | 0.3–0.6 L | CT clamp, accelerometer, mic, I/O module |
| Trays / fixtures | 0.3–1 L | 0.3–1 L | Per part family |
| Install + commissioning (India) | 0.5–1 L | 0.7–1.5 L | Gantry needs more custom mechanical work on site |
| **Total** | **~6.2–12.2 L** | **~4.1–9.9 L** | **[verify]** |

**Trade-off:** the gantry is cheaper and simpler but more custom per machine, and weaker at reorienting parts or reaching awkward chucks. The cobot is more flexible and faster to redeploy, and that matters for a rental fleet that moves between customers. The software supports both: the controller only uses named poses (see `cell/drivers/robot.py`).

## 4. Milestones  ← current milestone lives here

**Telemetry agent is done (built early, 2026-09-29):** `telemetry/`: offline SQLite queue, idempotent ids, outbound HTTPS with backoff, tested for 6 h offline / flapping network / restart mid-sync. W4 only needs to wire it into the hub; the cloud ingest endpoint (dedupe by id) comes with W5.

**M0 (cell sim) is done:** controller, SAFE, watchman sim, plausibility, fault injection, sim.run demo (2026-09-29). The arm track pauses here until January.

### Watchman track to December pilots

| # | Milestone | Done when | Target | Status |
|---|---|---|---|---|
| **W0** | Design: node / hub / cloud split, BOM, remote-pause protocol | Docs reviewed | 2026-10-03 | **← NOW** |
| W1 | Node hard-stop rules in portable C, parity with the Python reference | Identical decisions on all golden data; CI builds C | 2026-10-10 | |
| W2 | Node firmware (sensors, features, hard stops, relay, MQTT, record mode) + `tools/record` | 2 node kits recording at a makerspace; Parquet + labels flowing | 2026-10-24 | |
| W3 | Makerspace data: ≥ 20 labelled sessions incl. real tool breaks / wear | Parity suite and detector thresholds re-run on real data | 2026-11-07 | |
| W4 | Hub service (ingest, detectors, references, operator page, node-offline) + telemetry agent | Hub runs 48 h on a bench with 2 nodes, network pulled repeatedly, no data lost | 2026-11-14 | |
| W5 | WhatsApp alerts (templates approved) + owner web app (OTP, read-only, signed remote pause, audit) | Pause from a phone reaches a bench node and is confirmed back; no start path exists | 2026-11-28 | |
| W6 | Pilot installs: 2 factories, 3–5 machines each, **alert-only** | Nodes live, owners get alerts | 2026-12-05 | |
| W7 | Pilot review | False-alert rate, caught events, owner feedback; decide which machines opt in to auto-stop | 2026-12-31 | |

**Long lead items, start now:** WhatsApp Business verification + template approval; SMS/OTP provider account; node parts order (2 recording kits in October, pilot kits by mid-November); pilot factories' permission to wire the feed-hold relay.

### Arm track (paused until January)

| # | Milestone | Done when | Status |
|---|---|---|---|
| M0 | Desk sim | Done 2026-09-29 | ✅ |
| M1–M5 | Bench cell, supervised then unattended pilot, billing | See earlier plan; resumes January with real watchman data | paused |

## 5. Architecture (maps to CLAUDE.md)

```
nightshift/
  cell/
    drivers/      # interfaces + sim/mock impls: robot, cnc_io, gripper, sensors
    controller/   # explicit state machine (IDLE → LOAD → MACHINING → UNLOAD → … , SAFE)
    watchman/     # independent process; can feed-hold via CncIo any time
  telemetry/      # offline-first agent: local queue → sync to cloud
  cloud/          # API, dashboard, alerts, billing
  cells/          # per-cell YAML config (poses, timeouts, pins)
  tests/faults/   # fault-injection tests
  data/           # raw sensor data (git-ignored)
```

## 6. Biggest risks

1. **Watchman accuracy** — too many false alarms → owners ignore it; misses → a crash kills trust. Mitigation: M1/M2 data before robot.
2. **CNC integration variety** — Fanuc / Siemens / Mitsubishi / local controllers, old machines with no spare I/O, manual doors. Need to survey what's actually on shop floors.
3. **Part variety / changeover** — job shops switch jobs often; if changeover takes hours, the cell sits idle.
4. **Our capex** — renting means we fund hardware. Need low BOM + financing/grant (e.g. startup India schemes **[verify]**).
5. **Safety & liability** — guarding and e-stops done properly; who is liable if something breaks at night.
6. **Team logistics** — team is in Amherst; need an India field engineer by M3.
7. **Indian labor is cheap** (operators ~₹15–25k/month **[verify]**). The arm must win on more than wage savings: attrition/absenteeism, consistency, running hours people won't work, and watchman-backed safety. Validate this with every pilot factory.
8. **Hardware fundraising** — investors will want pilot data and a working bench cell before a real check; plan the raise around M2.
9. **WhatsApp Business verification / template approval** can take weeks — start now or pilots launch without WhatsApp.
10. **Feed-hold wiring** into customers' CNCs (warranty, service contracts, controller variety). Alert-only default means pilots don't depend on it.
11. **False alerts** kill trust faster than missed ones: pilot thresholds come from makerspace data, and alert rate is the first pilot metric.

## 7. Cost tracker

Fill in real quotes as they arrive. Never delete old rows; add a new row when a price changes.

| Item | Option | Vendor | Quote (₹) | Source (link / person) | Date | Notes |
|---|---|---|---|---|---|---|
| Cobot arm | | | | | | |
| Cobot arm | | | | | | |
| Gantry axes + drives | | | | | | |
| Gripper | | | | | | |
| Guarding / light curtain | | | | | | |
| Safety relay | | | | | | |
| Edge computer | | | | | | |
| CT clamp / accelerometer / mic | | | | | | |
| Trays / fixtures | | | | | | |
| Install (per cell) | | | | | | |
| Watchman node kit (see hardware/bom.md) | | | | | | |
| ESP32-S3-DevKitC-1 | | | | | | |
| SCT-013 clamp + ADS1115 | | | | | | |
| ADXL345 / ADXL355 | | | | | | |
| DIN interface relay | | | | | | |
| Shop hub (Pi 5 kit + UPS + 4G) | | | | | | |
| WhatsApp Business API (per-conversation) | | | | | | |
| SMS OTP provider | | | | | | |

## 8. Open questions (answer through customer interviews)

- What % of their jobs are repeat batches of 50+ identical parts?
- What machines/controllers do they have, and how old?
- What actually goes wrong at night today when they try? (tool breakage, chips, coolant, power cuts?)
- Would they accept base fee + per-hour + 12-month minimum + deposit? What feels fair?
- Who gets called at 2am — owner, supervisor?
- Power and internet reliability on the floor.

## 9. Next 2 weeks

- [ ] **Shrish:** WhatsApp Business verification + templates; SMS OTP provider account; line up 2 makerspace sessions; confirm the 2 pilot factories and machines; review watchman design docs.
- [ ] **CS:** W1 hard-stop parity (C + Python), then node firmware skeleton.
- [ ] **HW:** order 2 node recording kits; answer the open questions in `hardware/bom.md` (clamp range, feed-hold input per pilot machine, accel choice).
- [ ] **All:** review `docs/watchman-architecture.md`.
