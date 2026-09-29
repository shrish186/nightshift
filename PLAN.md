# NightShift — Plan (draft v0, 2026-09-29)

> Status: first draft. Every number marked **[verify]** is an assumption we have not checked yet.
> Owners: Shrish (backend/cloud/AI/business), CS cofounder (controller/vision/watchman), HW cofounder (arm/gripper/fixtures/guarding).
> Team is based in Amherst, MA; pilots and installs happen in India. Pilots via founders' contacts at large factories.
> Decision (2026-09-29): **we are building the arm + watchman together, done properly, and raising funding for it.** The watchman ships first as the safety/trust layer the arm depends on; the arm is developed in parallel, not deferred.

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

**Rental, billed per extra machine-hour** the cell produces (hours the machine would otherwise have been idle).

Rough unit economics — **all [verify]** in customer interviews:
- Shop's own machine-hour rate: ₹400–900/hr for VMC, ₹250–600/hr for CNC lathe.
- We charge: ~₹100–200 per extra hour (a share of value, not of cost).
- Target utilisation: 8 extra hrs/night × 25 nights = 200 hrs/month → ₹20k–40k/month per cell.
- Cell BOM target: ₹6–10 lakh (Chinese cobot ₹3–6 lakh, gripper, trays, guarding, edge box, sensors).
- Payback on our capex: ~18–30 months at those numbers. This is the key number to de-risk; it depends almost entirely on **how many nights the cell actually runs**.

Implication: we need a cheap cell, fast changeover between part families, and very high trust in the watchman.

## 4. Milestones  ← current milestone lives here

Two tracks in parallel. The watchman ships to pilots first (cheap, fast, builds trust and data); the arm is built on the bench at the same time and joins the watchman on the pilot machine.

| # | Watchman track (CS + Shrish) | Arm track (HW + CS) | Done when | Status |
|---|---|---|---|---|
| **M0** | Sensor/`CncIo` interfaces + mocks, telemetry agent (offline-first) | `Robot`/`Gripper` interfaces + mocks, controller state machine, fault-injection tests | Full load→machine→unload cycle + injected faults pass in sim; CI green | **← NOW** |
| M1 | Sensor kit on 3–5 machines at pilot factory, record labeled data | Pick 1 part family at pilot factory; choose cobot + gripper; bench cell in Amherst | 2–4 weeks of labeled data; bench arm loads a dummy fixture | |
| M2 | Alerts only (WhatsApp), measure false alarms | 500 consecutive bench cycles against sim CNC/PLC, zero unsafe events | False alarms < 1/night **[target]**; bench reliability number | |
| **Raise** | — | — | Pilot data + bench video + LOIs from factory contacts → seed round | |
| M3 | Watchman commands feed hold on high-confidence faults | Cell shipped to India, installed on 1 machine, supervised runs | 10 supervised shifts, every stop explained | |
| M4 | — | Unattended shifts with watchman as the safety layer | First paid machine-hours | |
| M5 | Watchman rolled out plant-wide | 3 cells across 2 factories | Install ≤ 2 days; repeatable | |

**India on-the-ground:** from M3 we need a full-time field engineer in India (first hire after raise). Until then, installs are done by founders during trips.

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

## 7. Open questions (answer through customer interviews)

- What % of their jobs are repeat batches of 50+ identical parts?
- What machines/controllers do they have, and how old?
- What actually goes wrong at night today when they try? (tool breakage, chips, coolant, power cuts?)
- Would they pay per hour? What would they consider fair?
- Who gets called at 2am — owner, supervisor?
- Power and internet reliability on the floor.

## 8. Next 2 weeks

- [ ] **Shrish:** 15 customer calls/visits via contacts in 2 clusters; fill section 7. Set up repo, CI, telemetry agent skeleton.
- [ ] **CS:** driver interfaces + mocks, controller state machine with tests (M0).
- [ ] **HW:** shortlist 3 cobots (price, payload, India support), sensor kit BOM for M1.
- [ ] **All:** pick the one friendly shop for M1.
