# Watchman architecture (December 2026 shop pilots)

Status: design for review. Decisions dated 2026-09-29 by the founders. Everything marked
**VERIFY** needs checking on real hardware or with a vendor.

## 1. What it is

A retrofit monitor for an existing CNC machine. It watches spindle current, vibration and
sound, alerts the shop owner, and can (opt-in, per machine) put the machine in feed hold on
a hard fault. It is **not a safety function**: operator safety stays with the machine's own
guarding, interlocks and e-stop. The watchman protects tools, parts and spindles.

Three tiers, each able to keep doing its own job when the tier above is gone:

```
 machine                      shop                          our cloud
┌────────────────────┐  MQTT  ┌──────────────────────┐ HTTPS ┌─────────────────────┐
│ ESP32-S3 node      │◀──────▶│ Raspberry Pi hub     │──────▶│ API + owner web app │
│ current, vib, mic  │ Wi-Fi  │ adaptive detectors   │ out-  │ OTP login, alerts,  │
│ features @ 10 Hz   │  LAN   │ references, alerts,  │ bound │ history, signed     │
│ HARD-STOP rules    │        │ telemetry buffer,    │ only  │ remote PAUSE        │
│ feed-hold relay    │        │ WhatsApp, op. page   │       │ (never start)       │
└────────────────────┘        └──────────────────────┘       └─────────────────────┘
```

| Tier | Keeps working without | Owns |
|---|---|---|
| Node | hub, internet | sensing, features, **hard-stop rules**, relay, local pause/reset button |
| Hub | internet | adaptive detection (chip buildup, wear, broken-before-cut), references per program + tool, local alerts, telemetry buffer, operator page |
| Cloud | n/a | owner app, WhatsApp, history, remote pause, audit log |

## 2. Node (one per machine)

**Hardware** (see `hardware/bom.md`): ESP32-S3; SCT-013-030 current clamp on one spindle
motor phase via ADS1115; ADXL345 accelerometer (SPI, 3.2 kHz) on a magnetic mount near the
spindle; INMP441 I2S microphone; opto-isolated relay wired to the CNC feed-hold input;
local PAUSE/RESET button; status LED.

**Firmware loop** (PlatformIO, Arduino-ESP32):
1. Sample current (ADS1115, up to 860 SPS), vibration (3.2 kHz) and audio (16 kHz, I2S).
2. Every 100 ms compute features: current RMS, vibration RMS, audio RMS and band energies.
3. Run the **hard-stop rules** (`node/lib/hardstop`, portable C shared with the parity
   tests): cut detection from current with hysteresis (standalone nodes have no CNC I/O),
   mid-cut tool break (current collapses vs. the cut's settled mean for a confirm window),
   overload (current or vibration above a limit for a confirm window), sensor-fault checks.
4. On a hard stop: **alert-only mode** (default) → publish the event. **Auto-stop mode**
   (opt-in per machine) → energise the relay (feed hold) immediately, then publish. The
   node never waits on the network to act.
5. Publish features, events and a 1 Hz heartbeat to the hub.

**Relay wiring (decided): energised = hold.** A dead or unpowered node leaves the machine
running; the hub raises "node offline" within seconds. The relay latches once energised and
**clears only with the local button** on the node. Nothing remote can clear it.
VERIFY per machine: which feed-hold input, its polarity, and that the relay contact meets
the input's spec.

**Record mode:** streams raw samples over USB serial (framed, CRC-checked) for
`tools/record`. Used at makerspace sessions to build training data.

## 3. Hub (one per shop)

A Python service on a Raspberry Pi 5 with Mosquitto.

- Ingests node features and runs one `cell.watchman` per machine, using the **same Python
  hard-stop reference** as the parity tests plus the adaptive detectors.
- References per program + tool, recorded only in supervised runs with operator
  confirmation of a fresh tool and of each clean cycle; deleted on tool change.
  Standalone machines have no CNC I/O, so the operator picks the active program + tool on
  the hub's local operator page (phone or tablet on the shop Wi-Fi). **Open question:**
  automatic job recognition from the current signature (later).
- Per-machine mode, `alert_only` (default) or `auto_stop` (opt-in), pushed to the node.
- "Node offline" after 5 missed heartbeats (5 s).
- Telemetry agent: SQLite queue, idempotent event ids, batched outbound HTTPS with
  backoff. Survives hours offline.

## 4. Cloud

FastAPI in Docker (deploy target TBD, e.g. AWS Mumbai).

- Owner login by phone number + OTP (SMS provider behind an interface; e.g. MSG91).
- Read-only dashboard: machines, live state, alerts, cut history, daily summary.
- WhatsApp alerts via the WhatsApp Business Cloud API (Meta) with approved templates.
  **Start business verification and template approval now: days to weeks.**
- Remote pause and audit log (below).

## 5. Remote pause (never remote start)

```
owner app ──(OTP session)──▶ cloud: PauseCommand{node_id, nonce, expires_at, user, reason}
cloud ──signs Ed25519──▶ hub (over its own outbound connection; hub polls/long-polls)
hub: verify cloud signature, expiry, nonce not seen ──▶ node (MQTT, HMAC with per-node key)
node: verify HMAC ──▶ energise relay (feed hold) ──▶ PauseAck{nonce, relay_state} (HMAC)
hub ──▶ cloud: ack ──▶ audit log {who, when, node, nonce, ack time, relay_state}
```

- **The protocol has no start, resume or release message**, and neither do the hub API,
  the cloud API or the firmware command parser. A test enumerates every message type and
  route to prove it. Resume only happens at the machine, with the node's local button and
  then the machine's own cycle start.
- Remote pause works in both alert-only and auto-stop modes (a human decided it).
- Expired, replayed or unsigned commands are rejected and logged.

## 6. Failure modes

| Failure | What happens |
|---|---|
| Node loses power or crashes | Relay de-energised, machine keeps running; hub alerts "node offline" in ~5 s |
| Node Wi-Fi drops | Node keeps sensing and hard-stopping locally (auto-stop mode); hub alerts offline; events buffered on node (ring buffer) and sent on reconnect |
| Hub down | Nodes keep hard-stopping locally; no adaptive detection or alerts until it's back |
| Internet down | Hub keeps detecting and alerting locally, and buffers telemetry; WhatsApp and the owner app are delayed |
| Sensor fault (e.g. clamp unplugged) | Node flags the sensor fault; hub alerts; hard-stop rules on that channel disabled and reported |
| Robot cell attached, node offline/unhealthy | Current cut finishes; no next load, no unattended start; supervised runs allowed (WAIT_WATCHMAN) |

## 7. MQTT topics (hub LAN, TLS + per-node credentials)

| Topic | Direction | Payload |
|---|---|---|
| `ns/{node}/features` | node → hub | 10 Hz: ts, current_rms, vib_rms, audio_rms, cutting |
| `ns/{node}/event` | node → hub | hard-stop / sensor-fault events, relay changes |
| `ns/{node}/heartbeat` | node → hub | 1 Hz: uptime, fw version, mode, relay state |
| `ns/{node}/config` | hub → node | mode (alert_only / auto_stop), thresholds (retained) |
| `ns/{node}/cmd` | hub → node | **PAUSE only** (HMAC-signed) |
| `ns/{node}/ack` | node → hub | PauseAck |

## 8. Parity: one set of hard-stop rules

The node's C rules (`node/lib/hardstop`) and the Python reference
(`cell/watchman/hardstop.py`) must make **identical decisions** on the same data. The
parity suite compiles the C library on the host, feeds both implementations the same golden
datasets (sim streams, threshold edge cases, and makerspace recordings once we have them),
and compares decisions sample by sample. The Python side computes in float32 to match the
ESP32.
