# Online features: exploration (not planned yet)

Status: **ideas for later, nothing scheduled.** Written 2026-11-05 at the owner's request ("the
handheld has Wi-Fi: should we plan internet features, other players' traffic, an AI ATC?"). PLAN §2
lists multiplayer and ATC as non-goals. This page collects what it would take to revisit that, so
a decision can be made later. Nothing here is built, and every item would need a work package, a
decision-log entry in [PLAN.md](PLAN.md) and, for new code that talks to the network, a licence
check for any library it brings in (D17).

## 1. What the device gives us, and what we don't know yet

- **Radio:** Wi-Fi 802.11a/b/g/n/ac (2.4 and 5 GHz) and Bluetooth 4.2 on the RG40XXH. CPU: four
  Cortex-A53 cores at 1.5 GHz, 1 GB RAM (DEVICE.md).
- **No microphone** is listed in the specs we found, so voice input is out. Pilot "radio calls" have
  to be picked from a menu (or typed on the on-screen keyboard, which is slow).
- **Unknown, to measure on the device** (a probe run, like T1 / X3 did for graphics):
  - whether the stock OS keeps Wi-Fi connected while a PortMaster port runs, and how long it takes
    to come back after sleep;
  - which network tools and libraries the stock OS has (`curl`, `wget`, `libcurl`, `libssl`
    versions) and whether a port may use them;
  - whether the clock is set at boot (TLS certificate checks fail with a wrong date);
  - Wi-Fi power cost: the battery is 3,300 mAh, and a radio that stays on costs flight time.
- **Today's game links only SDL2, JSBSim, libm, libpthread and libc** (check_package.sh enforces
  the list). Anything that speaks HTTPS needs a TLS library inside the game or a helper process.

## 2. Ground rules any online feature should keep

1. **Offline first.** The game must play exactly as now with Wi-Fi off. Every online feature is
   optional and off by default, with a settings row.
2. **No network I/O on the frame thread.** A background thread owns sockets; the game reads results
   from a queue, the same way terrain tiles arrive (W1).
3. **No secrets in the repository or the release zip.** An API key shipped in an open-source game is
   public the day it ships. Keys, if any, are the player's own and live in `conf/`.
4. **Privacy:** position and callsign leave the device only for features the player turned on, and
   the setting says what is sent where.
5. **Small and few dependencies.** Prefer a helper process that already exists on the device, or one
   small, permissively licensed library (for example mbedTLS, Apache-2.0) over a stack of them.

## 3. Building blocks

| Block | How | Good for | Cost |
|---|---|---|---|
| A. Launcher fetch | `FlightSim.sh` runs the system `curl` (if present) before the game starts and writes files into `conf/` / `data/` | Weather at launch, data checks | Smallest: no new code in the binary. Data only as fresh as the launch. |
| B. In-game HTTPS | A `net/` module: background thread, minimal HTTP/1.1 client, TLS through mbedTLS (decision needed) | Weather refresh in flight, traffic polling, AI ATC | New dependency and certificate bundle; about a week of work |
| C. Plain UDP | Sockets only, no TLS | Multiplayer positions (LAN or a relay server) | Small; positions are not secret, but nothing is encrypted |

## 4. Candidate features

### 4.1 Live weather (already on the v2 list)
METAR (and TAF) from aviationweather.gov, a US Government service. Block A gets the weather at
launch with no new code in the game (the X1 METAR reader already loads `data/weather/<ICAO>.txt`).
Block B could refresh it in flight and pick up weather at the destination. W7 clouds already come
from the METAR. **Effort: small. Value: high. Risk: low.** The natural first step.

### 4.2 Real-world traffic (ADS-B)
Show the real airliners near you: TCAS-like targets on the HUD and map, and V1's in-code models
drawn in the world at their reported positions. The game polls an ADS-B service every 10 to 15
seconds inside a box around the aircraft and moves the targets between updates.
- **OpenSky Network** REST API: anonymous use gets 400 API credits per day, registered users 4,000;
  a small box costs fewer credits. Their terms reportedly need a written agreement for use inside a
  product, even a non-commercial one: **check with OpenSky before building on it.**
- **adsb.lol**: community network, data under the ODbL (the OpenStreetMap licence), with a free API;
  live access may favour people who feed data. Check the terms and rate limits at api.adsb.lol/docs.
- **ADS-B Exchange**: commercial API, paid.

**Effort: medium** (network block B, a traffic store, HUD / map symbols, models in the world).
**Value: high**: the sky stops being empty, and it works in single player. **Risks:** data terms,
coverage gaps on the coast, and it needs Wi-Fi in flight.

### 4.3 Other players
| Option | What it is | For | Against |
|---|---|---|---|
| LAN | Two or more handhelds on the same Wi-Fi find each other by UDP broadcast and exchange position, attitude, gear, flaps and lights about 5 to 10 times a second | No server, no accounts, no running cost | Only people in the same room or house |
| Own relay | A tiny UDP (or WebSocket) relay the owner hosts; clients send their state, the relay forwards it to clients nearby | Works over the internet; simple | Hosting cost, uptime, and some moderation (callsigns) |
| FlightGear network | Speak FlightGear's multiplayer protocol to its public servers (UDP port 5000, servers interconnected, roughly 500 bytes per update, about 5 KB/s per client at 10 Hz) | Instantly populated with FlightGear pilots | Protocol details and server policy still to check; aircraft model names need mapping to V1 categories |
| VATSIM / IVAO | Online ATC networks | Real human ATC | Need approved client software and real procedures: not realistic for this game |

What the game needs in every case: a remote-aircraft store with interpolation (positions arrive late
and out of order), V1 models drawn per aircraft (one draw each, or batched), labels through the W6
overlay, and traffic symbols on the HUD and the map. **Effort: LAN medium, relay medium plus
hosting, FlightGear compatibility large.**

### 4.4 ATC
**Input on a handheld without a microphone:** a radio menu (a new pad layer or the pause menu) that
offers the calls that make sense now: "ready for departure", "request ILS 28L", "request higher",
"with you at 6,000". The controller's reply shows as text (and is spoken once S2 settles on a
voice source).

Two ways to make the controller:
- **Rule-based, offline.** A state machine per phase: clearance, takeoff, departure, enroute
  altitude changes, vectors to final, landing clearance, go-around. It uses what the game already
  knows (plan, runways, approaches, weather, traffic if 4.2/4.3 exist). It is deterministic and
  testable. Instructions can also drive the AP (heading and altitude bugs) with the player's
  consent. Taxi clearances need taxiway data the packs don't have yet.
- **LLM-based (AI ATC).** The game sends a compact state (callsign, position, phase, plan, runway,
  weather, the last few exchanges) and the player's chosen call. A small, fast model answers with a
  transmission and structured instructions (JSON: heading, altitude, speed, clearance). The game
  **validates** those instructions against the rules before showing or applying them, because a
  model can be wrong. A hybrid works best: the rules decide what is cleared, and the model phrases
  it and handles calls the rules don't cover.
  - **Cost:** with Claude Haiku 5.5 ($0.10 per million input tokens, $0.50 per million output tokens),
    a call with about 2,500 tokens of context and an 80-token answer costs about $0.0003. Sixty
    calls in a flight cost about two cents. Prompt caching only helps once the fixed part of the
    prompt is longer than the model's minimum cacheable prefix; at these sizes it doesn't matter.
  - **Latency:** expect one to a few seconds per call (to be measured). Fine for ATC: real
    controllers pause too, and the HUD can show "STANDBY". Low effort settings keep it short.
  - **Keys:** never in the build. Either the player brings their own API key (stored in `conf/`,
    sent only to the API), or the owner runs a small relay that holds the key and limits each device.
    The relay costs money and needs abuse limits.
  - **Needs:** block B (HTTPS) on the device, a decision on what leaves the device, and a fallback to
    the rule-based controller when offline.

**Effort: rule-based large but self-contained; the LLM layer on top medium.**

### 4.5 Smaller online extras
- Download region data packs from the GitHub releases inside the game (R3 builds them); needs block
  B and space checks on the SD card.
- Flight plan import from a planning service (SimBrief has an API keyed by the user's ID; check its
  terms).
- Opt-in upload of logs or a crash report to help debugging.
- Shared landing scores (with U1 "quick flight and landing score" from the work board in PR #10).

## 5. A sensible order, if the owner wants to go ahead

1. **Device probe for networking** (extend FlightSimProbe): Wi-Fi during a port, `curl` / `libssl`,
   clock, power draw. No game change.
2. **Weather at launch** through block A (no new dependency).
3. **Decide block B** (mbedTLS or a helper process), then **weather refresh in flight**.
4. **Real-world traffic** after a licence answer from adsb.lol or OpenSky. Single player, big effect.
5. **LAN multiplayer** over UDP: no server and no accounts.
6. **Rule-based ATC**, offline.
7. **AI ATC** on top of 6, bring-your-own key first.
8. **Internet multiplayer** (own relay, or FlightGear compatibility) if there is demand.

## 6. Questions for the owner

- Is moving multiplayer and ATC off the non-goals list (PLAN §2) wanted, and for which release?
- Is a server you host acceptable (relay for multiplayer or AI ATC), or must everything be
  peer-to-peer and serverless?
- Bring-your-own API key only, or a hosted relay paid by the project?
- Which traffic source's terms are acceptable to you?
- Is a TLS library in the game binary acceptable (decision-log entry), or should network access stay
  in the launcher?

## Sources

- [Anbernic RG40XX H product page](https://anbernic.com/products/rg40xx-h) (Wi-Fi, Bluetooth,
  CPU, RAM) and [handhelds.wiki overview](https://handhelds.wiki/RG40XX_H_Overview)
- [OpenSky REST API documentation](https://openskynetwork.github.io/opensky-api/rest.html) (credit
  limits); terms of use at opensky-network.org (not verified here)
- [awesome-adsb list](https://github.com/rickstaa/awesome-adsb) and the adsb.lol API docs at
  api.adsb.lol/docs (ODbL data, free API; verify terms)
- [FlightGear wiki: Multiplayer](https://wiki.flightgear.org/Multiplayer) and
  [Howto: Set up a multiplayer server](https://wiki.flightgear.org/Howto:Set_up_a_multiplayer_server)
- Claude model pricing as of November 2026 (Claude Haiku 5.5: $0.10 / $0.50 per million input /
  output tokens for prompts up to 100K tokens)
