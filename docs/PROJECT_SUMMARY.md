# UFH Controller: Project Summary

_Written 20 Nov 2026 to merge several parallel Claude threads into one project. The full engineering
history is in `docs/PROJECT_CONTEXT.md` in this repo. This file is the short version: what exists, how
it runs, what was decided and what is still open._

---

## 1. What this is

An ESPHome controller for an underfloor-heating (UFH) manifold and its mixing valve, integrated with
Home Assistant (HA). All heating logic runs on the controller, so the heating works with HA or the
network down. HA is for monitoring, setpoints and overrides.

- **Owner:** Jon does the wiring, testing and on-site checks. Claude writes and maintains the code.
- **Repo:** `ifold-estate/ufh-controller`, branch `main`. Local copy:
  `~/Documents/claude/UFH/ufh-controller`.
- **Build and deploy:**
  1. A push to `main` triggers GitHub Actions, which compiles the firmware.
  2. The CI `.bin` is flashed over the air (OTA) to the controller at **192.168.1.232** (Wi-Fi build,
     `ufh-controller-wifi.yaml`).
  3. Never flash the Ethernet build (`ufh-controller.yaml`) to the installed board. With no network
     cable it would go offline, and recovery needs USB-C.
- **A remote flash stops the floor pump for about 2 minutes.** That lets heat creep into the mixer
  area, which then arrives as a hot slug. Keep flashes few and batch changes.

## 2. Hardware

| Item | Details |
|---|---|
| Controller | Waveshare ESP32-S3-POE-ETH-8DI-8RO: 16 MB flash, 8 MB PSRAM, 8 relays, 8 opto-isolated inputs, isolated RS485, RTC. Wi-Fi with an external antenna; powered locally from DC, PoE unused. |
| Mixer drive | Waveshare SC15 bus servo in continuous (motor) mode, through the old actuator's gearbox (about 4.5:1). Connected to UART GPIO43/44 through Bus Servo Adapter A at 115200 baud, with no level shifter. |
| Mixer position | Servo potentiometer (about 210° visible per turn), plus the old actuator's four cam switches on DI5–8. DI5–8 form a code that changes one bit at a time, with the cold end at 0101 and the hot end at 1011. |
| Temperatures | Three DS18B20 probes on GPIO21: boiler flow (the mixer's hot inlet, about 1 m before the mixer), return, and mixed flow (into the floor). |
| Display | Waveshare 2.4" ILI9341 LCD on SPI3 (GPIO48/47, DC GPIO3, RST GPIO0), showing an animated heat-flow screen. |
| Zone valves | 24 Vac solenoids switched by relays 4–8. Openings are staggered 30 s apart, because opening all at once blew the 24 Vac fuse. |

**Relays:** 1 boiler call for heat (CFH), 2 floor pump, 3 spare, 4–8 zone valves.

**Thermostats and zones (corrected 18 Nov 2026):**

| Input | Wire | Room | Thermostat | Relay | Manifold zones |
|---|---|---|---|---|---|
| DI1 | purple | Playroom | T4, mechanical, not in HA | 7 | 7, 8 |
| DI2 | pink | Booth | N2, Nest v3, official integration (`climate.booth_booth`) | 4 | 1, 6 (zone 1's solenoid has failed) |
| DI3 | red | Kitchen / conservatory | N1, Nest Gen 2 on NoLongerEvil (`climate.nest_02aa01ac2613049n`) | 8, and 6 | 2, 3, plus 4, 5 through relay 6 |
| DI4 | orange | Lounge | N3, Nest Gen 2 on NoLongerEvil (`climate.nest_02aa01ac38130bgw`) | 5 | 9, 10 |

- "Relay 6 mode" is set to **Follow N1**: the kitchen centre (zones 4 and 5) follows the kitchen
  thermostat.
- Before 18 Nov, DI2 and DI3 were swapped in the firmware. Any room-by-room analysis from before that
  date has the kitchen and booth reversed.
- A fifth Nest, "Upstairs" (`climate.upstairs_upstairs`), runs the radiators and the hot water. It
  isn't a floor zone, but its demand starts the boiler pump, which changes how the mixer behaves (see
  §3).

## 3. The plant: the key facts behind the control design

- **Upstream timing chain:**
  1. Our CFH opens a 2-port valve, which takes about 15 s.
  2. Its end switch starts an upstream "dumb" controller's **3 min 50 s timer**, which then fires the
     boiler and its pump.
  3. Hot pumped flow reaches our probes about **4 min 15–30 s after CFH**.
  4. If our CFH drops before the timer ends, the timer resets and the boiler pump never starts.
- **Two flow regimes through the mixer:**
  - **Pulled:** boiler pump off. Our floor pump draws boiler water, and the share stays small.
  - **Pumped:** boiler pump on. The boiler side pushes hard and the share rises about 2.5×.
  - The valve curve (opening → boiler-water share) differs a lot between the two. In pumped mode
    nearly all the mixing happens between about 10 and 15 % open, so control is coarse.
  - The pumped curve also depends on how many zones are open: with three open it is about half the
    single-zone curve.
- **Low valve authority** is the root problem. The boiler side dominates, and boiler water even forces
  through the shut mixer when its pump runs. The standard fix is a regulating (lockshield) valve on the
  mixer's hot inlet. It's recommended, but not fitted.
- **Heat creep:** with our pump off and the boiler hot, the mixer area heats to about 66 °C by
  conduction.

## 4. Firmware: current state

Files:

| File | Contents |
|---|---|
| `ufh-controller.yaml` | Main config |
| `ufh-controller-wifi.yaml` | Production Wi-Fi wrapper |
| `mixer_control.h` | Mixer controller, positioner and position tracker |
| `heating_demand.h` | Thermostat-driven zone, pump and CFH sequencing |
| `ufh_screen.h` | LCD screen |
| `lcd_reset.h` | Periodic LCD re-initialisation |
| `cam_gpio.h` | Cam switch reading |
| `components/scs_servo/` | Feetech SCS servo driver |

**Heating auto (`heating_demand.h`):**
1. Each zone valve opens while its room has demand, with openings 30 s apart. Demand = thermostat
   OR the room's **Aligned burn** request from HA (20 Nov; see PROJECT_CONTEXT §5b). Requests clear
   at boot, after 255 min, and 30 s after HA disconnects. "Zone demand <room>" sensors show the result.
2. The floor pump starts once a valve has been open for 2 minutes.
3. CFH is allowed while any thermostat calls.
4. When the last call ends, CFH drops, the pump runs on for 3 minutes, then the valves close.

**Mixer mode "Average"** (the default; Band, Fine and Staged are fallbacks). The goal is a long-term
**average of 45 °C flow into the floor**. Brief peaks are OK, because the concrete smooths them. Servo
wear is secondary.
- **Continuous call:** CFH stays on while the pump runs and a thermostat calls.
- **Control law:** PI on the mixed-flow probe in boiler-water share, with gain 1 %/°C and integral
  time 180 s, plus feed-forward (setpoint − return) / (boiler − return).
  - The boiler is assumed to be at least 55 °C for the first 45 s of flow.
  - Anti-windup limits the integral to ±50 %.
- **Valve curves:** the controller uses the pulled curve until the boiler pump starts, capped at 61 %
  open, then the pumped curve.
  - **Pump start detection:** a fast rise in the mix, a 5-minute timeout, or the measured share
    matching one curve.
    - The share is checked against the geometric mean of the two curves.
    - It takes 1 minute of evidence to switch to pumped and 3 minutes to switch back to pulled.
  - **Hot-boiler start:** if the boiler is above setpoint + 15 °C, the call starts on the pumped
    curve.
  - The integral resets on every curve switch.
- **Early close (interim, 18 Nov):** at 4 minutes after CFH the valve goes to the pumped setting, just
  before the upstream timer starts the boiler pump. It can't switch back to pulled before 5.5 minutes.
- **Peak handling:**
  - Over the soft limit (51 °C), the valve closes 5 % per update.
  - If the mix stays over the soft limit for 30 s with the valve shut, CFH goes off until the mix is
    back under the setpoint (status "Cooling").
  - The hard limit (60 °C) latches CFH off until the mix is under 55 °C.
- **Status values:** No call, Call (waiting), Calling, Cooling, Pump off.

**Current settings:**

| Setting | Value |
|---|---|
| Mixed flow setpoint | 45 °C (raised from 40 on 14 Nov for freezing nights) |
| Soft peak limit | 51 °C |
| Mixed flow limit | 60 °C |
| Positioner deadband | 2 % |
| Average gain | 1 |
| Average integral time | 180 s |
| Boiler max assumed | 70 °C |
| Boiler cap | off |
| Relay 6 mode | Follow N1 |

**Diagnostic sensors:**
- Mixer hot share and Mixer target share
- Mixed flow average (15 min) and Mixed flow 24 h average
- Floor coil mean
- Heap free and PSRAM free
- Wi-Fi signal

**Tools:**
- `tools/mixer_sim/sim.cpp`: unit and safety tests. Run them after any change to `mixer_control.h`;
  all must print "safety checks passed".
- `tools/mixer_sim/*.py`: plant simulations.
- `tools/zone_analysis/fetch.py` and `analyze.py`: read-only HA history fetch, then a fit of each
  room's gain, lag and heat loss.
- `tools/scs_tool.py`: servo set-up.
- `tools/screen_preview/`: renders the LCD screen on a PC.

## 5. Results so far

- **Average mode works:**
  - Daily averages of 39.1–39.4 °C at a 40 °C setpoint, and 44.6–45.0 °C at 45 °C.
  - Staged mode had averaged about 35 °C.
- **Boiler-pump slugs:**
  - Before the early close, the mix spent 13–78 min a day over 50 °C, with peaks of 55–59 °C, almost
    all when the boiler pump started at the beginning of a call.
  - Since the early close (18–19 Nov), the call-start peaks have been 43–46 °C. No call has yet
    started with a hot boiler, so the early close itself is still untested.
  - On 19 Nov at 13:48–13:58, the boiler pump cycled mid-call (probably radiator or hot-water demand).
    The pump restarts gave 50–51 °C for about a minute each, which the soft limit caught. Left as is.

## 6. Fixed problems (for reference)

| Date | Problem | Cause and fix |
|---|---|---|
| 7 Nov | Valve jumped with the servo still | About 17 % lever play. Jon shimmed it and wired the cam switches. |
| 10 Nov | Average mode settled at about 34 °C | A false pump-start detection put it on the wrong curve. Fixed with a regime check from the measured share. |
| 10 Nov | The mix reached 55 °C with the valve shut | Boiler water forced through the mixer. The soft-limit CFH cut went from 2 minutes to 30 s. |
| 10 Nov | Positioner hammered the cold end | Fixed: no moves into an end switch that is already made. |
| 16–18 Nov | Input 4 (lounge) stuck ON for 44 h | ESPHome GPIO inputs are interrupt-driven by default, which suits a 24 Vac input poorly. DI1–4 are now polled (`use_interrupt: false`). |
| 18 Nov | DI2 and DI3 swapped | Corrected in the YAML. |
| 18 Nov | LCD blank most of the time | The 10-minute LCD re-init leaked a 150 KB frame buffer each time, and PSRAM ran out after about 8 h. Fixed, and PSRAM free is now steady. |

## 7. Open items and plans

1. **Verify the early close** on a call that starts with the boiler already hot, for example while hot
   water is heating. The call-start peak should stay under 50 °C.
2. **Kitchen:** watch whether it keeps up now that DI2 and DI3 are corrected. Before the fix it only got
   heat when the booth called. Then decide whether the 45 °C setpoint is still needed.
3. **Pressure sensor (Jon is sourcing one):**
   - **What:** a 0–5 bar gauge-pressure transmitter, RS485 Modbus RTU, 9–30 V DC, rated for water at
     100 °C or more, with a G¼ thread and an isolating valve.
   - **Where:** the boiler flow pipe before the mixer.
   - **Why:** to detect the boiler pump starting directly, from the pressure step, instead of
     inferring it from temperatures.
   - The first candidate on Amazon was only rated to 60 °C, so it's rejected.
   - Claude will write the ESPHome `modbus_controller` config once a datasheet is chosen.
4. **Second controller (Jon, at least a month away):** a second Waveshare board at the boiler, 2-port
   valve and upstream controller. It would drive the boiler pump directly and replace the timer chain.
   The early close and the pump detection are interim workarounds until then.
5. **Recommended hardware:** a regulating (lockshield) valve on the mixer's hot inlet, and possibly a
   lower boiler flow temperature. Either one spreads the mixing over more valve travel.
6. **Zone analysis:** re-run `tools/zone_analysis` with the corrected DI map once enough data has
   built up, to estimate each room's gain, lag and heat loss.
7. **Smaller items:**
   - Re-measure the valve curve at pump setting 3 with several zones open.
   - Replace the failed zone 1 solenoid.
   - Tidy the cam switch wiring, which glitches occasionally.
   - Consider a virtual thermostat that averages N1 and N2.

## 8. Access and conventions for Claude sessions

- **Home Assistant:**
  - `http://homeassistant.local:8123` (192.168.1.194) on the LAN, or `100.86.251.93:8123` over
    Tailscale.
  - HA is a Tailscale subnet router for 192.168.1.0/24, so remote OTA flashes work too.
  - Claude has **read-only** access, using a long-lived token kept in `~/.config/ufh/ha_token`. Never
    print it or commit it, and never sign into HA.
  - Controller entities are named `sensor.heating_ufh_plc_*`, `switch.heating_ufh_plc_relay_N` and
    `number.heating_ufh_plc_*`. The weather entity is `weather.forecast_home`.
  - For history queries, always pass `end_time`, or the API returns only 1 day.
- **Secrets:** `secrets.yaml` is gitignored and holds the API encryption key, the web password and the
  AP password. CI builds its own copy from the GitHub Actions secrets. Never print secrets.
- **Direct control:** the controller's native API, through `aioesphomeapi`, using the key in
  `secrets.yaml`. Claude keeps small helper scripts in a session scratch folder; they aren't in the
  repo.
- **Working rules agreed with Jon:**
  - Small fixes can be pushed without asking.
  - Changes to control behaviour need Jon's OK first.
  - Keep flashes few, because each one stops the pump for about 2 minutes.
  - Write commit messages in plain English.
- **Keep `docs/PROJECT_CONTEXT.md` updated** with every behaviour change. It is the authoritative
  history.
