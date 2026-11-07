# UFH Controller — Project Context and Handoff

This file carries the decisions and background from the first working session, so a new session
(human or Claude) can pick up the project. Keep it updated as decisions change.

_Last updated: 30 Oct 2026. Ethernet/PoE dropped: the installed controller runs on Wi-Fi with local power._

---

## 1. Goal

A controller for an underfloor heating (UFH) manifold and mixer, integrated with a local
Home Assistant (HA) server. The heating logic must run **on the controller itself**, so heating keeps
working if HA or the network is down. HA is used for monitoring, setpoints and overrides.

The owner (Jon) isn't familiar with the ESP32 environment, so **Claude writes and maintains the code**;
Jon does the wiring, flashing and testing.

## 2. Hardware

| Item | Details |
|---|---|
| Controller | **Waveshare ESP32-S3-POE-ETH-8DI-8RO**: ESP32-S3-WROOM-1U-N16R8 (16 MB flash, 8 MB octal PSRAM), W5500 Ethernet, PoE (802.3af) sub-module, 8 relays (10 A 250 Vac), 8 opto-isolated digital inputs, isolated RS485, PCF85063 RTC, TF slot, buzzer, WS2812 LED, 28-pin header. No CAN on this model. Docs: https://www.waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO (the PoE version uses the same baseboard). **Not** the 8DI-8DO board, which has transistor outputs. |
| Zone valves | 24 Vac solenoids on the manifold, switched by the relays |
| Thermostats | NEST v1 and v2, running on 24 Vac. Their call-for-heat output must reach the DIs (interface not designed yet; see §7). |
| Pump | 240 Vac, switched by a relay |
| Boiler call for heat (CFH) | 240 Vac signal, switched by a relay |
| Mixer | Mechanical mixing valve, turned by a **Waveshare SC15 bus servo** (Feetech SCS series, TTL half-duplex serial, 4.8–8.4 V, 15 kg·cm at 6 V, 0–1023 position over about 200°, feedback for position, speed, load, voltage and temperature). Load feedback will be used to find the valve's mechanical end stops. |
| Servo interface | **Waveshare Bus Servo Adapter A**. Jumper on A (UART mode); B is USB mode. Its input supply passes straight to the servo (SC15: 4.8–8.4 V; the bench uses 5.2 V). Its UART logic is 3.3 V (TXD input 5 V tolerant), so it connects straight to the ESP32. |
| Temperature sensors | Several DS18B20s (Jon already has them) |
| Display | **Waveshare 2.4" LCD Module**: ILI9341, 240×320, 4-wire SPI, no touch, pins VCC GND DIN CLK CS DC RST BL. Its logic level follows VCC, so it must be powered from 3V3. https://www.waveshare.com/wiki/2.4inch_LCD_Module |
| Parts on hand | Logic-level converter board, rectifier, PSU options |

## 3. Key decisions (and why)

1. **Firmware framework: ESPHome** (ESP-IDF framework). It integrates natively with HA through
   automatic discovery and the API, has on-device automations, thermostat and PID control, supports
   the W5500, TCA9554, PCF85063 and DS18B20, and its YAML and C++ are text Claude can maintain.
   Arduino/PlatformIO with MQTT and raw ESP-IDF were rejected as more code for the same result.
2. **Network is Wi-Fi; updates go over Wi-Fi (OTA)** after a first flash over USB-C. A network
   cable can't be pulled to the cabinet (30 Oct 2026), so the installed controller runs
   `ufh-controller-wifi.yaml` and Ethernet/PoE are unused. Fit the antenna outside or at the edge
   of a metal enclosure and check the "Wi-Fi signal" entity (aim for better than about −70 dBm).
   Heating control must not depend on Wi-Fi or HA being up, so the API's reboot-on-HA-loss is off
   (`reboot_timeout: 0s`); Wi-Fi keeps its default 15 min reboot to recover a stuck connection.
   The fallback "UFH Controller Setup" hotspot is WPA2-protected with `ap_password` (secrets.yaml
   locally, the `AP_PASSWORD` Actions secret in CI). Keep USB-C reachable in the enclosure
   for recovery.
3. **Servo on a plain UART from the header (Tx/Rx, GPIO43/44), not USB.** The ESP32-S3 can be a USB host, but
   that would take over the only USB port (needed for flashing, logs and recovery), USB-C isn't wired
   to supply VBUS, and ESPHome's USB-serial support is newer and less proven.
4. **The DI terminals can't be used for serial.** They are opto-isolated (5–36 V), input-only and slow.
5. **Don't power the board from 24 Vac.** The input is rated 7–36 V **DC**. 24 Vac peaks around 34 V
   and reaches roughly 37–40 V on a lightly loaded transformer, so it would overstress the input's TVS
   diode. A bridge rectifier doesn't help, since it gives the same peak voltage as DC. With PoE ruled
   out, **use a 24 Vdc (or 12 Vdc) DIN-rail PSU** from the cabinet's mains (e.g. Mean Well
   HDR-15-24).
6. **Logs go to USB Serial/JTAG**, so UART0 (header Tx/Rx, GPIO43/44) is free for the servo. The
   boot ROM prints start-up text on GPIO43 at 115200, but it has no 0xFF bytes, so it can't form a
   valid servo packet. (The servo was first planned for GPIO1/2; it moved because the header's Tx/Rx
   pins sit next to 3V3 and G, so one 4-pin connector carries everything.)
7. **The LCD uses the TF-card pins (47/48)**, so the TF card is not used. CS and BL are tied to fixed
   levels, so there's no backlight control. RST must be driven from GPIO0: with CS tied low and
   RST tied high, the panel stayed white, since it can't resync after stray clock edges at power-up.
   GPIO0 (BOOT) is high at power-on, so sharing it with the LCD reset is safe. The display runs at
   10 MHz in SPI mode 0 (set while debugging; keep them unless there's a reason to change).
8. **Servo link: 115200 baud, wired direct, no level shifter.** The adapter's schematic shows 3.3 V
   logic (SN74LVC1G125/126 on an AMS1117-3.3; RXD drives 0–3.3 V, TXD is 5 V tolerant). A TXS0108E
   shifter (HW-221) was tried first and was unreliable: its 5 V side needs ~3.4 V for a high, so the
   adapter's 3.3 V replies were marginal. The servo was set to 115200 with `tools/scs_tool.py`. A replacement servo needs the same change. Wiring is
   straight through, as Waveshare says: ESP Tx → adapter TX, ESP Rx → adapter RX (confirmed).

## 4. Pin map

Fixed by the board:

| Function | GPIO |
|---|---|
| DI1–DI8 | 4, 5, 6, 7, 8, 9, 10, 11 (active low, inverted in config) |
| Relays 1–8 | TCA9554 @ 0x20, expander pins 0–7 (EXIO1–8) |
| I2C (expander, RTC) | SDA 42, SCL 41 |
| W5500 (SPI2) | SCLK 15, MOSI 13, MISO 14, CS 16, INT 12 |
| RS485 | TX 17, RX 18 (automatic direction control) |
| TF card | 45, 47, 48 |
| WS2812 LED | 38 |
| Buzzer | 46 (strapping pin) |
| BOOT | 0 |

Header (2×14), read from Waveshare's board photo:

```
Top:    5V   5V   G    G    3V3  G    SCL  SDA  3V3  G    21   45   47   48
Bottom: Vbus G    DP   DN   3V3  G    Tx   Rx   3V3  G    0    1    2    3
```

Project allocation:

| Pin | Use |
|---|---|
| GPIO21 | DS18B20 one-wire bus, 4.7 kΩ pull-up to 3V3 |
| GPIO48 | LCD CLK (SPI3) |
| GPIO47 | LCD DIN (SPI3) |
| GPIO3 | LCD DC (strapping pin, fine as an output) |
| GPIO0 | LCD RST (BOOT pin; pressing BOOT also resets the LCD) |
| LCD CS → G, BL → 3V3, VCC → 3V3 | |
| GPIO43 (Tx) | Servo UART TX → adapter TX (direct) |
| GPIO44 (Rx) | Servo UART RX ← adapter RX (direct) |
| GPIO45 | Avoid (strapping pin for flash voltage) |
| Spare | GPIO1 and GPIO2. I2C (41/42) can take more devices. |

## 5. Code status (v0.1)

Repository: `ifold-estate/ufh-controller`

- `ufh-controller.yaml` sets up:
  - Ethernet (unused in the installation), the API (encrypted), OTA (encrypted with the API key), and the web server with auth
  - I2C, the TCA9554 relays (all `ALWAYS_OFF` at boot), DI1–8 with a 50 ms filter
  - The PCF85063 RTC (set from HA time), status, uptime, buzzer (RTTTL) and RGB LED
  - One DS18B20 placeholder (add addresses from the boot log)
  - The servo entities, and the LCD heat-flow screen (`ufh_screen.h`, from Jon's sketch, agreed
    30 Oct 2026): boiler loop, mixer and floor loop with pipes coloured by probe temperature
    (10 °C blue → sky blue → amber → 35 °C orange → 60 °C red, avoiding a purple midpoint); flame
    and pumps follow relays 8 and 7; zone dots follow relays 1–6; room names follow the thermostat
    DIs 1–4 (blue = calling for heat); bubbles move clockwise round a loop while its pump runs (redraw
    every 200 ms, otherwise every 2 s); mode pill and status word at the bottom. Preview it on a PC
    with `tools/screen_preview/render.py OUT_DIR`. Mode is fixed at MANUAL and status is derived
    from the relays until the heating logic sets them. Not yet seen on the real LCD; if the
    animation is jerky, try the SPI at 40 MHz (a full redraw takes ~120 ms at 10 MHz).
- `ufh-controller-wifi.yaml` is the production build. It includes the main config, swaps Ethernet
  for Wi-Fi (ESPHome can't run both at once), adds a "Wi-Fi signal" diagnostic sensor, and has no
  Wi-Fi password built in. The password is set
  after flashing, over USB (Improv on web.esphome.io) or through the "UFH Controller Setup" hotspot.
  Wi-Fi needs an antenna on the board's SMA socket, since the WROOM-1U has no PCB antenna. CI builds
  both variants.
- `components/scs_servo/` is a custom external component implementing the Feetech SCS protocol
  (big-endian registers, echo-tolerant half-duplex).
  - Registers: torque 40, goal position 42, present position 56, speed 58, load 60 (bit 10 is the
    sign, 0–1000), voltage 62 (0.1 V), temperature 63.
  - It is a PollingComponent with sensors, a `number` (goal position) and a `switch` (torque).
  - Lambda API: `set_goal_position()`, `set_torque()`, `ping()`, `is_online()`, `last_position()`,
    `last_load_percent()`.
- Fonts: DejaVu Sans, bundled in `fonts/`.
- `secrets.yaml` is not committed. `secrets.example.yaml` holds `api_encryption_key` and
  `web_password`.
- `.github/workflows/build.yml` compiles the firmware on every push and uploads the `.bin` files.
  It builds `secrets.yaml` from the repo's Actions secrets `API_ENCRYPTION_KEY` and `WEB_PASSWORD`.
  If those are missing (for example, a PR from a fork), it falls back to throwaway values and names
  the artifact `ufh-controller-firmware-untrusted`. Don't flash that one.

Verification so far:

- `esphome config` passes on ESPHome 2026.10.1.
- The servo C++ was syntax-checked on its own.
- CI compiles both the Ethernet and Wi-Fi variants.
- Bench test so far (Wi-Fi variant, flashed over USB then OTA): Wi-Fi, the encrypted API and OTA
  work, the TCA9554 (0x20) and RTC (0x51) respond on I2C, and the LCD test page displays.
  Relays 1–8 click from the web page, with their LEDs and the LCD following. The status LED is RGB
  order, not GRB. The servo answers at 115200 baud
  (position, load, voltage and temperature read correctly; about 300 polls at 50 ms with no errors).
  Movement checked over the native API (no load, no end stops): every move from 300 to 700 lands
  within 2–4 steps of the target, a 400-step move finishes within one 2 s poll, and the load reads
  about ±6–8 % while holding and 0 % with torque off. Writing a target with torque off still moves
  the servo, because the servo re-enables torque itself, so the torque switch can show OFF while
  the servo is holding. The built-in web server stopped answering HTTP once during testing, while
  the API kept working.
- DS18B20s: three probes on GPIO21, named Boiler flow, Return and Mixed flow by address. Identified
  by warming each one and labelled. They read room temperature (19–21 °C). The LCD shows all three.
- Servo link through the TXS0108E was unreliable (bursts of flipped bits, even with the servo idle),
  because the adapter's 3.3 V high is below the shifter's ~3.4 V threshold. Wired direct, it's
  clean (15/15 reads, no errors). The torque switch now reads back the servo's torque register, so
  it shows ON when a goal position re-enables torque.
- HA (2026.10.2 at 192.168.1.194) discovered and adopted the controller; new entities appear in HA
  after each firmware update without re-adding it. Removed entities (e.g. the old "Temperature
  probe 1") linger in HA as unavailable and have to be deleted there.
- Not confirmed yet: DIs. Ethernet is untested and no longer planned. Don't flash
  `ufh-controller.yaml` (the Ethernet build) to the installed board: it would drop off Wi-Fi with
  no network, and recovery needs USB-C.

## 5a. First hot commissioning test (6 Nov 2026)

Relays driven by hand over the API, mixer valve turned by hand (servo not yet coupled), 45 °C mixed
flow guard on CFH. Pump on speed setting 2 (of 3).

- **Supply:** opening all five zone valves at once blew the 1 A fuse on the 24 Vac circuit, and the
  controller (on the same circuit) lost power. Fitted 2 A, and opened the valves one a minute; no
  problem since. The heating logic must stagger zone valves; ideally the board gets its own supply.
- **Mixer direction:** clockwise (arrow right) = cooler, anticlockwise (arrow left) = hotter. Middle
  setting gave ~40 °C with the boiler at ~55 °C and return ~23 °C.
- **Timing:** heat reached the boiler-flow probe within a minute of CFH (17.8 → 30 °C in 1 min,
  45 °C in 2 min, ~55 °C after 6 min, ~62 °C after an hour), much faster than the 5 min expected
  (the boiler was probably warm). The return took ~3 min after hot water reached the floor to start
  rising, then climbed from ~16 °C to ~32 °C over an hour.
- **Disturbances seen (all at a fixed mixer position):**
  - The return warming pushes the mixed flow up (~+10 °C in a few minutes as the loops flush).
  - Radiators on another thermostat starting reduced the boiler-side pressure, so less hot water
    reached the mixer: mixed flow fell ~2 °C with the boiler flow unchanged.
  - Their cold water then reached the boiler: boiler flow fell ~63 → 51 °C over 5 min while still
    firing, and took ~15 min to recover; the mixed flow followed it down ~5 °C.
  - So no fixed valve position holds a temperature: the mixer needs closed-loop control (PID on
    the mixed probe, feed-forward from boiler flow and return).
- **Manifold:** one zone valve (first of the 10 pipes) is broken, so that loop stayed cold; a spare is
  to be fitted. Several other return pipes stayed cold for a long time: long runs or poor balancing.

## 5b. Mixer control (v0.2, from 7 Nov 2026)

The servo drives the mixer through part of the old actuator's gearbox (~3:1–4:1, mixer arm ~90°), so
it needs ~270–360° and runs in **continuous (motor) mode**, where it can't report an absolute
position. Control is therefore in two layers, both in `mixer_control.h`:

- **Controller** (every 5 s): target opening = feed-forward (setpoint − return) / (boiler − return) +
  PI trim on the mixed probe, with anti-windup (frozen while saturated or while the boiler is below
  setpoint + 2 °C), trim limited to ±30 %, target changes ≤ 5 % per update. Runs with "Mixer auto" on
  (default off) and the floor pump on, once the valve position is known.
- **Positioner** (every 1 s): estimates position from run time. Homes to cold (until an end is
  detected, or 1.3 strokes) when auto turns on, at every pump start and whenever the position is lost
  (jog, servo offline). Moves run for |target − position| × stroke time (max 5 s each), plus the
  backlash time when reversing; a 3 % deadband (≈35 moves/hour in simulation) limits servo wear.
  Reaching an end snaps the estimate; a target at an end overdrives once to re-sync.
- **Servo driver** (`components/scs_servo`): motor mode = angle limits 0/0 (regs 9–12, EEPROM, unlock
  via reg 48), drive = signed PWM in reg 44 (bit 10 = direction; FTServo `SCSCL::WritePWM`). Timed
  runs poll every 100 ms, log position/speed/current at DEBUG, and stop on a stall (current ≥ "Mixer
  stall current" for two polls after a 400 ms start-up) or on an end-of-travel hook for limit switches.
  In motor mode the load register just echoes the drive, so it can't detect a stall.
- **Over-temperature cut-out** (auto and manual): mixed > "Mixed flow limit" (45 °C) or no reading
  for 30 s → CFH (relay 1) off until 5 °C under the limit; in auto the valve also goes to cold.
- `tools/mixer_sim/sim.cpp`: safety checks, then controller + positioner against a plant modelled on
  the 6 Nov test with lever play and stroke-time errors. Average error ~0.3–0.5 °C, warm-up peak
  ~2.5 °C over setpoint, ~35 moves/hour.

**Unknowns for the bench** (not documented by Feetech/Waveshare): which reading shows a stall in
motor mode (current, reg 69, is the candidate), whether position still counts past one turn (the
wiki hints at ±7 turns), and the slowest drive power that still turns the valve. The gearbox's limit
switches (multi-lobe cam) can go on DI5–8: input changes are logged next to the run logs, which will
show the cam pattern; then wire them into `set_end_check` for homing without stalling.

**Bench results (7 Nov 2026, servo on the valve, 8.0 V supply, 30 % drive):**

- Motor mode set and read back (limits 0 / 0). The first attempts reported failure: an EEPROM write
  wasn't acknowledged within 20 ms; the mode change now waits 100 ms, retries and reads back.
- Positive drive turns the valve towards **hot** (Mixer reverse off).
- **Current (reg 69) always reads 0** on this SC15, and **load just echoes the commanded drive**, so
  neither can detect a stall.
- **Position**: the potentiometer covers ~210° (0–1023); for the other ~150° the reading sticks near
  0 or 1019 or gives meaningless values. Towards hot the reading counts **down**; on leaving the gap
  it reappears at the other end (it wrapped 0 → gap → 1019 after one turn), so **whole turns can be
  counted** in software. Speed (reg 58) is real inside the window.
- **Stroke**: cold end → hot end ≈ 8 s of drive at 30 % (8 × 1 s jogs, the first partly spent on
  play and the gap), ≈ 1⅓ servo turns (~430°), so the gearbox is nearer 5:1 or the valve turns
  more than 90°. Each 1 s jog coasts ~10–18° after stopping.
- **Ends**: the hot end falls inside the sensor window (stopped near 677 on the second pass, speed
  falling to −100 as it arrived), so a speed-based stall is visible there. The cold end falls in or
  near the gap (a stalled reading of 811 was not a real position), so it can't be seen: the cold end
  needs a limit switch.
- **Friction** varies with angle, with stiction near the middle; free speed ~450–500 steps/s,
  ~150–300 in the stiff region. The lever has some play.

- **Re-indexed** (7 Nov): the servo was set to raw 512 (window centre) with the valve 1/3 open and
  re-attached, so the usual working range (~10–55 % open) is in the window, the hot end in the next
  window, and the cold end ~35° into the gap. Limit switches are not being wired (no room).

**Positioning (v0.3, `mixer_control.h`):** a `Tracker` follows the shaft on an axis of sensor counts
(1755 per turn): exact inside a window, estimated from the last measured speed across the gap.
It leaves the gap only on readings that move consistently the right way (2–4 in a row, fewer from
the expected edge), only once the estimate is at least a quarter of the way across, and on
probation: if the next two readings don't carry on, the exit is undone (random or stuck gap readings,
like the 811, never pass). The `Positioner` moves closed-loop on that position: each move starts with
a 0.2 s kick (60 %) to break stiction, runs at the drive power (30 %), and is stopped "Mixer coast"
counts short of the target; the gap estimate holds while coasting. Homing is a timed cold run (the
cold end is in the gap, so it can't be seen); calibration then drives hot until the sensor shows a
stall, measuring the cold end (from where the reading first leaves the gap) and the stroke. In the
simulator (random gap readings, stiction, coast, lever play up to 30°) it calibrates within ~10 %,
holds the mix within ~1 °C, and makes ~40 moves/hour when steady.

**Position tests (7 Nov, after re-indexing; "Mixer manual opening" with auto off):**

- Calibration found the hot end exactly (stall at raw 677, x = 2086) but put the cold end at −394,
  ~400 counts too far back: the cold end sits at the top edge of window 0, where readings stick and
  jump. Corrected by eye.
- Lever play is large: reversing towards cold, the shaft moved to 33 % while the valve stayed at
  50 %, so play ≈ 350 counts (~17 % of the stroke, ~70° of servo).
- **Settings now in use:** Mixer cold end −175 (0 minus half the play), Mixer stroke 2086, Mixer play
  350, Mixer home time 10 s, drive 30 %, kick 60 %, coast 50, deadband 3 %.
- Checked by eye: 20 % → ~20 %, 50 % → exactly 50 % (in the gap), 75 % → ~80 % (deep in the gap),
  then reversing: 33 % → ~33 %, and back up to 50 % → exactly 50 %.
- Fixed on the way: "stopped on a stall" is decided when the move stops (the shaft springs back after
  a stall), and each run's first reading no longer counts idle time as driving time.
- To do: make calibration's cold-end estimate reliable (or keep setting it by eye), then test Mixer
  auto with heat.

**First heated test with Mixer auto (7 Nov, setpoint 40 °C; the failed zone solenoid had been
replaced, so all 10 circuits should flow):**

- Sequence worked: valves one a minute, pump (mixer homed, then "Waiting", opening towards hot), CFH.
- The boiler was already firing for the radiators, so the boiler flow went 18 → 33 °C in the first
  minute and ~50 °C in the second. With the valve fully open, the mix overshot to **~48.8 °C**: the
  board's cut-out fired (status Over-temp, valve driven to cold) and the script's 47 °C backup cut
  CFH. The over-temp latch cleared once the mix fell below 40 °C.
- **Faults found:**
  1. While "Waiting" the valve opened fully: pointless while the boiler is cold, risky when heat
     arrives suddenly. Should hold the opening a typical boiler temperature would need (~50 %).
  2. Opening changes were limited to 5 % per 5 s: far too slow for a boiler rising ~30 °C/min.
  3. Position at the ends: short pushes into the hot end never gave the tracker enough moving
     readings to leave the gap, so it stuck at the gap edge (84 %) while the valve was fully open, and
     kept pushing (slight hunting). After the over-temp drive to cold the estimate was wrong too
     (reported 84 % with the mix at 26 °C). Timed runs to an end must reset the position, as homing
     does; homing must also allow for the play (it reports 8.4 % at the cold end).
- At the hot stop the valve's own scale reads about 80 %.
- Test stopped: pump, valves and Mixer auto off.
- **Fixes (v0.4):** while the boiler is cold or warming, the blend uses a predicted boiler temperature
  (current + 2 min of its rate of rise, capped at setpoint + 20 °C) instead of the current one; while
  it's rising faster than 2 °C/min the trim may close but not open past the blend, and doesn't
  integrate towards hot; closing is no longer rate-limited when the mix is > 2 °C over the setpoint;
  a run to an end resets the position to that end (allowing for the play), and homing allows for the
  play. Simulated with a cold boiler and with today's hot one: peak mix 41–43 °C (was 48–50).

**Heated retest (7 Nov, v0.4):** homing now reads 0 %; while waiting the valve held at 54 % (matched
by eye) instead of fully open, and the mix stayed below target while stored heat came through. Then
the mix jumped 31 → 48.8 °C with the boiler flow steady at ~59 °C; the board's cut-out fired and
the backup cut CFH. Jon saw the valve at the hot stop while the firmware said 54 %, then 0 %, then
74 %; the servo's own reading (586) put the shaft about one turn further than the tracker thought.
So the valve (and the servo shaft with it) moved while the servo wasn't driving, and the tracker,
which only followed movement during runs, lost a turn. Whether the flow pushed it is unconfirmed.
Test stopped (pump, valves, auto off).

**v0.5:** the servo driver now reads the position 4×/s while idle, and the tracker follows movement
it didn't cause, including through the gap (direction taken from the readings; only clear exits
from the expected edge accepted). Such movement is logged as "Valve moved by itself", and the
controller drives the valve back. Simulator: tracks an idle push through the gap and back within
30 counts; 1000 s of random gap readings while still invent no exit.

The cam limit switches (DI5–8) would give absolute reference points and remove the risk of losing
a turn; Jon is reconsidering them despite the lack of space.

**Hand test (7 Nov, 19:00):** the servo/gearbox can't be turned by hand (self-locking), so flow can't
back-drive the servo. But the valve lever moves freely between ~33 % and ~50 % with the servo
still: the ~17 % lever play. Inside that band nothing holds the valve, so water forces can float it
to either side; that explains the retest's 31 → 48.8 °C jump with the servo stationary (in the
steep middle of this valve, 17 % is ~17 °C), and breaks the firmware's assumption that the valve
trails the shaft in the last direction moved. **This needs a mechanical fix** before more heated
tests: Jon is dismantling the linkage to reduce the play, and looking for a light preload spring
to hold the lever on one side of whatever play remains (then set Mixer play to 0). After reassembly:
check the servo index (raw ~512 at ~1/3 open), redo the cold end / stroke / play settings, and
re-run the position checks before heating. Decided: try without the limit switches for now.

**Rework and cam switches (7 Nov evening):** Jon shimmed the slop out of the gearbox coupling (a few
degrees remain in the valve's own shaft, after the cam) and wired the old actuator's four cam
switches to DI5–8 (two NO, two NC). Drive train: servo → gearbox (~4.5:1) → output shaft, shimmed
to the cam → cam doubles as the coupling to the valve lever → valve shaft. Some bracket flex is
visible under load.

- Read through the binary sensors, switch-ON changes arrived up to 1 s late (on a 1 s rhythm);
  read directly from GPIO8–11 at every servo reading they repeat to ~10–25 counts. Cause of the
  binary-sensor delay not yet found (DI1–4 thermostats use the same path; harmless there).
- The four switches form a code that changes one bit at a time (DI5 DI6 DI7 DI8, going hot):
  0101 (cold end) → 1101 at x≈65 → 1100 at 645 → 1110 at ~1025 (in the gap) → 1010 at ~1795 →
  1011 at 1956 (hot end). Going cold the same edges come at 1885, 1757, 945, 535 and 0. DI5 off is
  the cold end switch and DI8's last change the hot end switch; DI6/DI7 were auxiliaries.
- **v0.6:** the positioner snaps to the exact position at every cam edge, checks the tracked
  position against the cam zone (and starts from it after a restart), and uses the end switches
  for the ends (homing stops at the cold end switch; no more pushing on the stops). The range is
  now switch to switch: Mixer cold end 0, Mixer stroke 1956, Mixer play 0. Simulator (cam model,
  hot and cold boiler): mix within ~0.6 °C, peak ~42.5 °C, and a deliberately lost turn is
  corrected by the zone check within seconds.

**Heated test with the cam (7 Nov, 21:34–22:12), and the valve curve:**

- Cold boiler (high thermal mass, slow): the valve waited at ~45–55 %, the mix rose slowly with the
  boiler and settled at 40 °C with the valve ~38–41 %. When the boiler passed 42 °C the blend
  switched from "boiler will reach 60 °C" to a rate-based prediction (~46 °C) and jumped up, opening
  the valve while the mix was already over the setpoint (peak 44.7 °C, no trip); the trim then
  brought it back to ~41.8 °C. Fix: no jump at that hand-over; and while waiting, still close if the
  mix goes over the setpoint (it ignored the mix then).
- Cam inputs glitch at idle (all four, or DI5+DI6 together, for ~0.4 s): wiring (rushed, to be
  tidied). Planned: act on a cam change only once it has held for two readings.
- **Valve curve** (`docs/valve_curve_2026-11-07.png`): stepped 0→25 % (75 s per step, Mixer auto
  off) until the mix passed 45.5 °C at the start of the 30 % step (boiler had reached ~65 °C; the
  board's cut-out fired at 45.2 °C). Boiler-water share: ~17 % at 5–10 %, 20 % at 15 %, 30 % at
  21 %, 44 % at 27 %; readings during the automatic run (moving cold) give ~71 % at 32 %, ~85 % at
  35 %, ~97–100 % at 38–45 %. So in the switch-to-switch frame the valve does nearly all its mixing
  between ~10 % and ~40 % and is effectively fully hot above that, which the linear blend formula
  gets badly wrong. Next: put this curve into the estimate (invert it), lower the gain for the
  steep part, and consider narrowing the working range.

**v0.7 (branch `valve-curve`, not flashed):** control in terms of boiler-water share, converted to
an opening through the measured curve; smooth warm-up hand-over (the "boiler will reach setpoint +
20 °C" assumption fades out between setpoint and setpoint + 10 °C); while waiting, still cut back if
the mix is over the setpoint; cam changes count only after two readings; after a move, reversing
towards hot needs twice the deadband for 30 s (it hunted between two positions because the smallest
move, kick plus coast, exceeds the deadband); kick time configurable. Simulated with the measured
valve: mean error ~1 °C (vs ~13 °C assuming linear), but warm-up peaks still 50–57 °C and a valve
15 % steeper than measured isn't controlled. The simulator's servo model (kick, coast, stiction) is
a guess.

**The real limit:** nearly all mixing happens in ~30 % of the travel (~600 counts); ±1 °C needs the
valve placed to ~±10 counts, but the smallest servo move is ~50–130 counts. Likely cause: low valve
authority (boiler side has far more pressure than the floor loop; 97 % boiler water at 38 % open).
Options: (1) a regulating/lockshield valve on the mixer's hot inlet, partly closed, to spread the
mixing over more travel (standard fix); (2) measure the servo's smallest reliable move on the bench
(short pulses, with/without kick, different powers) and model it; low-power "creep" moves.

**Band control (Jon's proposal, simulated 7 Nov; `tools/mixer_sim/band_sim.py`,
`docs/band_control_sim.png`):** don't regulate the mix to ±1 °C. HEAT: CFH on, valve set coarsely
so the mix can't exceed the top of the band even with the boiler at its hottest (70 °C), re-set only
when that opening drifts > 3 %. FREE-WHEEL (mix reaches the top): CFH off; the valve limits the mix
to the top of the band for the boiler's actual, falling temperature, so the leftover heat is used.
Back to HEAT at the bottom. 8 h with the floor warming (simple plant, measured valve curve, 2 min
upstream delay, boiler thermal mass): band 35–42 °C gives 1.2 CFH cycles/h (~48 min), 9 servo
moves/h, mix 30–43 °C, never over 45 °C, and 98 % of the heat of fine control (which needs ~90
moves/h). Wider/lower bands deliver less heat. Not yet in the firmware.

**Boiler-flow cap (simulated 7 Nov, `docs/band_control_cap_sim.png`):** within the heating phase, also
drop CFH when the boiler flow reaches a cap, restarting some degrees below it; the valve assumes
the boiler reaches cap + 8 °C or its actual temperature if higher (radiators/hot water may still
drive it to 60–70 °C). The boiler keeps heating ~2 min after CFH drops, so the peak is ~cap + 7 °C.
Results (band 35–42 °C, 8 h): no cap 48 min cycles, boiler peak 68 / mean 50 °C, heat 195;
cap 50 / restart 5 below: 19 min cycles, peak 57 / mean 49, heat 190; cap 50 / 10: 27 min, mean 46,
heat 179; cap 45 / 10: 30 min, peak 53 / mean 42.5, heat 184, ~6 servo moves/h. So a cap lowers the
peaks (and with a wide restart gap the mean), at the cost of shorter CFH cycles and a few % less heat
per hour. A lower flow setting on the boiler itself, if it has one, would do the same without the
cycling.

**Commissioning (servo on the valve, Mixer auto off):**

1. Press "Servo motor mode" (once; it's stored in the servo). Check the log shows limits 0 / 0.
2. "Mixer jog hot" / "Mixer jog cold" run 1 s at "Mixer drive power" (30 %). Check the valve turns
   the right way (flip "Mixer reverse" if not) and that the power is enough under load. Note the
   current values in the log while running freely and when pushed against an end.
3. Set "Mixer stall current" between those (or wire the limit switches), then "Mixer calibrate":
   cold end → hot end (timed) → back to cold. "Mixer stroke time" is set from it.
4. Measure the lever play (jog one way, then count the run time on reversal before the valve moves)
   and set "Mixer backlash".
5. Set the setpoint, start the pump, turn Mixer auto on: it homes, then controls. Watch Mixer opening
   and the mixed flow; lower "Mixer gain" if it hunts, shorten "Mixer integral time" if it is slow.

## 6. Environment notes (for Claude sessions)

- In the cloud workspace, install ESPHome with `python3 -m venv .venv && .venv/bin/pip install esphome`.
  System pip is blocked.
- The cloud workspace **can't download the Espressif toolchain or Google Fonts** (proxy 403), so
  `esphome compile` fails there. Validate with `esphome config` and rely on CI for full builds.
- Git push needs the repo attached to the session. Personal access tokens are overridden by the
  session's GitHub proxy.
- Jon's Mac can be linked to the session, and the built-in browser works there (waveshare.com
  allowed).

## 7. Open questions and next steps

**Questions for Jon:**

- [ ] How is HA installed: HA OS/Supervised (ESPHome add-on available) or Container/Core? (HA 2026.10.2 runs at 192.168.1.194:8123; install type still unknown.)
- [x] Relay wiring (6 Nov 2026): relay 1 boiler CFH, relay 2 floor circulation pump, relay 3 spare,
      relays 4–8 five zone valves (which zone each feeds is still to be mapped). DI1–4 are the
      Playroom, Kitchen, Booth and Lounge thermostats. The LCD still uses the old plan (8 CFH,
      7 pump, 1–6 zones) and needs updating.
- [x] What logic level does the Bus Servo Adapter A UART use? 3.3 V (TXD 5 V tolerant), so no level shifter.
- [ ] NEST → DI interface. Proposed (31 Oct 2026): wire each thermostat's W straight to its DI and
      the 24 Vac transformer's C to the DI common, with a 1 kΩ ¼ W series resistor per input.
      - AC itself is fine: the inputs use bi-directional optos (anti-parallel LEDs), so both half
        cycles light one, with no reverse-voltage stress. The output drops briefly at each zero
        crossing (every 10 ms); the 50 ms `delayed_on_off` filter hides that.
      - The 5–36 V rating is the limit: 24 Vac peaks ~34 V, but a lightly loaded transformer can sit
        at 26–28 Vac (37–40 V peak). Under ~25.5 Vac measured open-circuit is within spec. The
        internal series resistor is probably ~3.3 kΩ (implied by 5–36 V), so 1 kΩ extra brings a
        40 V peak back under the 36 V current; check the AC current per input (a few mA).
      - All thermostats and the DI common must share one 24 Vac transformer.
      - NEST needs a C wire. Power-stealing pushes pulses through W when off, and a few-mA opto
        input may read them as "calling". Measure W–C with the thermostat off (should be ~0 V).
        Without C wires, use a 24 Vac relay per zone (a proper load for the NEST) and feed its dry
        contact to the DI instead.
      - Keep this low-voltage wiring away from the 240 Vac pump/boiler wiring in the cabinet.
- [ ] Which DS18B20 positions: flow (after the mixer) is essential, and return is recommended.
      Anything else, such as the boiler flow side?

- [ ] Servo coupling (7 Nov 2026): Jon will drive the mixer through part of the old actuator's
      gearbox (~3:1–4:1). The mixer arm turns ~90°, so the servo needs ~270–360°, beyond the SC15's
      ~210° position mode. Options: SC15 in continuous (wheel) mode with floating control and stall
      homing, or a multi-turn magnetic-encoder servo (e.g. ST3215) keeping absolute position. The
      current `mixer_control.h` assumes absolute position. **Decided:** SC15 continuous mode with
      run-time positioning (see 5b); the cam limit switches can go on DI5–8.

**Next work:**

1. ~~Push v0.1, let CI compile it, and fix any build errors.~~ Done.
2. Bench test: flash over USB-C, check Wi-Fi, HA discovery, relays, DIs and the LCD. (Ethernet dropped.)
3. Servo bring-up: ping, read position and load, move it, then tune the response timeout and baud.
4. Limit-finding routine: drive slowly to each end until the load rises, store the end-stop
   positions in flash, and map 0–100 % mixer opening onto that range. Use the goal-speed register
   (46) to creep during calibration, and keep normal moves a little short of the stops.
   **Polling design (agreed):** poll the servo every 2 s while idle and about every 100 ms while it
   moves or calibrates. The fast loop stays on the device. HA gets only filtered values on a slower
   schedule (e.g. position change > 2 steps, load change > 5 %, plus a periodic heartbeat), so the
   HA database doesn't fill with repeated readings. The servo is fast (under 2 s for 400 steps);
   that's fine for control, since the thermal loop is much slower, and speed can be capped later.
**Temperature sensors (planned): three DS18B20s at the mixer's ports.** Boiler flow (hot in),
return (cold in, from the manifold) and mixed flow (out to the manifold). Mixed flow is the PID's
control input and drives the over-temperature cut-out. Boiler flow and return show how much heat the
floors take and whether the boiler is working.

**Plant timing (from Jon, 30 Oct 2026):** the boiler has a high thermal mass and is some distance
away, and its CFH has a hysteresis of 1–2 minutes, so it takes about 5 minutes from a thermostat
call to hot water at the mixer. The zone actuators take about 2 minutes to open. Design consequences:
- Boiler fault: judge on boiler flow *rising* after a grace period (e.g. 15 min), not on a fixed
  temperature soon after the call.
- PID: hold it while boiler flow is below setpoint + margin, to avoid integral windup, and reset it
  when it starts. Pre-position the valve from all three temperatures, roughly
  (setpoint − return) / (boiler flow − return), and let the PID trim, to limit overshoot when hot
  water arrives.
- Sequencing: zone valves open → wait for the actuator opening time → pump on → boiler CFH. When
  demand ends: drop CFH, pump overrun to carry away stored heat.
- Over-temperature cut-out stays immediate.
- All timings (actuator opening, pump overrun, boiler grace period) are adjustable from HA.

5. Heating logic on the device:
   - zone demand → solenoids
   - pump on when any zone is open (with overrun)
   - boiler CFH only while the pump runs
   - PID on flow temperature → mixer position
6. Fail-safes:
   - flow over-temperature cut-out (close the mixer, drop CFH)
   - safe behaviour on sensor loss or servo offline
   - watchdog defaults while HA is unreachable
7. LCD: map the zone dots to relays 4–8 once the zones are known; have the heating logic drive the mode (AUTO / OVERRIDE / MANUAL) and status (Warming, Holding, Waiting…).
