# UFH Controller

ESPHome firmware for an underfloor-heating manifold controller, integrated with Home Assistant.

- **Controller:** Waveshare ESP32-S3-POE-ETH-8DI-8RO (ESP32-S3-WROOM-1U-N16R8, W5500 Ethernet + PoE)
- **Mixer actuator:** Waveshare SC15 bus servo (Feetech SCS protocol) via Bus Servo Adapter A
- **Temperatures:** DS18B20 one-wire probes
- **Display:** Waveshare 2.4" LCD module (ILI9341, 240×320)
- **Loads:** 24 Vac zone solenoids, 240 Vac pump, 240 Vac boiler call-for-heat (all via the onboard relays)
- **Inputs:** NEST thermostats (24 Vac) via the opto-isolated digital inputs — *interface circuit to be designed*

> Status: v0.1 — hardware layer only (relays, inputs, sensors, servo driver, display test page).
> Heating logic is not implemented yet. All relays start OFF.

## Repository layout

| Path | Purpose |
|---|---|
| `ufh-controller.yaml` | Main ESPHome configuration |
| `components/scs_servo/` | Custom ESPHome component for Feetech SCS bus servos |
| `fonts/` | DejaVu Sans fonts for the LCD (bundled, see `LICENSE-DejaVu.txt`) |
| `secrets.example.yaml` | Template for `secrets.yaml` (not committed) |
| `docs/PROJECT_CONTEXT.md` | Background, decisions, open questions — read this first |

## Pin map

### Fixed by the board

| Function | GPIO |
|---|---|
| Digital inputs DI1–DI8 | 4, 5, 6, 7, 8, 9, 10, 11 |
| Relays 1–8 | TCA9554 expander @ 0x20, pins 0–7 |
| I2C (expander, RTC PCF85063) | SDA 42, SCL 41 |
| W5500 Ethernet (SPI2) | SCLK 15, MOSI 13, MISO 14, CS 16, INT 12 |
| RS485 | TX 17, RX 18 |
| RGB LED (WS2812) | 38 |
| Buzzer | 46 |
| BOOT button | 0 |
| TF card | 45, 47, 48 — **not used; 47/48 reused for the LCD** |

### 28-pin header (2×14)

```
Top:    5V   5V   G    G    3V3  G    SCL  SDA  3V3  G    21   45   47   48
Bottom: Vbus G    DP   DN   3V3  G    Tx   Rx   3V3  G    0    1    2    3
```

### Project allocation

| Header pin | GPIO | Use |
|---|---|---|
| 21 | 21 | DS18B20 one-wire bus (4.7 kΩ pull-up to 3V3) |
| 48 | 48 | LCD **CLK** (SPI3 SCLK) |
| 47 | 47 | LCD **DIN** (SPI3 MOSI) |
| 3 | 3 | LCD **DC** |
| Tx / Rx | 43 / 44 | Servo UART, wired direct → Bus Servo Adapter **TX** / **RX** |
| 1, 2 | 1, 2 | Spare |
| SCL / SDA | 41 / 42 | Shared I2C bus — available for extra I2C devices |
| 45 | — | Strapping pin — do not use |
| 0 | LCD RST | BOOT button / strapping pin; high at power-on, so safe as the LCD reset |

## Wiring

### LCD (Waveshare 2.4" ILI9341) — plug-in jumper wires

| LCD pin | Connect to |
|---|---|
| VCC | header **3V3** (sets 3.3 V logic — do *not* use 5V) |
| GND | header **G** |
| DIN | header **47** |
| CLK | header **48** |
| CS | header **G** (display is the only device on this bus) |
| DC | header **3** |
| RST | header **0** (GPIO0). Needed: with CS tied low the panel stays white without a hardware reset |
| BL | header **3V3** (backlight always on) |

Remove any TF card from the board — its slot shares GPIO47/48.

### DS18B20 probes

All probes in parallel: red → 3V3, black → G, data (yellow) → header **21**, with one 4.7 kΩ resistor
between data and 3V3. Each probe's 64-bit address is printed in the ESPHome log at boot; add it to
`ufh-controller.yaml` under `sensor:` with a meaningful name (flow, return, …).

### Bus Servo Adapter A

- Jumper on **A** (UART mode). **B** is USB mode, used only with `tools/scs_tool.py`.
- Straight through, as Waveshare documents ("RX-RX, TX-TX"): header **Tx** (GPIO43) → adapter
  **TX**, header **Rx** (GPIO44) → adapter **RX**, header **G** → adapter **GND**. **No level
  shifter:** the adapter's logic runs at 3.3 V (its own AMS1117), RXD drives 0–3.3 V, and TXD is
  5 V tolerant. A TXS0108E shifter was tried and made the link unreliable (its 5 V side needs
  ~3.4 V for a high).
- **Baud rate 115200.** The servo's stored baud rate was changed from the factory 1 Mbps
  with `tools/scs_tool.py PORT set-baud 115200` (adapter jumper on B, adapter USB to a computer,
  ESP32 wires unplugged). A new or replacement servo needs the same change. `scs_tool.py PORT scan`
  finds a servo's current baud rate and ID.
- Servo supply: SC15 is rated 4.8–8.4 V, and the adapter passes its input straight to the servo.
  The bench test used 5.2 V; 6–7.4 V gives more torque and keeps the adapter's 3.3 V regulator
  (which needs ~4.4 V in) clear of motor-current sags. Power the adapter from its own supply, not
  the board's 5V header pin; only ground is shared.

### Power

7–36 V **DC** terminal or PoE (IEEE 802.3af). Do not connect 24 Vac to the DC terminal.

## Building and flashing

### First flash (USB-C)

1. Copy `secrets.example.yaml` to `secrets.yaml` and fill in the values.
2. In Home Assistant → ESPHome Device Builder, add this project (or build locally with
   `esphome run ufh-controller.yaml`).
3. Connect the board's USB-C to a computer with a **data** cable and choose *Install → Plug into
   this computer*. If the port is not detected, hold **BOOT** while plugging in.

### Later updates

Over the network (OTA) from the ESPHome Device Builder: *Install → Wirelessly*.

## Servo component (`scs_servo`)

Implements the Feetech SCS (SCSCL) protocol, half-duplex echo tolerant.

| Entity | Register |
|---|---|
| Position (0–1023) | 56 |
| Speed | 58 |
| Load (±100 %) | 60 |
| Voltage | 62 |
| Temperature | 63 |
| Target position (number) | 42 (goal position) |
| Torque (switch) | 40 (torque enable) |

Lambda API: `id(mixer_servo).set_goal_position(n)`, `set_torque(bool)`, `ping()`, `is_online()`,
`last_position()`, `last_load_percent()`.

## Roadmap

- [ ] Confirm Bus Servo Adapter A logic level; bring up servo comms
- [ ] Limit-finding routine (drive to each end stop, detect via load, store limits)
- [ ] Assign relays: zones, pump, boiler CFH
- [ ] NEST call-for-heat input interface (24 Vac → DI)
- [ ] Heating logic on-device: zone demand → solenoids, pump, CFH; PID mixer on flow temperature
- [ ] Fail-safes: flow over-temperature cut-out, sensor-loss handling, pump overrun
- [ ] LCD status pages
