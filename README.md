# Cooling system — ESP32-C3

This branch targets C3; `main` targets S3. Two turbine flow sensors and four
NTC temperatures are sent over classic CAN at 1 Mbps. The C3 has no PCNT,
so flow counting and timestamping use GPIO rising-edge interrupts.

## Pinout and hardware

| Signal | C3 GPIO |
|---|---:|
| Flow 1 / Flow 2 | 3 / 4 |
| ADS1115 SCL / SDA | 2 / 1 |
| CAN TX / RX | 6 / 7 |
| Native USB D− / D+ | 18 / 19 |

Flow inputs avoid boot straps GPIO8/9. GPIO2 is also a strap; keep SCL pulled
high externally at reset. Confirm these pins are free on the actual C3 board.
Use native USB for programming/logs and share ground between connected devices.

- Turbines: **390 pulses/litre**, 2–60 L/min (13–390 Hz). A 5 V output needs the
  10 kΩ series / 20 kΩ to-ground divider before the MCU input.
- ADS1115: 3.3 V, I2C address **0x48**, external pull-ups. Each Analog1–4 NTC
  uses a **510 Ω pull-up to 3.3 V**, NTC to ground, ADC at the junction.
  Conversion uses the SNSR-02101CK manufacturer table (−20 to 160 °C).
- CAN needs a transceiver with 3.3 V MCU-side logic and termination. Never
  connect a 5 V signal directly to an MCU GPIO or the 3.3 V ADC.

## Build and checks

Use ESP-IDF 5.5.2+ with the C3 toolchain and Python tooling in the project Conda
environment. From the repository root in an ESP-IDF-enabled shell:

```sh
conda activate cooling-system
idf.py set-target esp32c3
idf.py build
idf.py -p <native-usb-port> flash monitor
```

Exit the monitor with Ctrl+]. Host checks:

```sh
make -C test clean check
sh test/check_flow_sensor.sh
sh test/check_can_tx.sh
```

The GPIO check is versioned. Existing calculation/CAN test files remain
Git-ignored local files and must be supplied separately for a fresh clone.

## Architecture and logs

- `flow_sensor.c`: short, IRAM GPIO handlers reject edges less than 1 ms after
  the last accepted edge, increment per-channel totals, then queue timestamps.
  Rejected edges do not move the reference; queue overflow loses timestamps,
  not accepted pulse counts. Counts are read under a lock. This edge-spacing
  filter does **not** measure pulse width like the old PCNT glitch filter.
- `flow_calc.c`: the task derives rate from intervals and zeros it after more
  than 3 s without an accepted timestamp; restart needs two edges. Volume is
  count / 390. Totals reset on reboot and saturate at `INT_MAX` (about 63 days
  continuously at 390 Hz). Prolonged interrupt blocking can lose pulses;
  unlike PCNT, counting depends on the CPU servicing each edge.
- `ads1115.c` / `temp_calc.c`: sequential 8 SPS conversions, then a 500 ms
  pause; a healthy four-channel sweep takes roughly 1 s. Runtime read failures
  and open/short/off-curve readings invalidate temperatures without stopping flow.
- `can_tx.c`: a 100 ms timer snapshots values and uses DBC-generated codecs.
  Pending buffers remain owned until TX completion. Recovery is requested once
  per bus-off event. Keep generated `cooling_system.c/h` unchanged by hand.

USB logs: `flow` reports **channel 1 only**, once per second (`pulses`, L/min,
L); `adc_sampler` displays **Analog1/2 only**, with counts/volts/°C or warnings
(all four channels are still sampled and published);
`ads1115` reports pin/ACK checks; `cooling_system` reports CAN startup/failures.
Successful transmissions are not printed. Verify channel 2 at the receiver.

## CAN and bench testing

Temporary contract: `dbc/cooling_system.dbc`. Standard IDs, 8-byte big-endian
payloads; coordinate any changes with receivers.

| ID | Period | Payload (zero-based byte offsets) |
|---|---|---|
| `0x100` / `0x101` | 100 ms | Flow 1/2: unsigned 16-bit rate at 0 × 0.01 L/min, unsigned 32-bit volume at 2 × 0.001 L; bytes 6–7 unused |
| `0x102` | 500 ms | Four signed 16-bit temperatures at 0/2/4/6 × 0.01 °C |

Any invalid/unsampled temperature suppresses the entire TEMP frame; there is
no status signal or sampler-stall watchdog. The sibling Rust logger must load
its config from `/sdcard/config.json`. It repeats latest values in 500 Hz SD
rows without expiry, so missing TEMP leaves stale temperatures. This node has
no SD logging. Zero flow cannot distinguish stopped coolant from a dead sensor.

1. Connect a CAN receiver that **ACKs** and 120 Ω termination at both bus ends.
   A listen-only receiver will not ACK; solo-bus self-test is not enabled.
   Boot: expect ADS1115 ACK at `0x48`, CAN TX=6/RX=7. No ADC ACK: check rail,
   ground, address and external pull-ups.
2. Disconnect turbine outputs; drive 3.3 V square waves at GPIO3/4 with common
   ground. 65/130 Hz should give **10/20 L/min** and volume increments of about
   **1.667/3.333 L over 10 s**. Test 13/390 Hz (2/60 L/min), stop for ≥4 s,
   restart and swap channels. Repeat under ADC/CAN load and compare pulse totals
   against the generator. The sibling `cl-devboard` outputs GPIO4/5; wire them
   to this board's GPIO3/4 respectively.
3. With all four ADC inputs valid, substitute a measured 2.83 kΩ resistor for
   one NTC: expect about **2.796 V / 25 °C** at 3.3 V supply/510 Ω bias. Warm an
   NTC: voltage/resistance falls, °C rises. Open/short its sensor leg: expect a
   warning and TEMP suppression after the next sweep; flow continues. Restore
   it and verify TEMP returns. Check receiver values and logger SD data.

Generated pulses verify decoding, not turbine calibration or the 5 V divider.
Below 2 L/min is outside sensor spec. NVS calibration and pressure are not implemented.
