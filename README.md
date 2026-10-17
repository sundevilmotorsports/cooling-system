# Coolant monitor — ESP32-C5

`main` targets ESP32-C5 with ESP-IDF 5.5.5. The separate `c3` branch uses
GPIO counting instead of PCNT.

| Signal | C5 GPIO |
|---|---:|
| Flow 1 / Flow 2 | 2 / 1 |
| ADS1115 SCL / SDA | 8 / 7 |
| CAN TX / RX | 5 / 6 |

This maps the existing C3 Super Mini socket to the C5 Mini with the USB/power
ends aligned. C3 flow GPIO1/0 positions become C5 GPIO2/1; the four I2C/CAN
positions keep their GPIO numbers. The C5's extra RST pin shifts that side's
digital pins. Other unused C3 positions also differ: GPIO4 becomes RST,
GPIO3/2 become GPIO4/3, and GPIO20/21 become GPIO12/14. Extra C5 GPIO0/15
extend beyond the C3's eight-pin rows; check mechanical clearance.

PCNT counts rising edges independently for each flow channel, with a 1.5 µs
hardware glitch filter and overflow accumulation. Volume is count / 390 litres.
GPIO interrupts queue timestamps for rate calculation; the rate rejects edges
less than 1 ms apart and returns to zero after 3 seconds without an accepted
edge. A full timestamp queue can affect rate but does not stop PCNT volume
counting. The rate timestamps do not pass through PCNT's hardware glitch filter.
The driver's signed 32-bit total can overflow after about 64 days at 390 Hz;
totals reset on reboot.

Classic CAN runs at 1 Mbps: flow IDs `0x100`/`0x101` every 100 ms and temperature
ID `0x102` every 500 ms. Invalid temperatures retain each channel's last valid
value; TEMP waits until all four channels have had a valid reading. Serial
warnings include ADC read errors or raw counts/voltage for invalid readings.
CAN has no temperature validity or freshness indication, so retained values
can be stale during faults.

In an ESP-IDF shell using the project's Conda environment:

```sh
conda activate cooling-system
idf.py set-target esp32c5
idf.py build
```

Host checks (a working native C compiler and SDK are required):

```sh
make -C test -B check
sh test/check_pcnt_flow.sh
sh test/check_can_tx.sh
```

The PCNT check mocks the SDK around real firmware; it verifies initialization,
channel routing, queue-full behavior, wide driver counts and startup errors.
It does not emulate the hardware filter or prove physical pulse reception.
Existing calculation test files remain Git-ignored local files.

Bench check both 3.3 V flow inputs: 65/130 Hz should report 10/20 L/min;
390 pulses should add 1 litre. Stop for more than 3 seconds and restart to
check timeout/recovery. Run 390 Hz for 90 seconds to cross the PCNT high limit
(32767) and confirm the total remains continuous at about 35100 pulses/90 litres.
USB flow output displays channel 1 only; check channel 2 at the CAN receiver.
