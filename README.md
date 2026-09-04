# ESP32 DNC Firmware

Industrial **DNC over Wi-Fi** for CNC machine tools: an ESP32 that sits on the machine's RS-232 port, stores G-code programs on a microSD card, and serves an embedded web interface so the operator can send, receive and drip-feed programs from any browser on the shop floor.

FreeRTOS, six concurrent tasks, hardware and software flow control, no cloud, no PC required.

Built by [Kaiki Quadros Ferreira](https://github.com/dev-kaiki) at SMI, published with the company's permission.
Companion project to [`dnc-serial-engine`](https://github.com/dev-kaiki/dnc-serial-engine) — the same problem solved on a desktop, and here on bare metal.

---

## Why this exists

Sending a program to a CNC controller traditionally means walking a laptop to the machine, plugging in a serial cable, and hoping the flow control settings match. Machines that predate USB are still cutting parts every day, and they will outlive several generations of the laptops used to feed them.

This replaces the laptop with a permanently installed ESP32:

- Programs live on a microSD card in the machine's enclosure
- The operator opens a web page and picks a file
- **Drip-feed / online mode** streams programs larger than the controller's memory, in real time, while it cuts
- Programs can be pulled *back* off the machine and stored

## Hardware

| Component | Role |
|---|---|
| ESP32 DevKit V1 (30-pin) | MCU, Wi-Fi, FreeRTOS |
| MAX3232 | TTL ↔ RS-232 level conversion |
| microSD (SPI) | Program, received-file and log storage |
| 24 V → 5 V converter | Industrial panel power |

```
GPIO17 TX2 ──────> MAX3232 T1IN  ──> RS232 TX ──> CNC RX
GPIO16 RX2 <────── MAX3232 R1OUT <── RS232 RX <── CNC TX
GPIO25 RTS ──────> extra channel  ──> CNC CTS
GPIO26 CTS <────── extra channel  <── CNC RTS
GPIO27 DTR ──────> extra channel  ──> CNC DSR
GPIO33 DSR <────── extra channel  <── CNC DTR

microSD SPI: CS=5  SCK=18  MISO=19  MOSI=23
```

> **A plain MAX3232 module has only 2 drivers and 2 receivers.** That covers TX/RX plus a partial handshake pair. Full TX/RX + RTS/CTS + DTR/DSR needs a converter with more channels, or a second IC. Pin assignments live in [`Config.h`](Config.h).

## Architecture

```
microSD ──> TX ring buffer ──> TaskSerial ──> RS232 ──> CNC
CNC     ──> RS232 ──> TaskSerial ──> RX ring buffer ──> TaskDNC ──> microSD
```

| FreeRTOS task | Responsibility | Priority |
|---|---|---|
| `TaskSerial` | UART RS-232 read/write, flow control signalling | High |
| `TaskDNC` | Send, receive, drip-feed, pause/resume/cancel | High–Medium |
| `TaskWebServer` | Embedded web UI and file operations | Medium |
| `TaskStorage` | microSD diagnostics and recovery | Low |
| `TaskNetwork` | Wi-Fi reconnection | Low |
| `TaskLogger` | RAM and microSD logging | Low |

Serial I/O is decoupled from storage and from the web server by lock-free ring buffers (8 KB each), so a slow SD write or an HTTP request can never stall the byte stream feeding the machine — the one thing that must not happen mid-program.

## Flow control

Selectable per machine: `XON/XOFF`, `RTS/CTS`, `DTR/DSR`, or none.

Note that on most MAX3232 converters an asserted RS-232 control signal arrives **LOW** on the TTL side. The polarity is configurable rather than assumed:

```c
#define FLOW_INPUT_ACTIVE_LEVEL     LOW
#define FLOW_OUTPUT_ACTIVE_LEVEL    LOW
#define FLOW_OUTPUT_INACTIVE_LEVEL  HIGH
```

## Storage layout

```
/programs   G-code available to send
/received   files pulled back from the machine
/logs       session logs
```

## Building

Arduino IDE or `arduino-cli` with the **arduino-esp32** core. Board: *ESP32 Dev Module*.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COM3 .          # Windows
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 .   # macOS
```

`arduino-cli board list` prints the port name, which is the only thing that
differs between the three hosts. On macOS the board shows up under `/dev/cu.*`
and not `/dev/tty.*`: the `tty` device blocks on open waiting for carrier
detect, and a USB-serial bridge never asserts it, so the upload appears to hang
forever on a port that is in fact fine.

On first boot the device starts an access point:

| | |
|---|---|
| SSID | `SMI_DNC_ESP32` |
| Password | `12345678` |
| Hostname | `smi-esp32-dnc` |

**Change both before deploying.** They are first-boot defaults in [`Config.h`](Config.h), not secrets.

## Project status

Functional bench-grade base, validated against a CNC simulator and a real controller.

Before production use, validate on your own hardware: RS-232 electrical levels, **galvanic isolation**, grounding, DB9/DB25 pinout, connector strain relief, and the actual handshake protocol of each CNC — they differ between Fanuc, Siemens, Mach and older proprietary controls. A shop floor is electrically hostile and a mis-wired RS-232 line can take out the MCU, the controller, or both.

## Documentation

- [`docs/FLUXOGRAMAS.md`](docs/FLUXOGRAMAS.md) — operation flowcharts (PT-BR)
- [`docs/TESTE_BANCADA.md`](docs/TESTE_BANCADA.md) — bench test procedure (PT-BR)
- [`docs/README-pt.md`](docs/README-pt.md) — full original documentation (PT-BR)

## License

MIT — see [LICENSE](LICENSE). Published with permission from SMI.
