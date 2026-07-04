# laser-pointer-robot

## Microcontroller commands

The ESP32 accepts commands over USB serial at `115200` baud and over UDP after connecting to an existing Wi-Fi network.

Create `src/Laser_Pointer_Robot/secrets.h` with your local Wi-Fi credentials. This file is ignored by git.

```cpp
#pragma once

static const char *WIFI_SSID = "your-wifi-ssid";
static const char *WIFI_PASSWORD = "your-wifi-password";
```

- UDP target: the IP address printed by the ESP32 at startup, port `2222`

Commands are framed with the ASCII prefix `JJA` followed by a one-letter command code. Multi-byte integer parameters are sent big-endian.

| Command | Payload | Example | Description |
| --- | --- | --- | --- |
| `JJAH` | none | `JJAH` | Hello/keep-alive command. Stops the current `working` state without moving the axes. |
| `JJAT` | two signed 6-character decimal values, separated by any one byte | `JJAT+00900,-00450` | Sets target axis angles. Values are divided by `100`, so `+00900` means `9.00` degrees. |
| `JJAM` | eight signed 16-bit integers | `4A 4A 41 4D 03 84 FE 3E 00 00 00 00 00 00 00 00 00 00 00 00` | Sets channel values in binary form. This example sets channel 1 to `900` and channel 2 to `-450`; channels 1 and 2 are divided by `100` for axis targets. |
| `JJAS` | five signed 16-bit integers | `4A 4A 41 53 00 32 00 00 00 64 00 00 00 00` | Sets speed and acceleration percentages. This example sets XY speed to `50%` and XY acceleration to `100%`. |
| `JJAC` | none | `JJAC` | Calibrates/zeros the current motor position. |
| `JJAE` | none | `JJAE` | Emergency stop. Stops timers, disables the motor driver, and turns off the status LED. |

Example: move axis 1 to `9.00` degrees and axis 2 to `-4.50` degrees over UDP from PowerShell:

```powershell
$udp = [System.Net.Sockets.UdpClient]::new()
$bytes = [System.Text.Encoding]::ASCII.GetBytes("JJAT+00900,-00450")
$udp.Send($bytes, $bytes.Length, "<esp32-ip-address>", 2222)
$udp.Close()
```

The same payload can also be sent over the USB serial connection.
