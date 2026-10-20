# galaxy-buds-headtracker

Turn your **Samsung Galaxy Buds** into a PC head tracker. The earbuds already contain the motion sensor Samsung uses for *360 Audio*. This script reads that sensor over Bluetooth and streams your head's **left/right (yaw)** and **up/down (pitch)** to [opentrack](https://github.com/opentrack/opentrack), so any game that supports TrackIR / freetrack can follow your head. No camera, no extra hardware.

```
LEFT [--------------#-----|--------------------] RIGHT  yaw= +18.1  pitch=  -4.2
```

- Pure Python, no dependencies (standard library only)
- Windows and Linux
- ~50 updates per second

> **Status**
> - Tested: **Galaxy Buds2 (SM-R177)** on Windows 11, Python 3.13.
> - Linux support is implemented but **not tested on real hardware yet**. Reports are welcome.
> - Other models with 360 Audio (Buds Pro, Buds2 Pro, Buds3 …) may work, but are untested.

## Quick start

1. Pair the Buds with your PC and connect them (the normal audio connection is enough).
2. Find the Buds' Bluetooth MAC address:
   - **Windows (PowerShell):** `Get-PnpDevice -Class Bluetooth | Where-Object FriendlyName -match 'Buds'`
     The `InstanceId` contains `DEV_AABBCCDDEEFF`, so the MAC is `AA:BB:CC:DD:EE:FF`.
   - **Linux:** `bluetoothctl devices`
3. Run it with the earbuds in your ears:
   ```
   python buds2_headtracking.py AA:BB:CC:DD:EE:FF
   ```
4. In opentrack set **Input: UDP over network** (port 4242) and **Output: freetrack 2.0 Enhanced**, then press **Start**.
5. Look straight ahead and recenter (see below).

### Keys and options

| | |
|---|---|
| `r` | recenter (look straight ahead first) |
| **Ctrl+Alt+R** (Windows) | recenter from anywhere, even while a game has focus |
| `pkill -USR1 -f buds2_headtracking` (Linux) | recenter from anywhere. Bind it to a desktop shortcut. |
| `u` | measure the "look up" axis: press while looking straight, then lift your head within 3 s. Saved to `axes.json`. |
| `Ctrl+C` | quit |
| `--yaw-only` | left/right only, pitch is always 0 |
| `--channel N` | RFCOMM channel, if automatic discovery fails |
| `--trace` | print every Bluetooth frame in hex |

If left/right or up/down is reversed in a game, tick **Invert yaw / Invert pitch** in opentrack's *Options → Output*.

### Linux notes

- Your Python must have Bluetooth sockets: `python3 -c "import socket; socket.AF_BLUETOOTH"` must not fail.
- The RFCOMM channel is found with a built-in SDP query. If that fails, look it up with `sdptool browse AA:BB:CC:DD:EE:FF` (service UUID `2e73a4ad-332d-41fc-90e2-16bef06523f2`) and pass `--channel N`.

## How it works

### 1. Talking to the earbuds

Galaxy Buds expose a proprietary serial service over Bluetooth RFCOMM. The Buds+ and newer use UUID `2e73a4ad-332d-41fc-90e2-16bef06523f2`. The Buds2 also advertise a standard SPP service named `FACTORY`. Earlier head-tracking scripts connected to that one, which is why they failed on the Buds2: they connected but received no data and the earbuds dropped the link.

- **Windows:** Python's socket API only accepts a channel number. The script calls Winsock `connect()` with a `SOCKADDR_BTH` that carries the service UUID and port 0, so Windows looks up the channel itself.
- **Linux:** the script sends its own SDP *ServiceSearchAttributeRequest* over L2CAP (PSM 1) and reads the RFCOMM channel from the reply.

Each message is framed like this:

```
FD | size+flags (2, LE) | msg id | payload | CRC-16/XMODEM over id+payload (2, LE) | DD
```

Only three message IDs are used. Nothing else is ever sent to the earbuds.

| ID | Name | Use |
|---|---|---|
| 124 | `SET_SPATIAL_AUDIO` | `[1]` on, `[0]` off |
| 195 | `SPATIAL_AUDIO_CONTROL` | `[0]` attach, `[4]` keep-alive every 2 s, `[1]` detach |
| 194 | `SPATIAL_AUDIO_DATA` | sensor events from the buds. Type 32 is the game rotation vector (GRV): 4 × int16 / 10000 quaternion. |

### 2. From quaternion to head angles

The earbud sits tilted in your ear, so the sensor's own axes say little about where your head points. Recordings showed that in Samsung's GRV frame **world X is vertical**: during pure 90° head turns, world X stays almost fixed in the earbud frame, while it moves 50–65° when you nod.

The script therefore tracks a "nose" vector:

1. At recenter time it takes the up/down rotation axis (measured once, see `u`) and crosses it with the vertical to get a horizontal *forward* vector in earbud coordinates.
2. Every sample, it rotates that vector into the world frame.
3. **Yaw** is the vector's rotation in the horizontal plane. **Pitch** is its angle to the horizon.

A pure turn about the vertical therefore cannot change pitch, however the earbud is tilted. Angles are sent to opentrack as six little-endian doubles `(x, y, z, yaw, pitch, roll)` over UDP to `127.0.0.1:4242`.

## Credits

The protocol knowledge comes from [GalaxyBudsClient](https://github.com/timschneeb/GalaxyBudsClient) and [BudsPro-Headtracking](https://github.com/timschneeb/BudsPro-Headtracking) by [@timschneeb](https://github.com/timschneeb). This is an independent Python implementation.

Unofficial project, not affiliated with or endorsed by Samsung. Samsung, Galaxy and Galaxy Buds are trademarks of Samsung Electronics Co., Ltd. They are used here only to describe compatibility.

## Türkçe

Samsung Galaxy Buds kulaklığını PC'de kafa takip cihazı olarak kullanmanı sağlar. Kulaklığın 360 Audio sensörünü Bluetooth ile okur ve sağ/sol (yaw) ile aşağı/yukarı (pitch) açılarını opentrack'e gönderir. TrackIR/freetrack destekleyen oyunlarda kafanla etrafa bakabilirsin.

1. Kulaklığı PC'ye eşleştir ve bağla.
2. `python buds2_headtracking.py AA:BB:CC:DD:EE:FF` komutunu çalıştır (MAC adresini yukarıdaki gibi bul).
3. opentrack'te Input olarak **UDP over network**, Output olarak **freetrack 2.0 Enhanced** seç, sonra **Start**'a bas.
4. Düz bak ve ortala: penceredeyken `r`, oyundayken Windows'ta **Ctrl+Alt+R**.

Sadece sağ/sol istiyorsan `--yaw-only` ekle. Galaxy Buds2 ile Windows 11'de test edildi. Linux desteği yazıldı ama henüz gerçek donanımda denenmedi.

## License

MIT
