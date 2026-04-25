# MAVLink Telemetry Forward

The companion publishes its state as MAVLink v2 messages. Three message types:

| Message | Rate | Contents |
|---------|------|----------|
| `HEARTBEAT` (id 0) | 1 Hz | "I'm alive" — required by every MAVLink GCS to detect link loss |
| `COMPANION_STATE` (id 12500, custom) | 5 Hz | distance to waypoint (m), altitude (m), heading (deg), state code, safety reasons bitmap |
| `STATUSTEXT` (id 253) | on change | human-readable safety reason text |

Publishing is **off by default** — set `companion.mavlink_publisher` in
`config.json` to enable.

## Quickest path: QGroundControl on a phone

The fastest way to see companion state on a screen is a phone running
QGroundControl. Works without any radio integration.

### One-time setup

1. Install [QGroundControl](https://qgroundcontrol.com/) on your phone.
2. Set your Pi to host a WiFi access point. Edit `/etc/dhcpcd.conf` and
   `/etc/hostapd/hostapd.conf` per the
   [Raspberry Pi AP guide](https://www.raspberrypi.com/documentation/computers/configuration.html#setting-up-a-headless-raspberry-pi-access-point).
3. In `/etc/racer-companion/config.json`, set:
   ```json
   "mavlink_publisher": "udp://192.168.4.255:14550"
   ```
   (`192.168.4.255` is the broadcast address on the Pi's AP subnet by
   default. Adjust if you set a different `static ip_address` in `dhcpcd.conf`.)
4. Restart the companion: `sudo systemctl restart racer-companion`.

### On the field

1. Connect phone to the Pi's WiFi AP.
2. Open QGroundControl. It auto-discovers MAVLink on UDP 14550.
3. Within a couple of seconds you should see "Connected" and battery /
   altitude / heading start updating.
4. The custom `COMPANION_STATE` message will appear under MAVLink Inspector.
   Future Phase 2 work: a small QGC plugin to render state name + safety
   reasons as a big readout.

## Path 2: Logging only (post-flight analysis)

If you don't need a live HUD, point the publisher at a logger:
```json
"mavlink_publisher": "udp://127.0.0.1:14550"
```
Then on the Pi:
```bash
nc -lu -p 14550 > /var/log/racer-mavlink-$(date +%s).bin
```
The binary file can be replayed later with
[mavproxy.py](https://ardupilot.org/mavproxy/) or analyzed with pymavlink.

## Path 3: Radio HUD (deferred — Phase 2)

Getting companion data onto your transmitter screen is genuinely hard
because every link in the chain is variable:

- ELRS firmware version + receiver model (whether MAVLink-over-CRSF is
  supported and how)
- Radio (whether EdgeTX has a MAVLink display library installed; not all
  do by default)
- BF version (CRSF telemetry passthrough behavior shifts)

For radios with the [Yaapu telemetry script](https://github.com/yaapu/FrskyTelemetryScript)
and ELRS in MAVLink mode, this can work today — but the integration burns
a weekend per radio variant. See `ROADMAP.md` for the deferred plan.

If you have a radio with MAVLink mode and want to try it now:
```json
"mavlink_publisher": "serial:///dev/ttyAMA1@57600"
```
Wire the Pi's UART1 to your ELRS RX backchannel UART. Configure ELRS for
MAVLink mode. Configure your radio's MAVLink telemetry script. Be ready
for ELRS-version-specific quirks.

## Wire format reference

This is documented for future maintainers regenerating tests against
pymavlink, or for a future radio-side decoder.

### COMPANION_STATE (msgid 12500, CRC_EXTRA 42)

| Offset | Type | Field |
|--------|------|-------|
| 0      | float (LE) | `distance_m` |
| 4      | float (LE) | `alt_m` |
| 8      | float (LE) | `heading_deg` |
| 12     | uint8 | `state` (0=IDLE, 1=CLIMB, 2=TRANSIT, 3=HOLD, 4=RACING, 5=ABORT — wait, the actual mapping is `state_machine.State` enum order: 1=IDLE, 2=CLIMB, 3=TRANSIT, 4=HOLD, 5=RELEASED) |
| 13     | uint8 | `safety_reasons_bitmap` |

### Safety reason bitmap

| Bit | Reason |
|-----|--------|
| 0   | `no_gps_telemetry` |
| 1   | `no_gps_fix` |
| 2   | `low_sats` |
| 3   | `stale_gps` |
| 4   | `stale_altitude` |
| 5   | `stale_attitude` |
| 6   | `low_vbat` |
| 7   | `geofence` |

## Troubleshooting

**QGC says "no MAVLink heartbeat received."**
- Companion not running: `journalctl -u racer-companion -n 20`.
- `mavlink_publisher` empty in config — check `cat /etc/racer-companion/config.json`.
- Phone not on Pi's WiFi AP, or wrong subnet.
- Pi firewall blocking UDP 14550: `sudo ufw allow 14550/udp` if UFW is on.

**STATUSTEXT messages flooding QGC.**
- Companion is rapidly transitioning between safety states. Check journal
  for what's flapping. Common: GPS losing fix between satellites.

**Custom COMPANION_STATE message shows as "Unknown msg 12500."**
- That's normal — QGC doesn't know our custom dialect. The MAVLink
  Inspector view shows the raw bytes. Phase 2 will ship a small QGC
  plugin or a custom widget.

**MAVLink publisher init failed in journal.**
- Bad `mavlink_publisher` URI. Valid forms:
  - `udp://host:port`
  - `serial:///dev/path` (default 57600 baud)
  - `serial:///dev/path@115200`

## Verifying with pymavlink

For maintainers regenerating reference vectors after wire-format changes:

```bash
pip install pymavlink
python -c "
from pymavlink import mavutil
from pymavlink.dialects.v20 import common
import io
mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=191)
mav.seq = 0
buf = io.BytesIO(); mav.file = buf
msg = common.MAVLink_heartbeat_message(
    type=18, autopilot=8, base_mode=0, custom_mode=0,
    system_status=4, mavlink_version=3)
mav.send(msg)
print(buf.getvalue().hex())
"
```

The output is the byte-for-byte ground truth that `test_heartbeat_byte_for_byte_pymavlink`
asserts against. If you change the encoder, update both.
