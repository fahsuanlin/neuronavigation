# Starting the TMS Robot Bridge on the Windows PC

The **TMS Robot Bridge** (`TMS_Robot/TMS_Robot_Code/tcp/pc_bridge/tms_bridge.py`) is a Python program that runs
on the Windows PC. It connects the Mac's navigation terminal to the LabVIEW Main VI on the myRIO and relays between them.

```
Mac (MATLAB, nav_camera_terminal) ── text ──▶ tms_bridge.py (Windows PC) ── binary ──▶ LabVIEW Main VI (myRIO)
                         169.254.30.121:6340 (en23 cable)          172.22.11.2:6340 (USB)
```

- **Mac side:** text protocol v1 (`doc/tms_robot/PROTOCOL.md`): `GET STATUS`, `SET <axis> <value>`, `DELTA`, `PING`, …
- **LabVIEW side:** the Main VI's binary interface, the same one `codes/TMS_MATLAB_Controller/TMS_Start.m` uses:
  72-byte command packets out (18 big-endian float32), 44-byte frames in every ~100 ms (11 float32).

The Mac cannot reach the myRIO (it is on USB to the Windows PC), so the bridge is required.

## 1. One-time setup

1. **Get the files.** Clone or pull this repository on the Windows PC (branch `robot-tab`), or copy the folder
   `TMS_Robot/TMS_Robot_Code/tcp/pc_bridge/` (`tms_bridge.py`, `run_bridge.bat`, `build_exe.bat`).
2. **Python 3.8 or newer.** In a Command Prompt, run `py --version`. If it is not found, install Python 3 from
   python.org and tick *Add python.exe to PATH*. The bridge uses only the standard library (no `pip install`).
3. **Firewall.** Open TCP port 6340 for the Mac. In PowerShell **as Administrator**, run once:
   ```powershell
   New-NetFirewallRule -DisplayName "TMS Robot TCP 6340" -Direction Inbound -Protocol TCP -LocalPort 6340 -Action Allow -Profile Any
   ```
   Without this rule, the Mac's connection times out.
4. *(Optional)* **Single .exe.** Run `build_exe.bat` once. It builds `dist\tms_bridge.exe` with PyInstaller.
   The .exe runs on PCs without Python. For `GET DOC`, copy `BRIEF.md`, `PROTOCOL.md` and `GUIDE.md` into
   `dist\docs\`.

## 2. Each session

1. **Start (or restart) the LabVIEW Main VI** on the myRIO. Check that it has homed and that the motors are safe to drive.
   **Restart the VI every time the bridge is restarted or loses its connection:** the VI's TCP loop (built from the
   NI simple TCP server example, `TCP Receive.vi`) stops when its client disconnects. The VI then still accepts
   connections but sends nothing, and the bridge shows *connected, no 44-byte frame*.
2. **Free the VI's TCP port.** The VI accepts **one client**: close `TMS_Start` in MATLAB (`TMS_Stop`) and any other
   program connected to 172.22.11.2:6340. Also stop `robot_sim_server.py` if it is running, because it uses port 6340 on this PC.
3. **Start the bridge**, either:
   - double-click `run_bridge.bat`, or
   - in a Command Prompt in the `pc_bridge` folder:
     ```
     py tms_bridge.py
     ```
4. **Check the window:**
   - *LabVIEW robot* light **green**. The log shows `LabVIEW connected` and then `LabVIEW first frame: x=… phi=…`.
     The x/y/z/psi/theta/phi row updates live (orange = moving).
   - The log (newest line on top) keeps a **status history**: a `= VI x=… phi=…  idle` or `… moving: X` line on
     every change of a value or moving flag, and one every 10 s while nothing changes. Each command sent to the VI
     shows as `* 72B #n: x=… tab=…`. A panel change shows as `LabVIEW Y target is 3 (was 0 on the bridge): adopted`.
   - *Mac* light **grey** (listening) until the Mac connects, then **green**.
5. **On the Mac:** open `nav_camera_terminal`, *Robot* tab, press the robot comm. button. The bridge window shows
   `Mac connected from 169.254.x.x`. DELTA values appear about 5 times a second once a marker and a target are set.
6. **Motion.** *Allow motion from Mac (SET / Goto / CALIBRATE)* is **off** at start. While it is off, every `SET`
   from the Mac is refused (`ERR BUSY operator has control`) and nothing moves. Tick it only when the robot is
   ready to move and someone is watching it. Untick it to take control back at any time.

Close the window to stop the bridge. Closing it does **not** stop the motors: use the LabVIEW panel for that.
`STOP` from the Mac is always refused, because it would end the robot program.

## 3. Command-line options

| Option | Default | Meaning |
|---|---|---|
| `--robot HOST:PORT` | `172.22.11.2:6340` | LabVIEW Main VI address. `--robot sim` uses a built-in simulated robot instead. |
| `--robot-proto binary\|text` | `binary` | `binary` = Main VI 72B/44B interface; `text` = protocol v1 text lines (`GUIDE.md` VI loop). |
| `--speed S` | `4` | X/Y/Z/PSI speed in steps/s, sent in every packet (binary). |
| `--curve-speed S` | `4` | THETA/PHI speed in steps/s (binary). |
| `--init X,Y,Z,PSI,THETA,PHI` | | Only for a VI that sends no status before its first command: start targets, e.g. `0,0,0,0,0,0`. Must match the robot: the first `SET` sends all six. Not needed with the current VI (it sends status on its own after a fresh start). Once frames arrive, the VI's values replace these. |
| `--listen HOST:PORT` | `0.0.0.0:6340` | Where the Mac connects. |
| `--remote` | off | Start with *Allow motion from Mac* ticked. |
| `--nogui` | off | Console only, no window. |
| `-v` | off | Log every Mac request except DELTA and PING. |
| `--docs DIR` | | Folder with BRIEF/PROTOCOL/GUIDE.md for `GET DOC`. |

## 4. Without the robot

- **Built-in simulator:** `py tms_bridge.py --robot sim`. The Mac can test everything, including Goto.
- **VI binary stand-in** (tests the binary path): in one window
  `py ..\robot_simulator\lv_binary_sim.py --port 6341`, in another `py tms_bridge.py --robot 127.0.0.1:6341`.

## 5. What the bridge does with each Mac command (binary mode)

| Mac sends | Bridge does |
|---|---|
| `GET STATUS` | Answers from the newest 44-byte frame: the VI's values (targets, see §7), `mx my mz mpsi ml mr` (1 = moving). A just-SET axis reads as moving until the VI's moving flag has risen and fallen again, or for 2.5 s if it never rises (a SET to the current value). |
| `SET X\|Y\|Z\|PSI <mm>` | Range -25..25. Updates that target and sends a full 72-byte packet. Every other axis is sent with the VI's current value, so targets set on the LabVIEW panel are kept. |
| `SET THETA <v>` / `SET PHI <v>` | Sends with tab mode 0 (THETA) or 1 (PHI); the curve VI takes one of them per packet. |
| `SET L\|R` | `ERR BAD_AXIS`: not in the binary interface. |
| `DELTA dx dy dz d` | Acknowledged and stored on the bridge (`GET DELTA`); not sent to LabVIEW. |
| `PING`, `HELP`, `GET DOC`, `BYE` | Answered by the bridge. |
| `STOP` | Refused. |

Targets start from the VI's first frame after each connect, so starting the bridge or connecting the Mac sends no
packet and moves nothing. If no frame arrives, the bridge stays connected and says so in the window and log (it does
not reconnect, which would churn the VI's listener). Only a closed connection or an out-of-step frame drops the link;
then it reconnects every 2 s.

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| Mac: connection timed out | Bridge running? Firewall rule (step 1.3)? Windows PC still at 169.254.30.121 (`dns-sd -G v4 X202409041720.local` on the Mac)? |
| Bridge: `Address already in use` / Mac side not listening | Another program has port 6340 on this PC (`robot_sim_server.py`, an older bridge). Close it. |
| LabVIEW light red, `connection refused` / `timed out` | VI not running, myRIO USB link down (172.22.11.2), or `TMS_Start` still connected. |
| Connected but no `first frame` (*connected, no 44-byte frame for N s*) | The VI's TCP loop is not running, usually because an earlier client disconnected. Close the bridge, **restart the VI**, start the bridge. `SET` and `GET STATUS` answer `ERR BUSY no recent 44-byte frame`. |
| Bridge shows connected / reconnected over and over | Old bridge version (before 2026-11-24): `git pull`, restart the VI, start the bridge. |
| `bad 44-byte frame … out of step` | The VI's frame size or layout differs from 11 float32. The bridge reconnects. Compare with `TMS_Start.m`. |
| Mac: `ERR BUSY operator has control` | *Allow motion from Mac* is unticked: intended. It is unticked after every bridge start. |

## 7. How the real VI behaves (measured 2026-11-24)

- **Start state:** all six values 0, including THETA and PHI (`TMS_Start.m`'s PHI 53.1916 and the protocol notes'
  THETA −85 do not apply to this VI).
- **Status frames** come every ~100 ms on their own after a fresh VI start; no first command is needed.
- **The values are targets, not measured positions.** After a packet they jump to the new target within one frame,
  while the motor is still moving. A target changed on the LabVIEW panel shows up the same way.
- **The moving flag** (1 = idle in the VI) **rises about 1 s after a packet**, then stays up until the motor stops.
- **Speed:** 4 steps/s, 26 steps per mm. `SET X 0.5` (13 steps) reports done after 4.1–4.3 s (1 s delay + 3.25 s).
- **The TCP loop stops when its client disconnects** (§2 step 1).

## 8. Open points

- Units of the VI's values: assumed mm for X/Y/Z/PSI, as in protocol v1. The X move above looked consistent
  (13 steps for 0.5 mm) but the distance was not measured.
- Ask the VI programmer to make the TCP loop go back to *TCP Listen* after a client disconnects, so the VI does not
  need a restart after every bridge restart.
- THETA/PHI moves, Y/Z/PSI moves, CALIBRATE and Goto have not been run on the real robot yet.

## 9. Session log

**2026-11-24: first moves from the Mac through the bridge.**
1. Bridge started on Windows. Mac ↔ bridge worked at once (PING 1 ms, firewall rule OK). The bridge connected to the
   VI and then reconnected every 2 s: the VI sent no frames. Fix in the bridge: stay connected on silence.
2. Still no frames: the VI's TCP loop had stopped after the earlier disconnects. Restarting the VI fixed it, and status
   came through (x = 1 from a manual panel move; all other values 0, idle).
3. `SET X 0.5` from the Mac moved X from 1 to 0.5. It also moved **Y from 3 to 0**: Y had been set to 3 on the panel
   just before, and the bridge's packet still carried its own Y = 0. It also reported the move done after 0.4 s,
   because the values are targets. Both fixed in the bridge (commit `c0c2d7b`): other axes adopt the VI's values every
   frame, and done waits for the moving flag to rise and fall. The log gained the status history.
4. After `git pull`, VI restart and bridge restart: `SET X 0.5` then `SET X 0`. Each reported done at 4.3 / 4.1 s,
   the other five axes stayed 0, and X ended at 0.
