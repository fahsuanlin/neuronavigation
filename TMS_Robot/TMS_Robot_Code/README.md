# TMS Robot code (organized)

```
TMS_Robot_Code/
├── labview/        copies of all original VIs, project, aliases and Phi.txt (originals are untouched one folder up)
├── docs/           TMS_Robot_adjust_analysis.md  (what TMS Robot_adjust.vi does)
└── tcp/
    ├── PROTOCOL.md             the contract between the robot side and the Mac (give this to the Mac agent)
    ├── robot_simulator/        Python stand-in for the myRIO server (run on this Windows PC)
    ├── mac_client/             Python client library + CLI (copy to the Mac)
    └── pc_bridge/
        ├── tms_bridge.py       TMS Robot Bridge: standalone Windows program, Mac <-> LabVIEW, with monitor window
        ├── run_bridge.bat      double-click launcher
        ├── build_exe.bat       optional single .exe via PyInstaller
        └── tcp_bridge.py       old plain port forwarder (superseded by tms_bridge.py)
```

## Network
| Machine | Address |
|---|---|
| Windows PC, **wired Ethernet (used)** | 169.254.30.121 |
| Mac, wired Ethernet | 169.254.126.151 |
| Windows PC (Wi-Fi, not used) | 192.168.50.149 |
| myRIO (USB link, PC only) | 172.22.11.2 (PC side 172.22.11.1) |
| Port | 6340 |

Mac → `169.254.30.121:6340` over the direct cable. That reaches the simulator now. Once the LabVIEW TCP loop exists, the
same address reaches the bridge, which forwards to the myRIO.

## Quick test
PC:  `python tcp/robot_simulator/robot_sim_server.py`
Mac: `python3 tms_client.py --host 192.168.50.149 ping`

Windows Firewall must allow inbound TCP 6340. Run this once in an **admin** PowerShell:
`New-NetFirewallRule -DisplayName "TMS Robot TCP 6340" -Direction Inbound -Protocol TCP -LocalPort 6340 -Action Allow -Profile Any`

## Next step: LabVIEW server (to be added to TMS Robot_adjust.vi)
Add a separate **While Loop**, not a Timed Loop, next to the motor loops:
1. `TCP Listen` (port 6340), then write `HELLO TMS-ROBOT 1\r\n` with `TCP Write`.
2. Inner loop: `TCP Read` (mode CRLF, 512 bytes, 100 ms timeout). Error 56 (timeout) means keep
   looping. Errors 62/66 mean the client left, so close and go back to Listen.
3. `Scan From String` / `Match Pattern` on the line, then a Case structure on the command:
   `SET X` writes a local variable of `User Input x` (and the same for the other axes), `STOP`
   writes the `Stop` local, and `GET STATUS` uses `Format Into String` on locals of the
   indicators listed in PROTOCOL.md.
4. `TCP Write` the response plus `\r\n`. Stop the loop when `Stop` is true.

## TMS Robot Bridge (`tcp/pc_bridge/tms_bridge.py`)
A standalone program for this Windows PC. It sits between the Mac's navigation terminal and the LabVIEW
robot program, and it replaces the plain `tcp_bridge.py` forwarder. It needs only Python 3.8+ (standard
library), or the .exe made by `build_exe.bat`.

```
Mac (MATLAB) --TCP 169.254.30.121:6340--> tms_bridge --TCP 172.22.11.2:6340--> LabVIEW Main VI (binary 72B/44B)
                                          (window)    --or-- --robot sim  (built-in simulated robot)
```
- **Mac side**: protocol v1 unchanged (greeting, one client, 10 s idle drop). The bridge answers
  PING/HELP/HELLO/BYE, `GET DOC`, `DELTA` (validates and acks it, keeps the newest for `GET DELTA`, and forwards it to LabVIEW in
  the background) and `GET DELTA` itself. `GET STATUS`, `SET` and other commands go to LabVIEW. When the robot link is down they get
  `ERR BUSY robot not connected`.
- **Safety**: *Allow motion from Mac* is off by default. While it is off, `SET` returns `ERR BUSY operator has control`
  (the Mac's Goto/CALIBRATE stop cleanly). `STOP` from the network is always refused unless the bridge was started with `--allow-stop`.
- **LabVIEW side** (default `--robot-proto binary`): the Main VI's binary interface, the same one
  `codes/TMS_MATLAB_Controller/TMS_Start.m` uses. The bridge sends a 72-byte packet (18 big-endian float32:
  speed/position/jog per axis, tab mode, curve speed, theta, phi) on each `SET` and reads the 44-byte frames
  (11 float32: moving flag and value per axis, 1 = idle) that the VI sends every ~100 ms. `GET STATUS` is built
  from the newest frame. Targets start from the first frame, so connecting moves nothing. A `SET` axis reads as
  moving until the frames show the move. `SET THETA` uses tab mode 0 and `SET PHI` tab mode 1. `SET L/R` is not
  available, and `DELTA` stays on the bridge. No frame for 2 s drops the link; it reconnects every 2 s.
  `--robot-proto text` keeps the old protocol v1 text link (GUIDE.md) instead.
- **Window**: Mac and LabVIEW link lights, Mac idle time and request count, DELTA values and rate, robot
  x/y/z/psi/theta/phi (orange while moving), optical sensors, round trip time, newest-first log,
  *Disconnect Mac* and *Reconnect* buttons.

Start: stop `robot_sim_server.py` and close `TMS_Start` (MATLAB) first: port 6340 here, and the VI takes one
client. Then `py tms_bridge.py` (myRIO at 172.22.11.2:6340). Without the robot: `python3
tcp/robot_simulator/lv_binary_sim.py --port 6341` and `py tms_bridge.py --robot 127.0.0.1:6341`. Options: `--remote`, `--poll Hz`, `--listen`, `--docs`, `--nogui`, `-v`.
