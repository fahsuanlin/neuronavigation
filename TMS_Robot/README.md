# TMS Robot (sub-project of neuronavigation)

A positioning robot that holds the TMS coil. The robot's motors are driven by LabVIEW on an NI myRIO. The
neuronavigation app on the Mac uses the Polaris camera to measure where the coil marker is relative to a target, and
sends commands to the robot over TCP.

```
Mac: nav_camera_terminal (MATLAB, codes/)
   │  text protocol v1, TCP 169.254.30.121:6340 (Ethernet cable, en23)
   ▼
Windows PC: TMS Robot Bridge (bridge/tms_bridge.py)     ← monitor window, "Allow motion from Mac" switch
   │  binary 72-byte commands / 44-byte status, TCP 172.22.11.2:6340 (USB)
   ▼
myRIO: LabVIEW Main VI (TMS Robot_adjust.vi, this folder)  →  motors X Y Z PSI, curve THETA/PHI
```

## Status (2026-11-24)

- Mac ↔ bridge ↔ LabVIEW VI works with the real robot. Single X moves sent from the Mac are verified.
- Not yet run on the real robot: moves on Y/Z/PSI/THETA/PHI, CALIBRATE, and Goto (closed loop on the camera).
- The VI's TCP loop stops when its client disconnects, so the VI must be restarted after each bridge restart.
  A fix has been requested from the VI programmer.

## Folder map

| Path | What | Runs on |
|---|---|---|
| `*.vi`, `TMS Robot.lvproj`, `.aliases`, `.lvlps`, `Phi.txt` | LabVIEW project. `TMS Robot_adjust.vi` is the Main VI. `TCP Receive.vi` and `TCP_*_Control(SubVI).vi` hold the TCP interface. | myRIO (edited in LabVIEW on Windows) |
| [`bridge/`](bridge/) | `tms_bridge.py` (the bridge), `run_bridge.bat` (start), `build_exe.bat` (optional .exe), `tcp_bridge.py` (old plain forwarder) | Windows PC |
| [`matlab_controller/`](matlab_controller/) | `TMS_Start/Status/Stop.m`: the robot side's own MATLAB panel that talks to the VI directly (binary). Reference for the binary format. Cannot run while the bridge is connected. | Windows PC (MATLAB) |
| [`simulators/`](simulators/) | `tms_robot_sim.py`: v1 robot for testing the Mac app. `lv_binary_sim.py`: stand-in for the VI's binary interface. `robot_sim_server.py`: old v1 server. | Mac or Windows |
| [`tools/`](tools/) | `tms_client.py`: small Python v1 client/CLI for manual checks | Mac |
| [`doc/`](doc/) | Documentation (below) | |

Mac-side robot code lives with the rest of the navigation app in `../codes/`: `nav_robot_tab.m` (Robot tab),
`nav_robot_goto.m` (Goto / CALIBRATE), `nav_camera_tool_comm.m` (TCP link).

## Documentation

| Document | Read it for |
|---|---|
| [`doc/WINDOWS_BRIDGE.md`](doc/WINDOWS_BRIDGE.md) | **Starting the system:** Windows setup, each-session steps, options, troubleshooting, how the real VI behaves, session log |
| [`doc/PROTOCOL.md`](doc/PROTOCOL.md) | The Mac ↔ bridge contract (text protocol v1) |
| [`doc/MAC_ROBOT_TCP.md`](doc/MAC_ROBOT_TCP.md) | Full Mac-side reference: network, commands, STATUS fields, axes and units, DELTA, MATLAB implementation, Goto, testing, safety, change log |
| [`doc/labview_vi/`](doc/labview_vi/) | What is known about the LabVIEW VI (decoded labels, block-diagram exports) |
| [`doc/archive/`](doc/archive/) | Superseded documents, kept for reference |

`GET DOC BRIEF|PROTOCOL|GUIDE` from the Mac returns this file, `doc/PROTOCOL.md` and `doc/WINDOWS_BRIDGE.md`.

## Quick start

1. Windows: start (or restart) the LabVIEW Main VI, then in `TMS_Robot\bridge` run `py tms_bridge.py`.
2. Mac: open `nav_camera_terminal`, go to the **Robot** tab, and press the robot comm. button.
3. To let the Mac move the robot, tick **Allow motion from Mac** in the bridge window.

Details: [`doc/WINDOWS_BRIDGE.md`](doc/WINDOWS_BRIDGE.md).

## Safety

- *Allow motion from Mac* is off at every bridge start. Untick it to take control back.
- `STOP` from the network is refused, because it would end the robot program. Use the LabVIEW panel to stop the motors.
- Every command packet sets all axes. The bridge sends the VI's current values for the axes it is not moving, so
  targets set on the LabVIEW panel are kept.
