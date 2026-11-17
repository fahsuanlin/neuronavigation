# TMS Robot code (organized)

```
TMS_Robot_Code/
├── labview/        copies of all original VIs, project, aliases and Phi.txt (originals are untouched one folder up)
├── docs/           TMS_Robot_adjust_analysis.md  (what TMS Robot_adjust.vi does)
└── tcp/
    ├── PROTOCOL.md             the contract between the robot side and the Mac (give this to the Mac agent)
    ├── robot_simulator/        Python stand-in for the myRIO server (run on this Windows PC)
    ├── mac_client/             Python client library + CLI (copy to the Mac)
    └── pc_bridge/              PC -> myRIO port forwarder (needed only if the myRIO is on USB only)
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
