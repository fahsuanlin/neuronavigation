# Mac ⇄ Windows TMS-robot TCP link

How the Mac navigation app (MATLAB, `nav_camera_terminal`) talks to the TMS robot, which is controlled
by LabVIEW on the Windows PC.

| | |
|---|---|
| Protocol | **TMS-robot protocol v1**, ASCII lines over TCP |
| Server | Windows PC / robot side, `169.254.30.121:6340` |
| Client | Mac on its en23 port, self-assigned 169.254.x.x (was `169.254.126.151` on 11-17, `169.254.240.129` on 11-18) |
| Authoritative spec | [`PROTOCOL.md`](PROTOCOL.md). If this file and PROTOCOL.md disagree, PROTOCOL.md wins. |
| Other robot-side docs | [`BRIEF.md`](BRIEF.md) (status and notes for the Mac), [`GUIDE.md`](GUIDE.md) (LabVIEW implementation) |
| Mac code | `codes/nav_camera_tool_comm.m`, `codes/nav_camera_graphic_update.m`, `codes/nav_camera_terminal.mlapp` |
| Simulator | `codes/tms_robot_sim.py` |
| Last updated | 2026-11-24 (Windows bridge live with the real robot; first moves from the Mac) |

The three robot-side docs are copies of what the server returns for `GET DOC BRIEF|PROTOCOL|GUIDE`.
Re-fetch them whenever the robot side announces a change (see [§12.4](#124-refresh-the-robot-side-docs)).

---

## Contents
1. [System overview](#1-system-overview)
2. [Physical network](#2-physical-network)
3. [Transport and framing](#3-transport-and-framing)
4. [Connection life cycle](#4-connection-life-cycle)
5. [Command reference](#5-command-reference)
6. [STATUS fields](#6-status-fields)
7. [Axes, units and quantization](#7-axes-units-and-quantization)
8. [Motion and timing](#8-motion-and-timing)
9. [DELTA: streaming the tracker offset](#9-delta-streaming-the-tracker-offset)
10. [Errors and how the Mac handles them](#10-errors-and-how-the-mac-handles-them)
11. [Mac implementation (MATLAB)](#11-mac-implementation-matlab)
12. [Robot implementation (LabVIEW / simulators)](#12-robot-implementation-labview--simulators)
13. [Testing recipes](#13-testing-recipes)
14. [Troubleshooting](#14-troubleshooting)
15. [Safety rules](#15-safety-rules)
16. [Open items and change log](#16-open-items-and-change-log)

---

## 1. System overview

```
 ┌──────────────────────── Mac ─────────────────────────┐          ┌──────────── Windows PC ────────────┐
 │                                                      │          │                                    │
 │  Polaris Vega ──en22──► nav_camera_terminal (MATLAB) │   en23   │  TCP server :6340                  │
 │  (P9-23261)            ├ BX/BX2 tracking loop        │◄────────►│   now:   Python simulator          │
 │                        ├ Robot tab: marker − target  │  direct  │   later: tcp_bridge.py ──USB──►    │
 │                        └ nav_camera_tool_comm.m ─────┼─ cable ─►│          myRIO 172.22.11.2:6340    │
 │                          (TCP client)                │          │          TMS Robot_adjust.vi       │
 └──────────────────────────────────────────────────────┘          └────────────────────────────────────┘
```

- The Mac tracks tools with the Polaris Vega camera.
- In the **Robot** tab you pick a marker and type a target point. The app computes marker − target (dx, dy, dz, d).
- That offset is streamed to the robot as `DELTA` lines while the marker is visible.
- The robot side stores the offset and shows it. `DELTA` **never moves the robot**.
- The same connection can also read robot state (`GET STATUS`) and, when explicitly requested, send
  motion targets (`SET`).

Roles: the **robot side is always the server**, and the **Mac is always the client**.

---

## 2. Physical network

The Mac has two USB Ethernet adapters, each connected by a direct cable. Both use self-assigned
(link-local, 169.254.x.x) addresses.

| Mac port | Adapter | Mac address | Peer | Peer address | Port |
|---|---|---|---|---|---|
| **en23** | USB 10/100/1000 LAN | 169.254.240.129 (changes) | Windows/LabVIEW PC `X202409041720.local` | **169.254.30.121** | **6340** (robot) |
| en22 | AX88179B | 169.254.30.49 | Polaris Vega `P9-23261.local` | 169.254.8.140 | 8765 (NDI API) |
| en0 | Wi-Fi | 192.168.0.101 | internet | | |

Notes:
- **Link-local addresses can change** after a reboot or replug. The Mac's en23 address changed overnight
  (11-17 → 11-18) while the Windows PC kept 169.254.30.121. The server does not care about the client
  address. After a change, the first connection can time out once while the link re-learns
  addresses; just retry. Find the addresses again with:
  ```sh
  ifconfig en23 | grep inet          # Mac side
  arp -an | grep en23                # peer seen on the cable
  dns-sd -B _ni-logos._tcp local.    # Windows PC advertises NI services on en23
  dns-sd -G v4 X202409041720.local   # resolve its current IP
  ```
- The Windows PC **does not answer ping** (Windows Firewall default). Use a TCP check instead:
  `nc -vz 169.254.30.121 6340`.
- The Windows PC also has `192.168.50.149`, which is **not reachable** from the Mac. The Mac has no
  192.168.50.x address on en23, so that traffic goes out over Wi-Fi. If that subnet is ever needed:
  `sudo ifconfig en23 alias 192.168.50.10 255.255.255.0`.
- When the real robot comes online, the myRIO (USB-only, `172.22.11.2`) will be reached through
  `tcp_bridge.py` on the Windows PC, listening on the **same** `169.254.30.121:6340`. The Mac code does not
  change.

---

## 3. Transport and framing

| Item | Rule |
|---|---|
| Transport | TCP, port **6340** |
| Encoding | ASCII. Doc bodies are UTF-8. |
| Line ending | **`\r\n` (CRLF)** on every line. The server tolerates a bare `\n`, but the client **must** send CRLF, because LabVIEW `TCP Read` in CRLF mode needs the CR. |
| Line length | max 512 bytes |
| Exchange | strictly **one request → exactly one reply line**, in order. The only exception is `GET DOC` (header line + raw body). |
| Case | commands are case-insensitive. Replies use upper-case keywords. |
| Numbers | decimal with `.`, no thousands separators. The server prints `%g` style: up to 6 significant digits, no trailing zeros (`0`, `0.5`, `-84.0533`, `53.1916`). Clients must accept any decimal form. |
| Greeting | the server speaks first: `HELLO TMS-ROBOT 1 [free text]`. **The third word is the protocol version.** |
| Clients | **one at a time**. A second connection gets `ERR BUSY another client connected` and is closed. |
| Idle timeout | the server may close a connection after **10 s** with no request. |
| Poll rate | `GET STATUS` at **≤ 20 Hz** |

Because replies come strictly in order, a client may **pipeline**: send several requests and match the
replies first-in, first-out. The Mac does this (see [§11.4](#114-reply-matching)).

---

## 4. Connection life cycle

```
 Mac (client)                                   Robot (server)
 ────────────                                   ──────────────
 connect 169.254.30.121:6340  ───────────────►  accept
                              ◄───────────────  HELLO TMS-ROBOT 1 ...          (or ERR BUSY ... + close)
 check: word1=HELLO, word2=TMS-ROBOT, word3=1
 [optional] HELLO <anything>  ───────────────►
                              ◄───────────────  OK HELLO TMS-ROBOT 1 ...
 ┌─ loop ─────────────────────────────────────────────────────────────────────────┐
 │ DELTA dx dy dz d (~5 Hz)  ──►                ◄── OK DELTA dx dy dz d            │
 │ PING (after ≥4 s silence) ──►                ◄── OK PONG                        │
 │ GET STATUS (≤20 Hz)       ──►                ◄── STATUS t=..;x=..;...           │
 │ SET <axis> <v> (explicit) ──►                ◄── OK SET <axis> <v> | ERR ...    │
 └────────────────────────────────────────────────────────────────────────────────┘
 BYE                          ───────────────►
                              ◄───────────────  OK BYE                          then server closes
```

Ways a connection ends:

| Cause | What happens |
|---|---|
| Client sends `BYE` | `OK BYE`, then the server closes the socket. The robot keeps running. |
| 10 s without a request | the server closes. The client sees a read/write error. |
| Client process exits / socket closed | the server sees error 62/66 and goes back to listening |
| Client sends `STOP` | `OK STOP`, and **the whole robot program ends**. Never do this from the Mac. |

---

## 5. Command reference

`C:` is the client (Mac) and `S:` is the server (robot). Every line ends with `\r\n`.

### 5.1 `HELP`
One-line summary of all commands.
```
C: HELP
S: OK HELP protocol v1, ASCII lines ending CRLF, one reply per request | PING -> OK PONG | GET STATUS -> ... | ...
```

### 5.2 `HELLO ...` (optional)
Optional client greeting. It has no effect on state.
```
C: HELLO MAC 1
S: OK HELLO TMS-ROBOT 1 (send HELP for commands)
```

### 5.3 `PING`
Liveness check and keepalive.
```
C: PING
S: OK PONG
```

### 5.4 `GET DOC <BRIEF|PROTOCOL|GUIDE>`
The only multi-line reply: a header line, then exactly `<nbytes>` raw UTF-8 bytes **with no trailing
CRLF of its own**. Read the header, then read exactly `nbytes`.
```
C: GET DOC PROTOCOL
S: DOC PROTOCOL 5249
S: <5249 bytes of Markdown>
```

### 5.5 `GET STATUS`
All robot indicators on one line. The fields are listed in [§6](#6-status-fields).
```
C: GET STATUS
S: STATUS t=151905;x=0.5;y=0;z=0;psi=0;theta=-84.0533;phi=53.1916;mx=0;my=0;mz=0;mpsi=0;ml=0;mr=0;ox=0.12;oy=0.1;oz=0.11;opsi=0.09;ol=0.13;or=0.12
```

### 5.6 `SET <axis> <value>`: **moves the robot**
| Axis | Meaning | Value |
|---|---|---|
| `X` `Y` `Z` `PSI` | absolute target | mm, −25 … +25 (Y/Z/PSI limits provisional) |
| `THETA` | absolute azimuth target | starts at −85, ≈0.15778 per step. Drives motors L and R together. |
| `PHI` | absolute elevation target | starts at 53.1916, units not yet verified |
| `L` `R` | **relative** steps on one motor | integer steps |

```
C: SET X 0.5
S: OK SET X 0.5
C: SET X 30
S: ERR OUT_OF_RANGE X must be -25.0..25.0
C: SET W 1
S: ERR BAD_AXIS W
C: SET X 1        (real robot, panel Remote switch off)
S: ERR BUSY operator has control
```
`SET` replies as soon as the target is **accepted**. It does not wait for the motion. See [§8](#8-motion-and-timing).
On the real VI, `SET THETA` / `SET PHI` also switch the front panel's Azimuth/Elevation tab.

### 5.7 `DELTA <dx> <dy> <dz> <d>`: store the tracker offset (no motion)
```
C: DELTA 1.0 -2.0 3.0 3.7
S: OK DELTA 1.0 -2.0 3.0 3.7          ← the four tokens echoed exactly as received
C: DELTA 1 2 3
S: ERR BAD_ARGS ...                   ← not exactly 4 finite numbers, or d < 0
```
Details are in [§9](#9-delta-streaming-the-tracker-offset).

### 5.8 `GET DELTA`
The last stored offset and its age in ms, or `NONE`.
```
C: GET DELTA
S: DELTA 1.0 -2.0 3.0 3.7 0
C: GET DELTA                          (nothing stored yet)
S: DELTA NONE
```

### 5.9 `BYE`
```
C: BYE
S: OK BYE                             (then the server closes this connection; the robot keeps running)
```

### 5.10 `STOP`: **never send from the Mac**
Presses the VI's Stop button and **ends the robot program**. The reply is `OK STOP`. It is listed here only
so that it is recognised and avoided.

### 5.11 Unknown command
```
C: FOO
S: ERR UNKNOWN_CMD FOO (send HELP for the command list)
```

---

## 6. STATUS fields

The fields are always all present, in this order, `;`-separated `key=value`:

| Key | LabVIEW indicator | Meaning |
|---|---|---|
| `t` | – | ms since the server started |
| `x` `y` `z` `psi` | `Real Output x/y/z/psi` | **commanded** position, mm, quantized to whole steps |
| `theta` | `Real Output theta` | commanded azimuth |
| `phi` | `Real Output Phi` | commanded elevation |
| `mx` `my` `mz` `mpsi` | `Moving x/y/z/psi` | 1 while that motor is still stepping |
| `ml` `mr` | `Moving`, `Moving R` | 1 while motor L / R is stepping (THETA and PHI moves drive both) |
| `ox` `oy` `oz` `opsi` `ol` `or` | `Optical X/Y/Z/psi/L/R` | raw optical-sensor voltages, the **only real feedback** |

Important: `x`…`phi` are **not measurements**. They jump to the target as soon as `SET` is accepted. The
only way to know a move has finished is to wait until its `m*` flag reads `0`.

Parsing (MATLAB):
```matlab
kv = split(extractAfter(reply,"STATUS "), ";");
kv = split(kv, "=");
st = containers.Map(kv(:,1), num2cell(str2double(kv(:,2))));
st("mx")    % 0 or 1
```

---

## 7. Axes, units and quantization

From the LabVIEW subVI diagrams (details in [`GUIDE.md`](GUIDE.md)):

| Axis | Unit | Step size | Steps per unit | Range | Start |
|---|---|---|---|---|---|
| X | mm | 2/52 = 0.03846 mm | 26 / mm | −25 … +25 | 0 |
| Y, Z, PSI | mm (provisional) | same as X? | 26 / mm? | ±25? | 0 |
| THETA | (deg?) | k = 10800/(21788·π) ≈ 0.15778 | 1/k ≈ 6.338 | – | −85 |
| PHI | not verified | – | – | – | 53.1916 |
| L, R | motor steps | 1 | – | – | – |

Quantization: the robot computes `steps = round((target − position) / step_size)`. The reported
position becomes `position + steps × step_size`. So `SET X 0.5` gives 13 steps and `x=0.5`, while
`SET THETA -84` gives `theta=-84.0533`.

---

## 8. Motion and timing

| Quantity | Value |
|---|---|
| Motor speed | 1 step per loop period = `1000 / (Step/s)` ms. Panel default **4 steps/s**. |
| Example | `SET X 0.5` = 13 steps → `mx=1` for **≈ 3.25 s** (measured 3.2 s on the simulator) |
| Example | `SET THETA -85` from −84.0533 = 6 steps → `ml=mr=1` for ≈ 1.5 s |
| Status polling | ≤ 20 Hz (the Mac uses 10 Hz in tests) |
| Idle timeout | 10 s (the Mac pings after 4 s of silence) |
| DELTA rate | ≈ 5 Hz |

Typical move:
```
C: SET X 0.5          S: OK SET X 0.5
C: GET STATUS         S: STATUS ...;x=0.5;...;mx=1;...      ← target already shown, still moving
   ... poll every 100 ms ...
C: GET STATUS         S: STATUS ...;x=0.5;...;mx=0;...      ← arrived
```
Tip: after `SET`, wait until the flag has been seen as `1` **or** a short grace time (≈ 0.3 s) has passed
before trusting `0`. Otherwise a status read taken just before the loop picks up the new target can
look like "already done". A `SET` to the current position never sets the flag at all.

---

## 9. DELTA: streaming the tracker offset

### 9.1 What the numbers mean
In the app's **Robot** tab you pick a **marker** (a tracked tool) and type a **target** point (x, y, z).
DELTA carries the marker's offset from that target, the same numbers shown in `TargetUITable`:

| Field | Meaning | Unit |
|---|---|---|
| `dx` | marker − target, rightward in the camera display frame | mm |
| `dy` | marker − target, upward in the camera display frame | mm |
| `dz` | marker − target, distance from the camera (away = positive) | mm |
| `d` | √(dx² + dy² + dz²), the Euclidean distance, always ≥ 0 | mm |

These are in the **camera frame, not the robot frame**. Mapping them to robot axes is a separate, not
yet designed step. (Until 2026-11-17 the offset was taken from the Control tab's Mark ref. point
instead of a typed target.)

### 9.2 Format
`sprintf('DELTA %.1f %.1f %.1f %.1f', dx, dy, dz, d)`, with 0.1 mm resolution. Example: `DELTA 1.2 -0.4 3.1 3.4`.

### 9.3 When it is sent
- Only while the **Comm.** link is on, a **target is set**, and the selected marker is **valid**, i.e. its
  latest pose is less than 0.5 s old. The tracking loop only reports tools it can see. Nothing is sent
  while the marker is hidden. Keepalive PINGs cover those gaps.
- Rate ≈ 5 Hz, driven by the tracking loop (`nav_camera_graphic_update` → `nav_robot_tab(app,'frame',…)`).

### 9.4 Acknowledgement
The robot replies `OK DELTA <the four tokens exactly as received>`. The Mac compares each reply with
`"OK " + <the line it sent>`. Counts of acknowledged, wrong and missing replies are reported when the link is closed.

### 9.5 Robot side
The robot stores the last offset (`GET DELTA` returns it with its age), shows it on the new indicators
`Delta dx/dy/dz/d`, and **never writes any motion target** because of DELTA.

---

## 10. Errors and how the Mac handles them

| Server reply | Cause | Mac behaviour |
|---|---|---|
| `ERR BUSY another client connected` (instead of the greeting) | another client holds the single connection | alert "already in use by another client". No connection is made. |
| greeting third word ≠ `1` | protocol version change | alert "unexpected greeting". No connection is made. |
| `ERR BUSY operator has control` (reply to SET) | real robot, panel **Remote** switch off | info line "refused, the operator has control". The link stays up. |
| `ERR OUT_OF_RANGE ...` | SET outside ±25 mm | shown as "sent / expected / got" in the info box |
| `ERR BAD_AXIS ...` | unknown axis in SET | same |
| `ERR BAD_ARGS ...` | malformed request (e.g. DELTA with ≠ 4 numbers, d < 0) | same; counted as a "wrong reply" for DELTA |
| `ERR UNKNOWN_CMD DELTA` | older server without DELTA | reported **once** ("does not know DELTA"), then counted silently |
| `ERR UNKNOWN_CMD <x>` | typo / unsupported command | shown in the info box |
| no reply / socket error | cable pulled, server restarted, 10 s timeout | "link lost; disconnected". The button returns to **Comm.** |

General rule: an `ERR` reply never closes the connection. Only `BYE`, `STOP`, the idle timeout, or a
transport error does.

---

## 11. Mac implementation (MATLAB)

MATLAB **R2022b** (the version the user runs). It uses `tcpclient`, `configureTerminator`, `readline`
and `writeline`, which are available since R2020b.

### 11.1 Files
| File | Role |
|---|---|
| `codes/nav_robot_tab.m` | the **Robot** tab: marker readout, target, `TargetUITable`, 3D dots, log; decides when to send DELTA |
| `codes/nav_camera_tool_comm.m` | the TCP link: connect, DELTA, keepalive, reply matching, set, close |
| `codes/nav_camera_graphic_update.m` | calls `nav_robot_tab(app,'frame',idx,p)` for every tracked tool and frame |
| `codes/nav_camera_terminal.mlapp` | Robot tab components. `startupFcn` → `nav_robot_tab(app,'init')`. Tool check → `'populate'`. The `MarkerDropDown` / `TargetX/Y/ZEditField` / `RobotCommButton` callbacks call the two modules. |

**Robot tab components** (App Designer names): `MarkerDropDown`, `MarkerX/Y/ZEditField` (read-only),
`TargetX/Y/ZEditField`, `TargetUITable` (dx, dy, dz, d), `RobotCommButton`, `RobotCommTextArea`, `RobotUIAxes`.

> **Self-wiring:** on every camera frame, `nav_camera_graphic_update` calls `nav_robot_tab(app,'ensure')`.
> That runs `'init'` once, connects the Robot tab callbacks (MarkerDropDown, Target fields,
> RobotCommButton, window close) in code, and refills `MarkerDropDown` whenever the tool list
> from Tool check changes. The tab therefore works even when the .mlapp has none of the hook lines,
> which happens because App Designer saves an older open copy over outside edits. Until the first
> frame arrives (BX/BX2 streaming), the dropdown still shows App Designer's default items.

### 11.2 User workflow (Robot tab)
1. On the Control tab, connect the camera and press **Tool check**. This fills `MarkerDropDown`.
2. Start BX/BX2 streaming with repeat on.
3. **Robot** tab, Marker panel: choose the marker. Its x, y, z (mm, display frame) update live and are
   read-only. They are greyed out while the marker is not seen. A dot in the tool's colour follows it in
   `RobotUIAxes`; it turns grey while the marker is not seen and is removed when another marker is chosen.
4. Target panel: type x, y, z. Each edit (re)defines the target and moves the magenta target dot.
   While the marker is valid, `TargetUITable` shows marker − target (dx, dy, dz, d) at 10 Hz.
5. Press **Comm.** to connect. The button changes to **Stop comm.** DELTA (= the table values) then
   streams at ~5 Hz. Changing the marker or target while connected is allowed.
6. Everything is logged in `RobotCommTextArea`, **newest line on top**, with a timestamp and a mark:
   `>` sent, `<` received, `*` status, `!` problem. DELTA traffic is logged once per second; errors always.
7. Press **Stop comm.** (or close the window) to send `BYE`, with a summary of acknowledged, wrong and missing replies.

### 11.3 Module actions
`nav_robot_tab(app, action, ...)`:

| Action | Called by | Does |
|---|---|---|
| `'init'` | startupFcn | read-only marker fields, labels/widths, table columns, axes style, close handler |
| `'populate'` | Tool check | `MarkerDropDown` = tool names, `ItemsData` = 1..n (0 = none) |
| `'marker_changed'` | MarkerDropDown | removes the marker dot, resets the readout, logs |
| `'target_changed'` | TargetX/Y/ZEditField | target = the three fields, draws/moves the target dot, logs |
| `'frame', idx, p` | `nav_camera_graphic_update` | validity, readout + table + dots (10 Hz), `send_delta` (5 Hz) |
| `'log', kind, str` | both modules | adds a timestamped line to `RobotCommTextArea` (and the command window) |

`nav_camera_tool_comm(app, action, ...)`:

| Action | Called by | Does |
|---|---|---|
| `'toggle'` | **Comm.** (`RobotCommButton`) | connect + greeting check + keepalive timer, or close |
| `'send_delta', d` | `nav_robot_tab` | `DELTA dx dy dz |d|` for d = marker − target |
| `'keepalive'` | MATLAB `timer`, every 1 s | reads arrived replies. If there has been ≥ 4 s of silence, sends `PING`. |
| `'set', axis, value` | **explicit only** (code or command window) | `SET <axis> <value>`. **Moves the robot.** |
| `'close'` | **Stop comm.** | waits ≤ 1 s for outstanding replies, `BYE`, summary, stop timer |
| `'close_app'` | window CloseRequestFcn | `'close'`, then `delete(app)` |

Example from the command window, with the app handle `app = nav_camera_terminal;`:
```matlab
nav_camera_tool_comm(app,'set','X',0.5)   % moves X to +0.5 mm (only when you mean it)
```

### 11.4 Reply matching
- Every request goes through `request(app, msg, expect)`, which writes the line and appends
  `msg` / `expect` to the FIFO queues `app.nav_camera.robot_pending` / `robot_expect`.
- `drain_replies` runs before each send and on every timer tick. It reads **only the bytes already
  available** (never blocks the tracking loop) into a buffer, and pops one queue entry per *complete*
  line. A reply split across TCP packets is therefore never read half.
  - The reply matches `expect` → success (DELTA: `robot_n_acked++`).
  - Otherwise → classify (see [§10](#10-errors-and-how-the-mac-handles-them)) and report in `RobotCommTextArea` and the command window.
- Expected replies: `DELTA ...` → `OK DELTA ...` (exact echo), `PING` → `OK PONG`, `SET A v` → `OK SET A v`.

### 11.5 State kept in `app.nav_camera`
| Field | Meaning |
|---|---|
| `robot` | `tcpclient` object (empty when disconnected) |
| `robot_marker_idx`, `robot_target` | selected marker (tool index), target point (3×1 mm, NaN until set) |
| `robot_marker_h`, `robot_target_h` | the two dots in `RobotUIAxes` |
| `robot_rx_buf` | received bytes not yet forming a complete line |
| `robot_pending`, `robot_expect` | FIFO of requests awaiting a reply |
| `robot_n_sent`, `robot_n_acked`, `robot_n_bad` | DELTA counters |
| `robot_last_reply`, `robot_last_tx` | last reply line, `tic` of the last request |
| `robot_timer`, `robot_keepalive_s` | keepalive timer, silence threshold (4 s) |
| `robot_delta_unsupported` | set once the server answers `ERR UNKNOWN_CMD DELTA` |

### 11.6 Choosing the server
```matlab
setenv('TMS_ROBOT_HOST','127.0.0.1')   % use the local simulator (codes/tms_robot_sim.py)
setenv('TMS_ROBOT_HOST','')            % default: 169.254.30.121 (Windows PC)
```
The port is fixed at 6340 in `nav_camera_tool_comm.m`.

---

### 11.7 Goto button (`nav_robot_goto.m`)
`TargetGotoButton` moves the robot with `SET X/Y/Z` until the tracked marker reaches the target.
Press it again to abort. Abort only stops new moves. The robot finishes the move in progress, and
`STOP` is never sent. Goto needs a connection, a chosen marker that is visible, and a target.

Protocol v1 has no "move the marker to here" request, and the robot axes are not aligned with the camera
axes. Goto therefore runs a closed loop on the Mac:

1. **Calibration** is done by the **`CALIBRATE <step mm>`** command (§11.8). Goto also runs it when it has
   no calibration for the current marker (first Goto, marker changed, or after the loop diverged).
   It moves X, Y and Z by +2 mm one after another, or −2 mm when +2 mm would leave the range. After
   each move it waits until `mx=my=mz=0` in `GET STATUS` (polled at 4 Hz), lets the marker settle for
   0.5 s and averages it over 0.5 s. The marker displacements give the Jacobian
   `J` (3×3, marker mm per robot mm). Goto logs `J`. Calibration fails if one axis moves the marker
   less than 0.3 mm per mm, which usually means the marker is not on the robot.
2. **Servo**: `e = marker − target`; stop when `|e| ≤ 0.5 mm`. Otherwise `dr = −J⁻¹ e`, limited to
   10 mm per move. Goto sends `SET` to the axes that change, quantized to 1/26 mm and clamped to ±25,
   then waits for the move and measures the marker again, as in step 1.
3. **Stops**: Goto stops after 8 moves, or when the error grows twice in a row (it then drops `J`).
   It also stops when the move needs a robot axis beyond ±25 mm twice in a row ("out of reach"),
   on a refused `SET` (`ERR BUSY`, `ERR OUT_OF_RANGE`, ...), on a lost marker or link, or when a move
   takes longer than its expected time × 1.5 + 5 s.

Parameters are at the top of `nav_robot_goto.m`. Moves at 4 steps/s are slow: 2 mm takes 13 s, so a
first Goto with calibration takes about a minute on the real robot.

### 11.8 Manual commands (`CommCmdDropDown` + `CommParamEditField`)
- **No parameter** (`PING`, `GET STATUS`, `GET DELTA`, `HELP`): choosing it sends it at once, and the
  dropdown goes back to `(choose)` so the same command can be picked again.
- **With a parameter** (`SET X|Y|Z|PSI|THETA|PHI|L|R`, `GET DOC`): choosing it enables `param`
  (the placeholder shows the expected value). Typing a value and pressing Enter sends it. The field
  then empties, so the next value is sent too.
- `CALIBRATE <step mm>` (0.5..5, e.g. `2`) runs only the calibration of §11.7 (no target needed).
  It moves X, Y and Z by the step in turn and measures how the marker moves in camera coordinates.
  It then logs the Jacobian, moves the robot back to its start position, and keeps the result for
  Goto. The Goto/Abort button aborts it. It is a Mac-side command: the robot only sees `SET` and
  `GET STATUS`.
- `GET DOC <BRIEF|PROTOCOL|GUIDE>` saves the document to `tempdir/tms_robot_<NAME>.md` and logs the path.
- `STOP`, `BYE` and `DELTA` are not in the list. Comm. handles BYE, and DELTA is streamed automatically.

The callbacks are set in `nav_robot_tab(app,'init')`, like the other Robot tab controls, so the
.mlapp needs no callback code for them.

## 12. Robot implementation (LabVIEW / simulators)

### 12.1 Real robot (live since 2026-11-24)
```
Mac (MATLAB) ── text v1 ──▶ TMS Robot Bridge (Windows PC) ── binary 72B/44B ──▶ LabVIEW Main VI (myRIO)
            169.254.30.121:6340                            172.22.11.2:6340 (USB)
```
- The myRIO is reachable only over USB, so the Windows PC runs the **TMS Robot Bridge**
  `TMS_Robot/TMS_Robot_Code/tcp/pc_bridge/tms_bridge.py`. It serves protocol v1 to the Mac on 6340 (the Mac code is
  unchanged), answers PING/HELP/DELTA/GET DELTA/GET DOC itself, and translates GET STATUS and SET for the VI.
  It has an *Allow motion from Mac* switch (off at every start → `ERR BUSY operator has control`), refuses STOP, and
  shows a monitor window with a status history. Greeting: `HELLO TMS-ROBOT 1 bridge (robot connected|NOT connected)`.
  When LabVIEW is down, `GET STATUS`/`SET` → `ERR BUSY robot not connected (...)`.
- **The VI does not speak text v1.** It uses the binary interface of `codes/TMS_MATLAB_Controller/TMS_Start.m`
  (the robot side's own MATLAB controller, which talks to the VI directly from the Windows PC):
  - VI ← 72 bytes = 18 big-endian float32 `[x_speed x_pos 0 y_speed y_pos 0 z_speed z_pos 0 psi_speed psi_pos 0
    tab_mode curve_speed theta_pos 0 phi_pos 0]` (the 0s are jog). tab_mode 0 = THETA, 1 = PHI.
  - VI → 44 bytes every ~100 ms = 11 float32 `[x_mov x y_mov y z_mov z psi_mov psi curve_mov theta phi]`,
    moving **1 = idle**.
- How the bridge maps v1 onto it, and how the real VI behaves (values are targets, moving flag ~1 s late, VI restart
  needed after a disconnect), are in [`WINDOWS_BRIDGE.md`](WINDOWS_BRIDGE.md) §5 and §7. For the Mac this means:
  STATUS positions are targets; `SET L/R` → `ERR BAD_AXIS`; DELTA is acked and kept on the bridge only.
- Start-up steps on Windows: [`WINDOWS_BRIDGE.md`](WINDOWS_BRIDGE.md). The text-v1 VI loop in [`GUIDE.md`](GUIDE.md)
  was not built; the bridge still supports it with `--robot-proto text`.
- Test stand-in for the VI: `TMS_Robot/TMS_Robot_Code/tcp/robot_simulator/lv_binary_sim.py` (reports targets and
  starts motion 1 s late, like the real VI).

### 12.2 Robot-side Python simulator (on the Windows PC)
This is what currently answers on `169.254.30.121:6340`. It implements protocol v1, including DELTA,
GET DELTA, 4 steps/s motion and the `%g` number format.

### 12.3 Mac-side simulator: `codes/tms_robot_sim.py`
Use it when the Windows PC is not available.
```sh
python3 codes/tms_robot_sim.py -v                 # 127.0.0.1:6340, Remote ON, DELTA enabled
python3 codes/tms_robot_sim.py --operator -v      # start with Remote OFF (SET -> ERR BUSY)
python3 codes/tms_robot_sim.py --speed 26         # faster motors (26 steps/s = 1 mm/s on X)
python3 codes/tms_robot_sim.py --no-delta         # behave like a server without DELTA
kill -USR1 <pid>                                  # toggle the Remote switch while running
```
It mirrors protocol v1: greeting, HELP, HELLO, PING, GET STATUS / DELTA / DOC (served from this
folder), SET with range/axis checks and step quantization, DELTA with BAD_ARGS rules, one client at a
time, 10 s idle disconnect, BYE, and STOP (which exits the simulator).

### 12.4 Refresh the robot-side docs
```sh
python3 - <<'EOF'
import socket
s = socket.create_connection(("169.254.30.121", 6340), timeout=5); f = s.makefile("rb"); f.readline()
for name in ("BRIEF", "PROTOCOL", "GUIDE"):
    s.sendall(f"GET DOC {name}\r\n".encode())
    n = int(f.readline().split()[2])
    open(f"doc/tms_robot/{name}.md", "wb").write(f.read(n))
s.sendall(b"BYE\r\n"); f.readline()
EOF
```

---

## 13. Testing recipes

### 13.1 Quick manual check (Terminal)
```sh
nc -vz 169.254.30.121 6340                                    # port reachable?
printf 'PING\r\nGET STATUS\r\nBYE\r\n' | nc -w 3 169.254.30.121 6340
```
Expected:
```
HELLO TMS-ROBOT 1 ...
OK PONG
STATUS t=...;x=0;...
OK BYE
```

### 13.2 Move and wait (Python)
```python
import socket, time
s = socket.create_connection(("169.254.30.121", 6340), timeout=5); f = s.makefile("rb"); f.readline()
def tx(c): s.sendall((c + "\r\n").encode()); return f.readline().decode().strip()
print(tx("SET X 0.5")); t0 = time.time(); seen = False
while True:
    st = dict(p.split("=") for p in tx("GET STATUS")[7:].split(";"))
    seen |= st["mx"] == "1"
    if st["mx"] == "0" and (seen or time.time() - t0 > 0.3): break
    time.sleep(0.1)
print(f"done in {time.time() - t0:.2f} s"); tx("SET X 0"); tx("BYE")
```

### 13.3 MATLAB without the app
```matlab
c = tcpclient("169.254.30.121", 6340, "Timeout", 2);
configureTerminator(c, "CR/LF");
readline(c)                         % HELLO TMS-ROBOT 1 ...
writeline(c, "DELTA 1.0 -2.0 3.0 3.7"); readline(c)   % OK DELTA 1.0 -2.0 3.0 3.7
writeline(c, "GET DELTA");              readline(c)   % DELTA 1.0 -2.0 3.0 3.7 <age>
writeline(c, "BYE");                    readline(c)   % OK BYE
clear c
```

### 13.4 Full app path without hardware
1. `python3 codes/tms_robot_sim.py -v`
2. In MATLAB: `setenv('TMS_ROBOT_HOST','127.0.0.1')`, start `nav_camera_terminal`, connect the camera, then
   Tool check → BX2 + repeat → **Robot** tab: choose the marker, type a target → **Comm.**
3. The simulator prints each `delta dx=… dy=… dz=… |d|=… mm`. Pressing **Stop comm.** prints the ack summary in MATLAB.

---

## 14. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Cannot connect to 169.254.30.121:6340` | cable/adapter down, IP changed, server not running | `ifconfig en23` (status active?). Re-discover the IP ([§2](#2-physical-network)). Ask the robot side to start the server. |
| Connects, then "already in use by another client" | another client (an old MATLAB session, a test script) still connected | close the other client. In MATLAB, `clear` the stale `tcpclient`. |
| Link drops after ~10 s when idle | no keepalive | make sure the Comm. timer is running (`timerfindall('Name','nav_camera_tool_comm keepalive')`) |
| `ERR UNKNOWN_CMD HELLOW` and similar | typo, or text sent without CRLF splitting | send exact commands with `\r\n` |
| All DELTA counted as "wrong reply" | server without DELTA, or the reply isn't an exact echo | check with `GET DELTA` / `HELP`. Compare the echoed tokens. |
| `SET` refused with `ERR BUSY operator has control` | panel Remote switch off | the operator turns Remote on, or the move is done from the panel |
| `mx` reads 0 right after `SET` | the target equals the current position, or the status was read before the loop picked up the target | see the tip in [§8](#8-motion-and-timing) |
| Nothing is sent although Comm. is on | no target typed yet, or the marker is not visible (greyed readout) | type x, y, z in the Target panel; check the marker in view of the camera |
| `192.168.50.149` unreachable | that subnet isn't configured on en23 | use 169.254.30.121, or add an alias ([§2](#2-physical-network)) |

---

## 15. Safety rules

1. **Never send `STOP`.** It ends the robot program.
2. **`SET` moves hardware.** The Mac never sends it automatically. It is sent only through the explicit
   `'set'` action, and only when a person asks for that move.
3. **`DELTA` must never cause motion** on the robot side.
4. Before the first test on the real robot, follow the robot side's test plan: joints free or de-energized,
   start with `HELP` / `PING` / `GET STATUS`, then a small `SET X 0.5`.
5. Respect the operator: if the panel's Remote switch is off, do not retry `SET` in a loop.
6. One client at a time. Close test scripts (`BYE`) before starting the app.

---

## 16. Open items and change log

### Open / unverified
- Y, Z and PSI limits and resolution: assumed equal to X (±25 mm, 26 steps/mm), not yet verified.
- PHI units: not verified.
- Mapping DELTA (camera frame) to robot axes: not designed. Any closed-loop use needs a calibration
  between the Polaris frame and the robot frame, plus an explicit safety review.
- Real robot: single X moves from the Mac verified on 2026-11-24. Y/Z/PSI/THETA/PHI moves, CALIBRATE and Goto
  not yet run on the real robot. The VI's TCP loop must be restarted after each disconnect (to be fixed on the VI).

### Change log
| Date | Change |
|---|---|
| 2026-11-17 | Link found: en23 ↔ Windows PC 169.254.30.121, server on 6340 (`HELLO TMS-ROBOT 1`). |
| 2026-11-17 | Mac: ToolDropDown + Comm. button stream `DELTA` (~5 Hz) with an echo ack. Keepalive PING, version check. |
| 2026-11-17 | Robot side: `DELTA` and `GET DELTA` added (still v1). PHI starts at 53.1916. 4 steps/s motion. `%g` numbers. Greeting version = third word. Remote switch planned (`ERR BUSY operator has control`). |
| 2026-11-17 | Mac: `'set'` action, ERR BUSY handling, simulator updated to match. This document written. |
| 2026-11-18 | Mac en23 address changed to 169.254.240.129; link re-verified (robot docs unchanged). Simulator fixes: motion stalled at low speeds (fractional steps were dropped), and quantization is now anchored at the start value like the VI (THETA −85 → −84 gives −84.0533). `'set'` with a bad axis or value reports and sends nothing instead of throwing. Tested in MATLAB R2022b: 6/6 DELTA acked by both servers; SET → `ERR BUSY operator has control` handled. |
| 2026-11-18 | App: new **Robot** tab (`nav_robot_tab.m`) with marker readout, target, marker − target table, 3D dots and a comm log. DELTA now carries marker − target (was the Mark ref offset). Replies are buffered by complete line. |
| 2026-11-18 | Robot tab made self-wiring from the tracking loop (App Designer saves kept removing the hook lines from the .mlapp). |
| 2026-11-19 | **Goto** button (`nav_robot_goto.m`: closed loop with auto-calibrated camera↔robot Jacobian) and manual **cmd./param** commands. Comm layer: quiet requests, STATUS parsing (`robot_status`), SET refusals recorded (`robot_set_err`), prefix reply matching, multi-line `GET DOC`. Fix: a request is queued *before* `writeline`. Before, a reply read by another timer callback during the write found no pending request, and all later replies were mismatched. Tested in R2022b against the simulator with a rotated/flipped camera frame: 0.08–0.16 mm in 2–3 moves; out-of-reach and abort cases; 113/113 DELTA acked. |
| 2026-11-19 | `CALIBRATE <step mm>` command in cmd./param: only the camera↔robot calibration, then back to the start, result kept for Goto. Simulator: J within 0.06 of the true mapping; the next Goto skipped calibration and reached 0.23 mm in 2 moves. |
| 2026-11-19 | Windows side: **TMS Robot Bridge** `tms_bridge.py` (standalone, tkinter monitor, Remote switch, STOP blocked, built-in sim). Tested on the Mac: protocol cases (one client, idle drop, DELTA rules, GET DOC framing, LabVIEW down/up auto-reconnect), and MATLAB CALIBRATE + Goto through the bridge unchanged (27/27 DELTA acked, 0.32 mm). Not yet run on Windows. Mac: ERR BUSY on SET now shows the server's reason. |
| 2026-11-24 | Windows bridge **live with the real robot.** The VI turned out to use the binary 72B/44B interface of `codes/TMS_MATLAB_Controller` (added to the repo), so the bridge got a binary LabVIEW link (default; text v1 kept as `--robot-proto text`) and `lv_binary_sim.py`. Real-robot findings: VI values are targets, moving flag rises ~1 s after a packet, start state all 0, VI TCP loop stops after a disconnect. Bridge fixes: stay connected when the VI is silent; `--init` start targets; adopt the VI's targets for axes it is not moving (a first `SET X 0.5` had sent Y 3 → 0, overwriting a panel value); done = moving flag rose and fell; status history in the log. Verified: `SET X 0.5` / `SET X 0` done at 4.3 / 4.1 s, other axes unchanged. Guide: [`WINDOWS_BRIDGE.md`](WINDOWS_BRIDGE.md). |
