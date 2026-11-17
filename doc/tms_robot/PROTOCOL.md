# TMS Robot TCP protocol — v1

Shared spec between the **robot side** (LabVIEW `TMS Robot_adjust.vi` on the NI myRIO, or the Python
simulator) and the **Mac side** (client). Both Claude agents should treat this file as the contract;
change the version number if it changes.

## Roles and transport
- **Server = robot side** (myRIO / LabVIEW `TCP Listen`). **Client = Mac.**
- TCP, default port **6340**.
- ASCII text, one message per line, terminated by **`\r\n`** (CRLF). LabVIEW `TCP Read` in *CRLF mode*
  needs the CR; the server accepts a bare `\n` too, but clients must send `\r\n`.
- One request → exactly one response line. Max line length 512 bytes.
- Numbers: decimal, `.` as decimal point, no thousands separators. Numbers sent by the server use
  `%g` style: up to 6 significant digits and no trailing zeros (`0`, `0.5`, `-83.7377`, `53.1916`).
  Clients must accept any decimal form.
- The greeting's **third word is always the protocol version** (`HELLO TMS-ROBOT 1 ...`). Text after it is free-form.
- Case-insensitive commands; responses are upper-case keywords.

## Connection
On accept the server sends a greeting:
```
HELLO TMS-ROBOT 1
```
(`1` = protocol version.) The client should check the version.

## Requests
| Request | Meaning | Maps to front-panel item |
|---|---|---|
| `HELP` | one-line summary of all commands | – |
| `GET DOC <BRIEF\|PROTOCOL\|GUIDE>` | reply `DOC <name> <nbytes>\r\n` then exactly `<nbytes>` raw UTF-8 bytes (the only multi-line reply) | – |
| `HELLO ...` | optional client greeting; replied with `OK HELLO TMS-ROBOT 1` | – |
| `PING` | liveness check | – |
| `DELTA <dx> <dy> <dz> <d>` | Polaris tracker offset of the tool from the reference, in mm (camera display frame: dx right, dy up, dz distance from camera; d = √(dx²+dy²+dz²)). **Stored/displayed only. Never moves the robot.** About 5 Hz while the tool is visible. Reply `OK DELTA` echoes the four tokens exactly as received. Not 4 finite numbers, or d < 0 → `ERR BAD_ARGS`. | new indicators `Delta dx/dy/dz/d` |
| `GET DELTA` | last stored offset: `DELTA <dx> <dy> <dz> <d> <age_ms>`, or `DELTA NONE` | – |
| `GET STATUS` | read all indicators | see status fields |
| `SET X <value>` | target for X axis | `User Input x` |
| `SET Y <value>` | target for Y axis | `User Input y` |
| `SET Z <value>` | target for Z axis | `User Input z` |
| `SET PSI <value>` | target for Psi axis | `User Input psi` |
| `SET THETA <value>` | azimuth target | `User Input theta` |
| `SET PHI <value>` | elevation target | `User Input Phi` |
| `SET L <value>` | relative steps, L motor | `Motor Input L` |
| `SET R <value>` | relative steps, R motor | `Motor Input R` |
| `STOP` | presses the VI **Stop** button. **Ends the robot program.** | `Stop` |
| `BYE` | server replies, then closes this connection (robot keeps running) | – |

Units (from the subVI diagrams, `docs/LabVIEW_TCP_server_guide.md`):
- **X** (and probably Y, Z, Psi; not yet verified): absolute target in **mm**, resolution 2/52 mm
  (26 steps/mm), allowed range **−25…+25**. Out-of-range values get `ERR OUT_OF_RANGE`.
- **THETA**: absolute target, resolution ≈ 0.15778 per step (`10800/(21788·π)`). Starts at −85.
- **PHI**: absolute target (start value 53.19). Units not yet verified.
- **L, R**: relative motor steps.
- `x`, `theta` and the other positions in STATUS are the **commanded** position, quantized to whole steps,
  not measured positions. Use the `m*` flags to know when motion has finished.

## Responses
```
OK PONG
OK SET X 120
OK STOP
OK BYE
ERR <CODE> <free text>
STATUS key=value;key=value;...
```
Error codes: `UNKNOWN_CMD`, `BAD_ARGS`, `BAD_AXIS`, `OUT_OF_RANGE`, `BUSY`.

### STATUS fields (fixed order, all present)
```
STATUS t=<ms since start>;x=<Real Output x>;y=..;z=..;psi=..;theta=<Real Output theta>;phi=<Real Output Phi>;
       mx=<Moving x 0|1>;my=..;mz=..;mpsi=..;ml=<Moving 0|1>;mr=<Moving R 0|1>;
       ox=<Optical X>;oy=..;oz=..;opsi=..;ol=..;or=..
```
(All on one line in practice. The line breaks above are only for readability.)

## Example session
```
S: HELLO TMS-ROBOT 1
C: PING
S: OK PONG
C: SET THETA -60
S: OK SET THETA -60
C: GET STATUS
S: STATUS t=10234;x=0;y=0;z=0;psi=0;theta=-84.0533;phi=53.1916;mx=0;my=0;mz=0;mpsi=0;ml=0;mr=0;ox=0.12;oy=0.10;oz=0.11;opsi=0.09;ol=0.13;or=0.12
C: BYE
S: OK BYE
```

## Rules
- Only **one client** at a time. A second connection receives `ERR BUSY another client connected` and is then closed.
- A client should poll with `GET STATUS` at no more than 20 Hz.
- If the server receives nothing for **10 s**, it may close the connection. Clients should `PING` while idle.
- Motion timing: each motor makes one step per loop period, `1000 / Step/s` ms. The panel default is
  4 steps/s, so `SET X 0.5` (13 steps) keeps `mx=1` for about 3.25 s. The reported position jumps to the
  quantized target at once; the `m*` flag is the only way to tell that the motor has arrived.
- Real robot: if the operator has the panel's **Remote** switch off, `SET` gets `ERR BUSY operator has control`.
  `GET STATUS`, `DELTA` and `PING` always work.
- `SET` returns as soon as the target is accepted. It does not wait for motion to finish. Poll the `m*` flags to see when motion is done.
