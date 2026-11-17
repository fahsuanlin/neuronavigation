# Adding programmatic (TCP) control to TMS Robot_adjust.vi

## Key finding: no button changes needed
The subVI diagrams (exported to `vi_export/`) show how a joint actually moves:

**`Single Axis motor_X(SubVI)`**, which runs once per motor step:
1. `Forward x` pressed: `User Input x += 2/52`, coerced to [−25, 25]. `Backward x` pressed: `−= 2/52`.
   The subVI writes this back through the `User Input` control reference. **The buttons only nudge
   the target.**
2. `steps = round((User Input − Position) × 26)`. `Position` and `Real Output` become
   `Position + steps × 2/52`, i.e. the target quantized to whole steps. `−steps` is written to `Step diff`.
3. A nonzero `Step diff` is added into `Motor Input` (the target step count).
4. The `Motor` counter moves ±1 per call toward `Motor Input`. `Moving = (Motor ≠ Motor Input)`.
   `Motor mod 4` picks the coil pattern (down/up/right/left), which drives the joint's DIO pair.

So **`User Input x` is an absolute target in mm** (26 steps/mm, 2 mm per 52 steps). Writing a new
value into it is exactly equivalent to an operator typing it or holding Forward/Backward.

**`Theta Calculation_adjust(SubVI)`** works the same way. `User Input theta` is the absolute target, and one step is
`k = 10800 / (21788·π) ≈ 0.15778`. `steps = round((User Input − Position) / k)` goes to
`Step difference L/R`, which turns the L and R motors together. Position starts at −85.

> Not yet verified: the Y/Z/Psi subVIs were not exported. They look like copies of X, but their
> constants (26 steps/mm, ±25 limit) may differ. Check them before relying on mm for those axes.
> Phi (Elevation tab) was not exported either.

**Real Output is the commanded (quantized) position, not a measured one.** `Moving` tells you when
the motor has finished. The only feedback sensors are `Optical *` (raw AI volts).

## What to add in LabVIEW (one new loop, existing loops untouched)
Place a plain **While Loop** on the main diagram, outside the timed loops. Give it the same Stop
condition: read the `Stop` local and OR it with the loop's error.

```
TCP Listen (port 6340, timeout -1)  ->  connection
  TCP Write  "HELLO TMS-ROBOT 1\r\n"
  inner While Loop:
     TCP Read  (mode = CRLF, bytes = 512, timeout = 100 ms)
        error 56 (timeout)       -> clear error, check Stop, loop again
        error 62 / 66 (closed)   -> exit inner loop, TCP Close, go back to Listen
     Trim Whitespace -> Scan From String "%s %s %f"  (cmd, axis, value)
     Case structure on cmd (upper-cased):
        "PING"   -> reply "OK PONG"
        "SET"    -> inner Case on axis:
                    "X"     -> In Range and Coerce(value, -25, 25) -> local var  User Input x   (write)
                    "Y"     -> ... User Input y
                    "Z"     -> ... User Input z
                    "PSI"   -> ... User Input psi
                    "THETA" -> ... User Input theta   (and set Tab Control = Azimuth)
                    "PHI"   -> ... User Input Phi     (and set Tab Control = Elevation)
                    "L","R" -> Motor Input L/R (local read + value -> local write)
                    Default -> "ERR BAD_AXIS"
                    reply "OK SET <axis> <value>"
        "GET"    -> Format Into String:
                    "STATUS t=%d;x=%g;y=%g;z=%g;psi=%g;theta=%g;phi=%g;mx=%d;my=%d;mz=%d;mpsi=%d;ml=%d;mr=%d;ox=%g;oy=%g;oz=%g;opsi=%g;ol=%g;or=%g"
                    from locals: Real Output x/y/z/psi, Real Output theta, Real Output Phi,
                    Moving x/y/z/psi, Moving, Moving R (Boolean To (0,1)), Optical X/Y/Z/psi/L/R
        "DELTA"  -> Scan From String "%f %f %f %f" on the rest of the line
                    ok    -> write indicators Delta dx / Delta dy / Delta dz / Delta d (+ timestamp)
                             reply "OK DELTA " + the 4 tokens exactly as received
                    error -> "ERR BAD_ARGS ..."      (NEVER writes any User Input: no motion)
        "BYE"    -> reply "OK BYE", exit inner loop
        "STOP"   -> write Stop local = TRUE, reply "OK STOP"
        "HELP"   -> reply with the one-line command list
        Default  -> "ERR UNKNOWN_CMD <cmd> (send HELP for the command list)"
     TCP Write  reply + "\r\n"
  TCP Close
```

Notes:
- The greeting must keep the version as the third word (`HELLO TMS-ROBOT 1`). The Mac checks only that word.
- Format numbers with `%g` so they match the simulator (`0`, `0.5`, `-83.7377`).
- **Operator vs. TCP (the Mac agent asked about this):** a TCP `SET` writes the same `User Input` control
  the operator types into, and the Forward/Backward buttons also edit that control. Whoever writes last
  wins; nothing locks them against each other. `SET THETA`/`SET PHI` also flip `Tab Control`. Only the
  selected tab's calculation (Azimuth = theta, Elevation = phi) runs in the L loop, so a remote command
  changes the operator's view and suspends the other angle's calculation.
  **Recommendation:** add a front-panel **Remote** switch (default OFF). When it is OFF, `SET` replies
  `ERR BUSY operator has control`, while `GET STATUS`, `DELTA` and `PING` keep working. Only one side
  then drives motion at a time.
- Make locals from the existing controls (right-click the terminal, then *Create » Local Variable*).
  `User Input x` and the other inputs are numerics, so writing them from code is allowed.
  Latch-button restrictions don't apply because no buttons are written.
- `Stop`: the diagram already writes a `Stop` local at startup, so Stop is not a latch button and can
  be written. Confirm this under its *Mechanical Action*.
- Optional: write with a **Value (Signaling)** property node instead of a local to also fire events.
  This VI has no event structure, so a local is enough.
- **Phi / Elevation:** the Phi case lives in the L loop's Tab Control case structure, so it only runs
  when the tab shows Elevation. That's why SET THETA and SET PHI also set the tab.
- The myRIO must be reachable from the Mac. The myRIO is USB-only (172.22.11.2), so either run
  `tcp/pc_bridge/tcp_bridge.py --listen 169.254.30.121:6340 --target 172.22.11.2:6340` on the PC or
  put the myRIO on the wired network. While the bridge uses port 6340, stop the Python simulator.

## Test plan
1. Run the VI with the robot de-energized or the joints free, from the Mac: `HELP`, `PING`, `GET STATUS`.
2. Send `SET X 0.5`. Expect `Moving x` = 1 for about 13 steps (0.5 × 26), then `Real Output x` = 0.5.
3. Compare against the front panel: typing in the panel and sending over TCP must give the same motion.
