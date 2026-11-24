# TMS Robot_adjust.vi — what we could read from the files

The VIs are binary LabVIEW 2018 (18.0) files. LabVIEW is not available on this PC session, so this
analysis comes from decoding the VI resource blocks: front-panel labels, linked subVIs, and
diagram strings. **Wiring and numeric constants (step rates, limits, which DIO pin drives which
motor) cannot be read reliably without LabVIEW.** To fill those gaps, open the VI in LabVIEW and
use *File → Print… → VI documentation → HTML*, which exports block-diagram images, or take
screenshots of the block diagram.

## Deployment target
| Item | Value | Source |
|---|---|---|
| Hardware | NI myRIO-1900 (`NI-myRIO-1900-0328bb07`) | `TMS Robot.lvproj` |
| myRIO IP (USB link) | `172.22.11.2` | `TMS Robot.aliases` |
| Host PC IP | `192.168.50.149` | `TMS Robot.aliases` |
| VI location in project | under the **myRIO target**, so it runs on the myRIO and not on the PC | lvproj |
| LabVIEW version | 18.0 (2018) | `vers` block |
| VI server TCP on target | disabled (port 3363) | lvproj |

## Structure (inferred)
- **Timed Loops** with a 1 kHz clock (the `Timing.Period` and `1 kHz` timing-source strings).
  Each axis seems to have its own timed loop that calls that axis's subVI.
- **myRIO Express I/O**: `myRIO AI Open` + `AI Read` (analog inputs) and `myRIO DIO Open` +
  `DIO Write` (digital outputs that drive the stepper coils), followed by `Close`.
- **Tab control** with two pages, **Azimuth** and **Elevation**.

## Axes / subVIs
| Axis | Motion subVI | Stop subVI | Front-panel controls (inputs) | Indicators (outputs) |
|---|---|---|---|---|
| X | `Single Axis motor_X(SubVI).vi` | `Motor stop_X` | User Input x, Motor Input x, Step/s x, Forward x / Backward x | Real Output x, Motor Position x, Motor State x, Step diff x, Moving x, Optical X, Stop number x |
| Y | `Single Axis motor_Y(SubVI).vi` | `Motor stop_Y` | User Input y, Motor Input y, Step/s y, Left y / Right y | Real Output y, Motor Position y, Motor State y, Step diff y, Moving y, Optical Y, Stop number y |
| Z | `Single Axis motor_Z(SubVI).vi` | `Motor stop_Z` | User Input z, Motor Input z, Step/s z, Up z / Down z | Real Output z, Motor Position z, Motor State z, Step diff z, Moving z, Optical Z, Stop number z |
| Psi | `Single Axis motor_Psi(SubVI).vi` | `Motor stop_Psi` | User Input psi, Motor Input psi, Step/s psi, Left psi / Right psi | Real Output psi, Motor Position psi, Motor State psi, Step diff psi, Moving psi, Optical psi, Stop number psi |
| L | `Single Axis motor_L(SubVI).vi` | `Motor stop_L` | Motor Input L, Step/s curve | Step difference L, Moving, Optical L, Stop number L |
| R | `Single Axis motor_R(SubVI).vi` | `Motor stop_R` | Motor Input R | Step difference R, Moving R, Optical R, Stop number R |
| Theta (azimuth) | `Theta Calculation_adjust(SubVI).vi` | – | User Input theta, Left/Right | Real Output theta |
| Phi (elevation) | `Phi Calculation(SubVI).vi` | – | User Input Phi, Up/Down | Real Output Phi |

The global **Stop** button ends the program.

### SubVI internals (from shift-register / label names)
- `Single Axis motor_*`: these keep state between calls in shift registers (`Position=0`, `State=1`,
  `Motor=0`). Each call steps a **motor state machine** (the "State Chart" with up/down/left/right
  phases) one step toward `User Input`, and outputs `Real Output`, `Step diff` and `Moving`.
  The X/Y/Z/Psi versions track an absolute position. The L/R versions are relative (`Motor Input`
  only).
- `Motor stop_*`: takes `Stop number`, outputs `Loop stop` and de-energizes the coils
  (`Motor, C / I / O`).
- `Theta Calculation_adjust`: theta position memory starts at **−85** (`Position=-85`) and steps with
  `Next/Previous`.
- `Phi Calculation`: phi position memory starts at **0**.
- `Phi.txt`: 190 values that fall smoothly from 51.70 to about 48.9. This looks like a phi
  lookup or calibration table; how the VI uses it is not confirmed.

## Differences between `TMS Robot_adjust.vi` and `TMS Robot.vi`
| `TMS Robot.vi` (original) | `TMS Robot_adjust.vi` |
|---|---|
| Homing x / y / z / psi / (L) buttons | Replaced by **Optical X/Y/Z/psi/R/L** indicators (analog optical sensors read through myRIO AI) |
| `Theta Calculation(SubVI).vi` | `Theta Calculation_adjust(SubVI).vi` |
| Tab page "Elevation" constant | Tab page "Azimuth" constant |

## Confirmed from the block diagram (`vi_export/TMS_Robot__adjustd.png`)

### I/O map (myRIO)
| Joint | Coil/driver outputs (DIO) | Optical sensor (AI) |
|---|---|---|
| X | A/DIO4, A/DIO5 | A/AI0 |
| Y | A/DIO2, A/DIO3 | A/AI1 |
| Z | A/DIO0, A/DIO1 | A/AI2 |
| Psi | B/DIO0, B/DIO1 | B/AI2 |
| L | B/DIO4, B/DIO5 | B/AI0 |
| R | B/DIO2, B/DIO3 | B/AI1 |

### Loop structure
- On startup, an initialization column writes local variables: `Stop number *` = −2, `Step/s *` = 4,
  `User Input theta` = 0, `User Input Phi` = 53.19163884, `User Input x/y/z/psi` = 0,
  `Motor Input *` = 0, `Tab Control` = Azimuth.
- Each joint has a **Timed Loop** on the 1 kHz clock. Each iteration sets the next period
  `dt = 1000 / (Step/s)` ms, so one motor step per iteration. The default 4 steps/s is 250 ms per step.
  Shift registers start at `State = 1`, `Motor = 0`, `Position = 0`.
- Each iteration calls `Single Axis motor_<axis>` with:
  - the **values** of `User Input`, `Motor Input`, `Step diff` and the two direction buttons;
  - **control references** (refnums) to `Motor Input`, `User Input` and `Step diff`. The subVI writes back
    into these front-panel controls itself.
- The outputs are two booleans (`up/left`, `down/right`). These go to `DIO Write` on the joint's two DIO
  lines, and to `Moving`, `Motor State`, `Real Output` and `Motor Position`.
- **Theta (azimuth)** sits in a case structure on `Tab Control` inside the **L loop**.
  `Theta Calculation_adjust` takes `User Input theta` and the Left/Right buttons. It writes
  `Real Output theta` and, by reference, `Step difference L` and `Step difference R`. So theta is
  produced by moving the L and R motors together, paced by `Step/s curve`. Phi is the "Elevation" case.
- Every motion loop stops on the global `Stop`. A second timed loop per joint (100 ms period) then
  runs `Motor stop_<axis>` (`Stop number`), writes the final DIO states, and closes the DIO channels.

## Open questions (need LabVIEW or screenshots)
1. Which myRIO DIO pins drive each motor, and which AI channels feed each optical sensor.
2. Units of User Input (steps, mm, or degrees) and the travel limits for each axis.
3. Whether **Stop** stops only motion or ends the whole VI. The label suggests it ends the VI.
4. How `Phi.txt` is loaded.
