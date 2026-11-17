# Camera-guided navigation with the NDI Polaris Vega

This guide explains how to connect MATLAB to the NDI Polaris Vega camera and stream tool poses. It then covers marking anatomical landmarks on a head mesh (`prep_landmark.m`), registering the subject's head to the MRI with a pointer, and following the brain as the head moves.

## Files

| File | What it does |
|---|---|
| `nav_camera_terminal.mlapp` | The App Designer app: connects to the camera, sends commands, streams tool poses, and displays the tools (Camera tab) and the brain (Brain tab) |
| `nav_camera_decode.m` | Decodes camera replies: `BX`, `BX2` (binary, CRC-checked), `PHSR`, `PHINF`, … |
| `nav_quat_rot.m` | Converts a quaternion to a rotation matrix and Euler angles |
| `nav_camera_graphic_update.m` | Called once per tool per frame: updates the tool dots, auto-fits the axes, refreshes Mark ref, samples landmarks, moves the brain |
| `nav_camera_mark_ref.m` | **Mark ref.** button: freezes the current tool positions as reference points and shows the offsets (dx, dy, dz) |
| `nav_brain_landmark.m` | Brain tab landmarks: loads the landmark file, samples each landmark with the pointer, registers MRI ↔ head tracker |
| `prep_landmark.m` | Offline step: marks the anatomical landmarks on the scalp mesh and saves them to `fhlin_landmark.mat` |
| `lh.seghead.tri` | Scalp (head) mesh of the subject, used by `prep_landmark.m` |
| `fhlin_landmark.mat` | Landmarks made by `prep_landmark.m`; the app reads the variable `sensor` |

## Requirements

- **MATLAB.** Tested with R2025b. You need the **Instrument Control Toolbox** for `tcpip`.
- **[fhlin_toolbox](https://github.com/fahsuanlin/fhlin_toolbox) on the MATLAB path.** It provides `etc_render_topo`, `etc_render_fsbrain`, `inverse_read_tri` and `read_surf`.
- **The subject's FreeSurfer surfaces.** You need a scalp mesh for the landmarks and a brain surface (e.g. `lh.pial`) for display. **Both must be in the same FreeSurfer (tkRAS) space.**
- **The tools loaded and enabled on the camera.** NDI Track does this with each tool's `.rom` file. You can keep NDI Track running alongside the app.

---

## 1. Connect to the camera

1. Plug the Vega into the Mac's Ethernet port. The camera uses a link-local address (169.254.x.x) and announces itself as **`P9-xxxxx.local`**, where the number is the camera's serial number. Our camera is `p9-23261.local`.
2. If you don't know the name, find it in a terminal:
   ```
   arp -an | grep 169.254     # the camera's link-local IP
   ping p9-23261.local        # check that the name resolves
   ```
   Use `arp -an`, not `arp -a`. Without `-n`, `arp` looks up a name for every entry and can hang for several seconds.
3. Open `nav_camera_terminal.mlapp` and run it.
4. Enter **IP**: `p9-23261.local` (or the 169.254.x.x address). Enter **Port**: `8765`.
5. Click **Connect**. The lamp turns green and the button changes to **Disconnect**.

About the connection:
- The camera closes a connection after **300 s of inactivity** (`Param.Connect.Idle Timeout`). The app reopens the connection automatically when you click Send or Tool check after more than 240 s of idle time, so you don't need to reconnect by hand.
- If the camera stops answering, click Disconnect, then Connect.
- If the camera is not tracking yet (no NDI Track running), send **INIT** and then **TSTART** from the Command dropdown first.

## 2. Check the tools

1. Click **Tool check**.
2. The app sends `PHSR 00` (list port handles), then `PHINF <handle>0004` for each handle to read the tool's part number (e.g. `P-1735`).
3. The tool table fills with one row per tool, each with its own colour. The same colour is used for that tool's dot in the Camera tab.
4. The **Brain tool** and **Landmark tool** dropdowns in the Brain tab are filled with the same list.

Run Tool check again whenever tools are added or removed on the camera.

## 3. Stream tool positions (Camera tab)

1. Choose **BX2** in the Command dropdown. The parameter field fills in `--6d=tools --1d=none`. **BX** also works.
2. Set **repeat** to **On** and click **Send**. The app now streams continuously.
3. To stop, set **repeat** to **Off**. The loop ends within a frame.

Each tool appears as a dot in the 3D axes, in its table colour. The display axes are:

| Display axis | Meaning | Camera axis (Vega frame) |
|---|---|---|
| X | right (viewer's right, facing the camera) | −y |
| Y | up | −x |
| Z | distance from the camera (mm) | −z |

All units are mm. The axes fit themselves to where the tools are: they grow but never shrink.

### Mark ref.

1. Click **Mark ref.** while streaming. Each visible tool's current position becomes its reference point, drawn as a **blue** dot.
2. A new live dot keeps tracking the tool.
3. The table under the button shows each tool's offset from its reference (dx right, dy up, dz distance, and the length |d|, in mm). It updates about 5 times per second.

Click Mark ref. again to set a new reference.

---

## 4. Prepare the landmarks (offline, once per subject): `prep_landmark.m`

`prep_landmark.m` follows the fhlin_toolbox wiki page [EEG: prepare topography rendering](https://github.com/fahsuanlin/fhlin_toolbox/wiki/EEG:-prepare-topography-rendering). It uses the same tools as for EEG electrodes, applied to anatomical landmarks.

1. In MATLAB, `cd` to the folder containing the subject's scalp mesh. The script reads `lh.seghead.tri`; edit `surfin_os` at the top of the script for another subject.
2. Edit the landmark names in `electrodes_init` if needed. The defaults are:
   ```matlab
   electrodes_init={'LPA'; 'RPA'; 'Nasion'; 'Nose tip'};
   ```
3. Run `prep_landmark`. The scalp is rendered with `etc_render_topo`.
4. Mark each landmark:
   - Rotate the head to a convenient view and **click the landmark** on the scalp.
   - Press **`b`** over the figure to open the sensor window. You only need to do this once.
   - Click **add (+)** and type the landmark's name. The clicked point is stored under that name.
   - To fix a landmark, select it in the list, click the new location, and update it. Use **−** to delete one.
5. Click **save** in the sensor window and choose **`fhlin_landmark.mat`**. This writes the variable
   ```matlab
   sensor.name    % {'Nasion'; 'Nose tip'; 'LPA'; 'RPA'}
   sensor.coords  % n x 3, mm, MRI (tkRAS) coordinates of the landmarks
   ```
   together with the mesh (`vertex`, `face`) and the nearest-vertex indices (`electrode_idx`). The app only needs `sensor`.

When `prep_landmark.m` itself saves at the end, it asks before overwriting `verts_electrode_idx` in an existing file. Answer **No** to keep the file as it is. Rerunning `prep_landmark` shows the saved landmarks on the scalp so you can check them.

Choose landmarks you can find reliably with the pointer on the real head, e.g. nasion, nose tip, left/right preauricular points (LPA/RPA), and the outer canthi. Use at least 3, not in a line. Four or more spread around the head is better.

---

## 5. Register the subject (Brain tab)

### Setup

1. Attach the **head tracker** firmly to the subject's head. It must not move relative to the head after registration.
2. Have the **pointer** (landmark tool) ready. Its *hot spot*, the tool origin in its `.rom` file, is the tip for NDI probes.
3. Connect, run **Tool check**, and start streaming with **BX2** + repeat **On** + **Send**.

### Steps

1. **Brain**: load the subject's brain surface (`.pial`, `.white`, `.inflated` or `.tri`) into the Brain tab.
2. **Brain tool dropdown**: choose the head tracker.
3. **Landmark tool dropdown**: choose the pointer (e.g. `P-1735`).
4. **Landmark file**: choose `fhlin_landmark.mat`. The landmarks are listed in the list box and drawn as labelled dots. They start **red** (not sampled).
5. For each landmark:
   1. Highlight it in the list box.
   2. Hold the pointer's tip on that spot of the subject's head, and keep it still.
   3. Click **Landmark sample**. The landmark turns **orange** while about 30 frames (0.5 s) are averaged.
   4. When the sample is accepted, it turns **green** and the next unsampled landmark is highlighted.

   A sample is rejected if the tip moved more than 2 mm during the 0.5 s. It is abandoned if the pointer and head tracker were not both visible within 5 s. Simply sample that landmark again; you can re-sample any landmark at any time.
6. When **all landmarks are green**, the registration (MRI → head tracker) is computed automatically. The Brain tab title shows the RMS error and the error for each landmark, in mm.
7. From now on:
   - the **brain moves with the subject's head** (in anatomical axes);
   - a **magenta dot** shows where the pointer's tip is on the mesh.

   **Check the registration:** touch the nasion or another landmark; the magenta dot should sit on the same landmark of the mesh.

The **Landmark sample** button is enabled only while the chosen landmark tool is tracked, a landmark file is loaded, and no sample is in progress. When it is disabled, the reason is printed in the MATLAB command window. The head tracker is checked when you click.

Messages appear as the **Brain tab title** and in the **MATLAB command window**.

### How the brain moves

| State | Brain display |
|---|---|
| No landmark file loaded | Follows the head tracker relative to its pose when tracking started, rotating about the brain's centre (no registration) |
| Landmark file loaded, not all sampled | Held still |
| All landmarks sampled (registered) | Follows the head tracker in MRI axes; the pointer tip is shown as a magenta dot |

The axes grow when needed so the whole brain stays visible.

### Accuracy

- The camera itself is accurate to about 0.25 mm RMS.
- Landmark registration is usually **2–4 mm**. Landmarks on skin are fuzzy, and the skin moves under the pointer.
- **If the head tracker slips on the head, the registration is invalid.** Reload the landmark file and sample again.
- **Changing the Brain tool after registration also invalidates it.**

---

## Under the hood

- **Poses.** For each tool the camera reports a quaternion and a translation, which give the 4×4 pose `R` (tool → camera, mm).
- **Pointer samples are stored relative to the head tracker.** Head and camera movement during sampling therefore doesn't matter:
  ```
  hot_spot_head = inv(R_head) * R_pointer * [tip; 1]
  ```
- **Registration.** A rigid least-squares fit (Horn/SVD) of `sensor.coords` (MRI) to the sampled points gives `T` (MRI → head-tracker frame).
- **Brain display after registration.** `R0` is the head pose when following starts.
  ```
  inv(T) * inv(R0) * R_head * T
  ```
- **Pointer dot.**
  ```
  inv(T) * inv(R0) * R_pointer * [tip; 1]
  ```
- **Custom pointer.** If the pointer's hot spot is not its tool origin, set the offset (mm, in the pointer's own frame) before sampling:
  ```matlab
  app.nav_camera.landmark_tip = [x; y; z];
  ```

## Known limitations

- **Tool numbering.** Tools are numbered by their order in the camera's frame, but the dropdowns use the order from `PHSR 00`. These match as long as every allocated port handle is enabled. If a tool shows "has not reported any pose", re-check the tools in NDI Track and run Tool check again.
- **The registration is not saved to a file yet.** Re-sample after restarting the app.
- **`tcpip` is deprecated.** MATLAB recommends `tcpclient`; migrating is a planned change.
