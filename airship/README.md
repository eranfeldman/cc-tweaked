# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. It uses GPS for X/Z navigation, prefers Create: Avionics altitude-sensor readings for altitude hold, and controls burner power through the adjacent Create Redstone Link.

## Install

In the computer terminal, install Basalt 2.5, download the updated controller beside it, and run it:

```text
wget run https://basalt.madefor.cc/2.5/install.lua minified
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua airship-updater.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/pid-tuner.lua pid-tuner.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight peripherals named `liquid_vector_thruster_0` through `_7`. For improved altitude hold, add one Create: Avionics **Altitude Sensor** to the ship and connect it to the same CC peripheral network. Its peripheral type is `altitude_sensor`; the app detects it automatically and falls back to GPS altitude if it is absent.

Install `airship-updater.lua` once beside the controller. Then press **UPDATE** (or **U** on the computer) to stop the controller, open the updater in a second Multishell tab, download and validate the latest controller, replace the installed copy, and launch it. The updater keeps the previous version as `airship.lua.bak`; calibration and PID files are left in place. The button requires CC:Tweaked Multishell. To tune altitude gains from the last altitude-step diagnostic, install `pid-tuner.lua` beside the controller and run `pid-tuner.lua`; it analyzes the step overshoot, suggests conservative P/D changes, and only saves after you confirm. It cannot tune from a log recorded while the saved burner map bypassed altitude PID.

To check that CC:Tweaked sees it, run `lua` and enter:

```lua
for _, name in ipairs(peripheral.getNames()) do print(name, peripheral.getType(name)) end
```

The app searches both directly attached peripherals and peripherals one Malahit hub deep behind the wired peripheral hub. It reads `altitude_sensor` (`getHeight`, `getVerticalSpeed`, `getAirPressure`), `navigation_table` (`getHeadingRad`) and `gimbal_sensor` (`getAngularRatesRad`); DirectGPU can also be discovered through that nested hub. GPS still supplies world position. The navigation table heading rotates GPS error and velocity into the ship's current axes, so turning while navigating does not leave the controller steering in the old direction. If heading is unavailable, destination autopilot refuses to engage.

Attach a CC:Tweaked monitor to the ship's peripheral network. The computer screen stays the control and text-entry console. If a DirectGPU peripheral is also visible to the computer, the monitor shows the high-resolution RGB flight panel with touch controls; its layout scales to the detected display size, including a 4-wide by 3-tall monitor wall. Click the **FLIGHT**, **CALIBRATION**, and **HEIGHTS** tabs above the panel to switch views. The HEIGHTS view lists every saved calibration point with its altitude, burner level, and pressure; use **PREVIOUS/NEXT** to browse longer lists and tap a row to select that height. Otherwise the monitor uses the standard Basalt UI. The DirectGPU block must also be physically near the monitor wall so it can detect that display; a wired modem connection alone exposes the peripheral but does not move its monitor scan. Enter coordinates, altitude targets, and PID values on the computer itself. With no monitor attached, the computer screen shows the full interface. Check for DirectGPU with `peripheral.find("directgpu")`; connect its block to the computer's peripheral network if this returns `nil`.

On the full-size computer UI, use **FLIGHT** for navigation and **CALIBRATION** for PID values, direction checks, manual burner control, locking a stable burner level, and the 15–5 sweep. **ALT - / ALT +** moves to the previous or next saved height (wrapping at either end); if no heights are saved, it changes the target by one block. **SAVED ALT** selects the next saved height. The calibration screen's **CLEAR HEIGHTS** button clears saved height data from every view after a second tap within eight seconds. The standard monitor has touch controls for flight and calibration; the DirectGPU panel has separate flight, calibration, and saved-height views, with touch controls for all the main actions. It also shows vertical speed, sensor status, and the current target burner level when calibrated. While a text box is focused, keyboard shortcuts are paused; **Esc** clears focus, and **Enter** submits the current field.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## Using the UI

1. Type **G**, enter destination `X Z` (for example `200 -700`), and press Enter. Type **Y**, enter target altitude, and press Enter. Up/Down changes the altitude target by one block; **H** captures current height, **A** engages autopilot, and **S** cuts outputs.
2. In a clear area, test the low-power forward/back buttons and use **Left/Right/F/B** keys for the four directions. The ship front is configured as **east**; X+ should move south and Y+ should move east. Press **H** to stop each test. After all four directions are confirmed, click **CALIBRATE** twice within eight seconds or press **C** twice.
3. **T** focuses the PID tuning field. Enter seven space-separated values in this order: `horizontal-P horizontal-I horizontal-D altitude-bias altitude-P altitude-I altitude-D`, then press Enter. The app validates and saves them in `airship-pid.cfg`.
4. Set a destination and height, then press **A**. The horizontal PID steers toward the X/Z destination and uses GPS movement to brake. Thruster throttle now varies smoothly with the PID command instead of switching between zero and a fixed level, capped at 35% for slower travel. It stops applying horizontal thrust once position and speed are both close to the target. The altitude PID adjusts burner redstone strength from altitude error and vertical speed, using the Avionics sensor when connected and GPS-derived altitude/speed otherwise.

## Initial PID values and tuning

Starting values: horizontal `0.035 0.0005 0.18`; altitude `bias 4, P 0.45, I 0.005, D 2.5`. These are conservative starting values, not empirically tuned for your ship. Valid ranges are horizontal P `0.005–0.15`, I `0–0.01`, D `0–0.8`; altitude bias `0–12`, P `0.1–3`, I `0–0.2`, D `0–5`.

Test in a backup/creative world with short moves. Burner output now ramps at no more than 6 redstone levels per second and uses all analog strengths 0–15 instead of jumping immediately to the PID result. If altitude overshoots, lower altitude P or raise D; if it keeps rising at the target, lower bias; if it sinks, raise bias. Make small changes and use **T** to save them. The right bias depends on how much burner signal is needed to hold this ship up.

## Manual burner calibration

Press **M** or **MANUAL M** to hold a fixed burner signal. **Up/Down** changes the signal by one from 0–15; the screen shows the current burner signal and average normalized thrust. To automatically record levels **15 down to 5**, put the ship in clear air, choose a safe starting height, then press **V** twice within 8 seconds (or tap **SWEEP** twice on the DirectGPU monitor). The app ramps toward each signal at no more than six redstone levels per second, then saves its signal and altitude after vertical speed stays below 0.12 blocks/sec for four seconds. It skips a level if it cannot settle within 45 seconds. The sweep no longer stops based on vertical speed or distance traveled; it still stops if GPS/pressure is lost or saving fails. The last stop reason and saved/skipped counts stay visible for 15 seconds, and previously saved levels remain. After the sweep it resumes altitude hold at the current height. **K** or **LOCK LEVEL** still saves one manually stabilized signal. Saved altitude-to-signal points are interpolated; when calibration data exists, altitude hold uses the mapped burner signal directly without PID trimming. The burner output still slews toward that signal.

Press **M** again to leave manual mode; automatic altitude hold resumes at the current height. **P** or **SAVED ALT** cycles through the altitudes stored by the last calibration, sorted from low to high and wrapping to the lowest. **HOLD HERE** holds the current height. If no calibrated altitudes are saved, the chooser reports that and does not change the target. Without calibration data the altitude PID controls height; after recording altitude-to-signal points, altitude hold uses the burner signal associated with the target altitude. The sweep covers signals 15 through 5; it does not test 0–4.

The UI refreshes on a 20 Hz timer. The altitude PID reads the Avionics sensor every control-loop pass; sensor speed is sampled at server tick rate and the sensor documentation notes a one-tick lag. GPS calls use a 0.1-second timeout, horizontal navigation updates on fresh GPS fixes, and outputs are cut after 0.5 seconds without GPS. Basalt 2.5 requires HTTP access if you download it with `wget`; the app and `basalt.lua` must be in the same folder. GPS is still required for X/Z autopilot navigation. If GPS is lost during autopilot, outputs are cut and you must re-engage after the fix returns. The controller does not check obstacles or handle landing. The exact PID response depends on the ship, burner/link behavior, and sensor/GPS quality; the defaults have not been flight-tested.

## Collect altitude tuning data

In a large, obstacle-free area, stop the controller with **Ctrl+T**, then install and start the diagnostic:

```text
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/diagnostic.lua diagnostic.lua
diagnostic.lua
```

After a diagnostic run containing altitude steps, stop the controller and run `pid-tuner.lua`. It reviews the latest CSV and suggests modest P/D changes; confirm to save them, then restart the controller and repeat the test. It does not drive the ship itself.

The ship front must be the edge with thrusters 0 and 6, facing east. It asks for ship size, balloon capacity, burner count and fill, and mass/load. Once you finish entering the data, it discovers every `liquid_vector_thruster` peripheral by type and name, then individually pulses each one on both vector axes. The computer reports each device name with its measured relative X/Z movement and includes those readings in the CSV. This checks the actual response for names such as `liquid_vector_thruster_0`; it does not assume those suffixes match your drawing. After the survey, it automatically runs the altitude steps and four autopilot moves to fixed points 10 blocks from the starting position (X+, X−, Z+, Z−), allowing up to 60 seconds for each to settle. The diagnostic and flight controller now reverse the PID command to account for the fact that the thruster nozzle vector points opposite the resulting craft thrust. The CSV records PID gains, active altitude-control mode, burner signal, thruster commands and readings, altitude, pressure, relative GPS position, and velocity. Absolute GPS coordinates are not recorded. It reads the same saved PID and height-map files as the controller; when a height map exists, the altitude PID is bypassed and the log says so. Speed, altitude-change, and distance cutoffs have been removed. GPS loss still ends the test because navigation data would no longer be valid; **Q** also aborts. On exit it turns off thrusters and restores the burner signal that was active before the test. It saves a new CSV without overwriting older logs, then automatically uploads it using CC:Tweaked's built-in `pastebin put` program to a **public** Pastebin paste. HTTP access must be enabled.

The prompts default to the facts provided so far: 15x15 footprint, 845 blocks of balloon capacity (please verify), four burners, and 210 max fill each. It calculates combined usable fill as the smaller of balloon capacity and burner capacity (840 with those defaults). Correct any value that differs on the ship. Run only where the ship can move freely and stay at the controls; there is no speed, altitude, or distance cutoff.
