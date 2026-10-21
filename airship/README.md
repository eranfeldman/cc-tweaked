# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. It uses GPS for X/Z navigation, prefers Create: Avionics altitude-sensor readings for altitude hold, and controls burner power through the adjacent Create Redstone Link.

## Install

In the computer terminal, install Basalt 2.5, download the updated controller beside it, and run it:

```text
wget run https://basalt.madefor.cc/2.5/install.lua minified
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua?bootstrap=1.1.1 airship-updater.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/pid-tuner.lua pid-tuner.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight peripherals named `liquid_vector_thruster_0` through `_7`. For accurate altitude readout and burner calibration, connect a Create: Avionics **Altitude Sensor** to the computer through the peripheral network. Height control still comes from burner amounts. Its peripheral type is `altitude_sensor`; the app detects it automatically and falls back to GPS altitude if it is absent.

Install `airship-updater.lua` once beside the controller. Then press **UPDATE** (or **U** on the computer) to stop the controller, open the updater in a second Multishell tab, download and validate the latest controller, replace the installed copy, and launch it. The download bypasses cached copies, checks the build number, and the controller title shows its running version. The updater keeps the previous version as `airship.lua.bak`; calibration and PID files are left in place. The button requires CC:Tweaked Multishell. If an old updater does not install the latest build, bootstrap the new updater with `wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua?bootstrap=1.1.1 airship-updater-new.lua`, then run `airship-updater-new.lua`. To tune horizontal autopilot gains, install `pid-tuner.lua` beside the controller and run it after `diagnostic.lua`; it measures X/Z overshoot and suggests small P/D changes for your confirmation. Height is controlled only by burner amounts from the saved calibration map.

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
2. In a clear area, press **CALIBRATE** once, or **RECALIBRATE** if a calibration is already saved. It automatically pulses X−, X+, Y+, and Y− at low power, with short pauses, then saves calibration. Press **HOLD** to cancel. Watch the ship during the sequence: it sends the four commands but does not verify the resulting movement. The ship front is configured as **east**; X+ should move south and Y+ should move east. The individual **Left/Right/F/B** keys and forward/back buttons remain available for manual checks.
3. **T** focuses the PID tuning field. Enter three space-separated values: horizontal `P I D`, then press Enter. These gains affect X/Z steering only.
4. Choose a saved calibrated height and destination, then press **A**. The horizontal PID steers toward the X/Z destination and uses GPS movement to brake. Thruster throttle is capped at 35% for slower travel and stops when position and speed are close to the target. Height selection sends the burner amount recorded for that height; there is no altitude PID.

## Initial PID values and tuning

Starting horizontal values: `0.035 0.0005 0.18`. These are starting values, not empirically tuned for your ship. The calibration field and `airship-pid.cfg` use only horizontal P `0.005–0.15`, I `0–0.01`, D `0–0.8`. Older seven-number config files remain readable using their first three values.

Test in a clear area with short moves. Burner output ramps at no more than 6 redstone levels per second. For horizontal overshoot, run `pid-tuner.lua` after the diagnostic and confirm or reject its suggested P/D change. Height tuning is done by recording stable burner amounts, not PID gains.

## Manual burner calibration

Press **M** or **MANUAL M** to hold a fixed burner signal. **Up/Down** changes the signal by one from 0–15; the screen shows the current burner signal and average normalized thrust. To automatically record levels **15 down to 5**, put the ship in clear air, choose a safe starting height, then press **V** twice within 8 seconds (or tap **SWEEP** twice on the DirectGPU monitor). The app ramps toward each signal at no more than six redstone levels per second, then saves its signal and altitude after vertical speed stays below 0.12 blocks/sec for four seconds. It skips a level if it cannot settle within 45 seconds. The sweep no longer stops based on vertical speed or distance traveled; it still stops if GPS/pressure is lost or saving fails. The last stop reason and saved/skipped counts stay visible for 15 seconds, and previously saved levels remain. After the sweep it resumes altitude hold at the current height. **K** or **LOCK LEVEL** still saves one manually stabilized signal. Saved altitude-to-signal points are interpolated; when calibration data exists, altitude hold uses the mapped burner signal directly without PID trimming. The burner output still slews toward that signal.

Press **M** again to leave manual mode; burner output follows the selected saved height. **P** or **SAVED ALT** cycles through calibrated altitudes, sorted low to high. If no calibration points are saved, automatic height selection has no mapping; use manual burner control until you calibrate. The sweep covers signals 15 through 5; it does not test 0–4.

The UI refreshes on a 20 Hz timer. Avionics height, pressure, heading, and gimbal yaw rate are used for display and burner calibration; height does not feed a PID. GPS calls use a 0.1-second timeout, horizontal navigation updates on fresh GPS fixes, and outputs are cut after 0.5 seconds without GPS. Basalt 2.5 requires HTTP access if you download it with `wget`; the app and `basalt.lua` must be in the same folder. GPS and navigation-table heading are required for X/Z autopilot navigation. If GPS is lost during autopilot, outputs are cut and you must re-engage after the fix returns. The controller does not check obstacles or handle landing. Horizontal PID defaults are not yet flight-tested.

## Collect altitude tuning data

In a large, obstacle-free area, stop the controller with **Ctrl+T**, then install and start the diagnostic:

```text
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/diagnostic.lua diagnostic.lua
diagnostic.lua
```

After a diagnostic run containing all four X/Z autopilot legs, stop the controller and run `pid-tuner.lua`. It reviews the latest CSV and suggests modest horizontal P/D changes; confirm to save them, then restart the controller and repeat the test. It does not drive the ship itself.

The ship front must be the edge with thrusters 0 and 6, facing east. It asks for ship size, balloon capacity, burner count and fill, and mass/load. Once you finish entering the data, it discovers every `liquid_vector_thruster` peripheral by type and name, then individually pulses each one on both vector axes. The computer reports each device name with its measured relative X/Z movement and includes those readings in the CSV. This checks the actual response for names such as `liquid_vector_thruster_0`; it does not assume those suffixes match your drawing. After the survey and altitude steps, it resets its relative GPS origin to the ship's current position, then automatically runs four autopilot moves to points 10 blocks away (X+, X−, Z+, Z−), allowing up to 60 seconds for each to settle. The diagnostic and flight controller now reverse the PID command to account for the fact that the thruster nozzle vector points opposite the resulting craft thrust. The CSV records horizontal PID gains, burner signal, thruster commands and readings, altitude, pressure, heading, yaw rate, relative GPS position, and velocity. Absolute GPS coordinates are not recorded. It reads the same saved PID and height-map files as the controller; height control uses saved burner amounts only; there is no altitude PID. Speed, altitude-change, and distance cutoffs have been removed. GPS loss still ends the test because navigation data would no longer be valid; **Q** also aborts. On exit it turns off thrusters and restores the burner signal that was active before the test. It saves a new CSV without overwriting older logs, then automatically uploads it using CC:Tweaked's built-in `pastebin put` program to a **public** Pastebin paste. HTTP access must be enabled.

The prompts default to the facts provided so far: 15x15 footprint, 845 blocks of balloon capacity (please verify), four burners, and 210 max fill each. It calculates combined usable fill as the smaller of balloon capacity and burner capacity (840 with those defaults). Correct any value that differs on the ship. Run only where the ship can move freely and stay at the controls; there is no speed, altitude, or distance cutoff.
