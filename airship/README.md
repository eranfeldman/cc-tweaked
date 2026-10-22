# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. GPS, Avionics heading, yaw rate, and air pressure drive the measured horizontal autopilot. Height uses saved burner amounts through the adjacent Create Redstone Link; there is no altitude PID.

## Install

Controller v1.3.1 includes FULL AUTO CAL with a smoothed, bounded settling check to tolerate brief GPS and physics wobble. The existing updater installs it without extra dependencies. GPU rendering retains cooperative yields, a 20 Hz refresh target, and native monitor/GPU click support.

In the computer terminal, install Basalt 2.5, download the updated controller beside it, and run it:

```text
wget run https://basalt.madefor.cc/2.5/install.lua minified
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua?bootstrap=1.3.1 airship-updater.lua
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/pid-tuner.lua pid-tuner.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight `liquid_vector_thruster` peripherals. They can be connected directly or exposed by the wired peripheral hub; their peripheral names and number suffixes can vary. The controller finds them by peripheral type and stops with a clear count error unless it sees all eight. For accurate altitude readout and burner calibration, connect a Create: Avionics **Altitude Sensor** to the computer through the peripheral network. Height control still comes from burner amounts. Its peripheral type is `altitude_sensor`; the app detects it automatically and falls back to GPS altitude if it is absent.

Install `airship-updater.lua` beside the controller. It validates and installs the controller, both diagnostics, the PID tuner, and itself from one GitHub commit. It keeps each previous copy as a `.bak`, leaves calibration and PID files in place, then launches the controller. To bootstrap the rollback updater from an older install, run `wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua?bootstrap=1.3.1 airship-updater-new.lua` then `airship-updater-new.lua`. The temporary bootstrap file can be deleted afterward. Height is controlled only by burner amounts from the saved calibration map.

To check that CC:Tweaked sees it, run `lua` and enter:

```lua
for _, name in ipairs(peripheral.getNames()) do print(name, peripheral.getType(name)) end
```

The app searches both directly attached peripherals and peripherals one Malahit hub deep behind the wired peripheral hub. It reads `altitude_sensor` (`getHeight`, `getVerticalSpeed`, `getAirPressure`), `navigation_table` (`getHeadingRad`) and `gimbal_sensor` (`getAngularRatesRad`) through either route. GPS still supplies world position. The navigation table heading rotates GPS error and velocity into the ship's current axes, so turning while navigating does not leave the controller steering in the old direction. If heading is unavailable, destination autopilot refuses to engage.

Attach a CC:Tweaked monitor to the ship's peripheral network. The computer screen stays the control and text-entry console. When DirectGPU is available, including over a wired modem as `directgpu_5`, the monitor shows the high-resolution RGB flight panel with touch controls; its layout scales to the detected display size, including a 4-wide by 3-tall monitor wall. Click the **FLIGHT**, **CALIBRATION**, and **HEIGHTS** tabs above the panel to switch views. The HEIGHTS view lists every saved calibration point with its altitude, burner level, and pressure; use **PREVIOUS/NEXT** to browse longer lists and tap a row to select that height. The DirectGPU block must be physically near the monitor wall so it can detect that display; a wired modem connection exposes the peripheral but does not move its monitor scan. Enter coordinates, altitude targets, and PID values on the computer itself. With no monitor attached, the computer screen shows the full interface. Check for DirectGPU with `peripheral.find("directgpu")`.

On the full-size computer UI, use **FLIGHT** for navigation and **CALIBRATION** for PID values, direction checks, manual burner control, locking a stable burner level, and the 15–5 sweep. **ALT - / ALT +** moves to the previous or next saved height (wrapping at either end); if no heights are saved, it changes the target by one block. **SAVED ALT** selects the next saved height. The calibration screen's **CLEAR HEIGHTS** button clears saved height data from every view after a second tap within eight seconds. The standard monitor has touch controls for flight and calibration; the DirectGPU panel has separate flight, calibration, and saved-height views, with touch controls for all the main actions. It also shows vertical speed, sensor status, and the current target burner level when calibrated. While a text box is focused, keyboard shortcuts are paused; **Esc** clears focus, and **Enter** submits the current field.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## Full autopilot calibration

Connect all eight `liquid_vector_thruster` devices, working wireless GPS, an Avionics `altitude_sensor`, `navigation_table`, and `gimbal_sensor`. The sensor and thrusters may be exposed through the supported wired/hub routes. Calibration measures the current ship and load; weight and thruster number positions do not need to be entered.

1. Get the ship airborne in an open area. Use a saved height or manual burner level to let it settle at your normal cruising height. Small rocking and an off-center load are supported; perfect balance is not required. Keep enough fuel for at least 20 minutes of automatic movement, allowing for retries.
2. Open **CALIBRATION** and press **FULL AUTO CAL**, or press **R** on the computer with no text box focused. It starts even if an old calibration already exists. No sweep is started. The current burner level is held throughout the procedure.
3. Let it finish: 32 individual thruster/nozzle tests measure movement and turning over six-second pulses, rather than two individual readings. Four-second coast windows separate steady drift from thrust and estimate drag. Five automatic moves test X+, X-, Z+, Z-, then return. It chooses P/I/D gains from measured response, sample timing, and filtering delay. Excessive overshoot or a failed arrival triggers retuning and a complete repeat of the direction tests, with up to three validation passes. Each leg gets time for its travel distance and measured settling response. The simulated test ships took about 13–15 minutes; actual time depends on the ship and server. Watch the displayed stage, remaining position error, and net drift. **CANCEL AUTO CAL**, **HOLD**, or **H** cancels; **STOP/S** also cuts burner output.
4. After **FULL AUTO CAL saved**, select a saved altitude, enter destination `X Z` on the computer, and press **AUTO/A**. The controller brakes using measured response, counters yaw through individual thruster commands, and keeps correcting position after arrival. The calibration finish leaves the burner at its starting level until you choose another height or flight mode.

The measured model, tuned gains, pressure reference, and verification results are saved in `airship-autopilot.cfg` and loaded after restart. A failed or canceled run keeps the last working profile. Changed peripheral names invalidate the profile. Recalibrate after changing mass/cargo, fuel type, thruster placement, or propulsion settings. Autopilot learns a running correction for persistent horizontal drift and turning bias, estimates speed from a short position trend, and uses gimbal gravity to separate heading rate from tilt. Validation measures net drift over several seconds and bounds individual position excursions; a brief rocking velocity spike does not reset the test. Calibration models horizontal movement and yaw, not active roll/pitch leveling: large tilts or insufficient counter-thrust still require a physical change to the ship. The optional **DIR CHECK/C** button only sends the old four wiring-check pulses and does not replace FULL AUTO CAL.

### Air pressure

Calibration measures drag at its current pressure. Autopilot scales that learned drag estimate with the live pressure reading and actively brakes instead of relying on coasting to stop. Both checked pack configs have `useAtmosphericPressure = false`, so this controller does not apply an invented inverse-pressure thrust boost. If the server changes that setting, or you fly under substantially different conditions, recalibrate at the intended cruise height. Pressure compensation is a learned approximation, not a guarantee across every altitude.

### Controls and manual gains

**G** focuses destination entry, **Y** focuses height entry, **P** cycles saved heights, **A** engages autopilot, **H** holds/cancels calibration, and **S** cuts outputs. **Esc** clears text focus. Height remains controlled by the burner map; FULL AUTO CAL does not generate or change that map. Run the separate sweep only when you need new saved heights.

FULL AUTO CAL chooses P, I, and D automatically. Manual **SET PID** overrides are saved into the active profile; use the displayed measured gains as the starting point. Accepted ranges are P `0.0001–1`, I `0–0.05`, D `0–4`. Re-running full calibration replaces manual overrides with newly measured and tested gains. The older `airship-pid.cfg` applies only before a measured profile exists; the standalone PID tuner is for the legacy controller and directs measured-profile users back to FULL AUTO CAL.

## Manual burner calibration

Press **M** or **MANUAL M** to hold a fixed burner signal. **Up/Down** changes the signal by one from 0–15; the screen shows the current burner signal and average normalized thrust. To automatically record levels **15 down to 5**, put the ship in clear air, choose a safe starting height, then press **V** twice within 8 seconds (or tap **SWEEP** twice on the DirectGPU monitor). The app ramps toward each signal at no more than six redstone levels per second, then saves its signal and altitude after vertical speed stays below 0.12 blocks/sec for four seconds. It skips a level if it cannot settle within 45 seconds. The sweep no longer stops based on vertical speed or distance traveled; it still stops if GPS/pressure is lost or saving fails. The last stop reason and saved/skipped counts stay visible for 15 seconds, and previously saved levels remain. After the sweep it resumes altitude hold at the current height. **K** or **LOCK LEVEL** still saves one manually stabilized signal. Saved altitude-to-signal points are interpolated; when calibration data exists, altitude hold uses the mapped burner signal directly without PID trimming. The burner output still slews toward that signal.

Press **M** again to leave manual mode; burner output follows the selected saved height. **P** or **SAVED ALT** cycles through calibrated altitudes, sorted low to high. If no calibration points are saved, automatic height selection has no mapping; use manual burner control until you calibrate. The sweep covers signals 15 through 5; it does not test 0–4.

The local UI targets 20 Hz. Avionics and wired control run in a separate cooperative loop with a 0.2-second delay between passes; actual sample timing includes GPS and peripheral latency. GPS uses a 0.5-second timeout and loss for over two seconds stops navigation. Autopilot also requires fresh heading, yaw, and pressure readings and must be re-engaged after navigation data is lost. The controller does not detect obstacles or land the ship. Successful simulated checks do not replace testing the actual craft.

## Legacy diagnostic script

FULL AUTO CAL above is the current calibration and tuning path. The standalone diagnostic below runs the older uniform-thruster steering model, not the measured mixer, so its results do not validate the new autopilot. Keep it for inspecting legacy behavior and producing logs.

In a large, obstacle-free area, stop the controller with **Ctrl+T**, then install and start the diagnostic:

```text
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/diagnostic.lua diagnostic.lua
diagnostic.lua
```

After a diagnostic run containing all four X/Z autopilot legs, stop the controller and run `pid-tuner.lua`. It reviews the latest CSV and suggests modest horizontal P/D changes; confirm to save them, then restart the controller and repeat the test. It does not drive the ship itself.

The ship front faces east. It asks for ship size, balloon capacity, burner count and fill, and mass/load. Once you finish entering the data, it discovers every `liquid_vector_thruster` by type, including devices exposed through the peripheral hub, then individually pulses each one on both vector axes. The computer reports each full hub path/name with its measured relative X/Z movement and includes those readings in the CSV; it does not assume peripheral number suffixes match the thruster positions on your ship. After the survey and altitude steps, it resets its relative GPS origin to the ship's current position, then automatically runs four autopilot moves to points 10 blocks away (X+, X−, Z+, Z−), allowing up to 60 seconds for each to settle. The diagnostic and flight controller now reverse the PID command to account for the fact that the thruster nozzle vector points opposite the resulting craft thrust. The CSV records horizontal PID gains, burner signal, thruster commands and readings, altitude, pressure, heading, yaw rate, relative GPS position, and velocity. Absolute GPS coordinates are not recorded. It reads the same saved PID and height-map files as the controller; height control uses saved burner amounts only; there is no altitude PID. Speed, altitude-change, and distance cutoffs have been removed. GPS loss still ends the test because navigation data would no longer be valid; **Q** also aborts. On exit it turns off thrusters and restores the burner signal that was active before the test. It saves a new CSV without overwriting older logs, then automatically uploads it using CC:Tweaked's built-in `pastebin put` program to a **public** Pastebin paste. HTTP access must be enabled.

The prompts default to the facts provided so far: 15x15 footprint, 845 blocks of balloon capacity (please verify), four burners, and 210 max fill each. It calculates combined usable fill as the smaller of balloon capacity and burner capacity (840 with those defaults). Correct any value that differs on the ship. Run only where the ship can move freely and stay at the controls; there is no speed, altitude, or distance cutoff.
