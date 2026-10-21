# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. It uses GPS for X/Z navigation, prefers Create: Avionics altitude-sensor readings for altitude hold, and controls burner power through the adjacent Create Redstone Link.

## Install

In the computer terminal, install Basalt 2.5, download the updated controller beside it, and run it:

```text
wget run https://basalt.madefor.cc/2.5/install.lua minified
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight peripherals named `liquid_vector_thruster_0` through `_7`. For improved altitude hold, add one Create: Avionics **Altitude Sensor** to the ship and connect it to the same CC peripheral network. Its peripheral type is `altitude_sensor`; the app detects it automatically and falls back to GPS altitude if it is absent.

To check that CC:Tweaked sees it, run `lua` and enter:

```lua
for _, name in ipairs(peripheral.getNames()) do print(name, peripheral.getType(name)) end
```

Look for a row whose type is `altitude_sensor` (the device name may have a numeric suffix). The app uses its `getHeight()` and `getVerticalSpeed()` methods.

Attach a CC:Tweaked monitor (a 3-wide by 2-tall monitor works) to the ship's peripheral network. The computer screen stays the control and text-entry console. If a DirectGPU peripheral is also visible to the computer, the monitor shows the high-resolution RGB flight panel with touch controls; otherwise it uses the standard Basalt monitor UI. The DirectGPU block must also be physically near the monitor wall so it can detect that display; a wired modem connection alone exposes the peripheral but does not move its monitor scan. Enter coordinates, altitude targets, and PID values on the computer itself. With no monitor attached, the computer screen shows the full interface. Check for DirectGPU with `peripheral.find("directgpu")`; connect its block to the computer's peripheral network if this returns `nil`.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## Using the UI

1. Type **G**, enter destination `X Z` (for example `200 -700`), and press Enter. Type **Y**, enter target altitude, and press Enter. Up/Down changes the altitude target by one block; **H** captures current height, **A** engages autopilot, and **S** cuts outputs.
2. In a clear area, test the four low-power vector buttons or **Left/Right/F/B** keys. The ship front is configured as **east**; X+ should move south and Y+ should move east. Press **H** to stop each test. After all four directions are confirmed, click **CALIBRATE** twice within eight seconds or press **C** twice.
3. **T** focuses the PID tuning field. Enter seven space-separated values in this order: `horizontal-P horizontal-I horizontal-D altitude-bias altitude-P altitude-I altitude-D`, then press Enter. The app validates and saves them in `airship-pid.cfg`.
4. Set a destination and height, then press **A**. The horizontal PID steers toward the X/Z destination and uses GPS movement to brake. Thruster throttle now varies smoothly with the PID command instead of switching between zero and a fixed level, capped at 35% for slower travel. It stops applying horizontal thrust once position and speed are both close to the target. The altitude PID adjusts burner redstone strength from altitude error and vertical speed, using the Avionics sensor when connected and GPS-derived altitude/speed otherwise.

## Initial PID values and tuning

Starting values: horizontal `0.035 0.0005 0.18`; altitude `bias 4, P 0.45, I 0.005, D 2.5`. These are conservative starting values, not empirically tuned for your ship. Valid ranges are horizontal P `0.005–0.15`, I `0–0.01`, D `0–0.8`; altitude bias `0–12`, P `0.1–3`, I `0–0.2`, D `0–5`.

Test in a backup/creative world with short moves. Burner output now ramps at no more than 6 redstone levels per second and uses all analog strengths 0–15 instead of jumping immediately to the PID result. If altitude overshoots, lower altitude P or raise D; if it keeps rising at the target, lower bias; if it sinks, raise bias. Make small changes and use **T** to save them. The right bias depends on how much burner signal is needed to hold this ship up.

## Manual burner calibration

Press **M** or **MANUAL M** to hold a fixed burner signal. **Up/Down** changes the signal by one from 0–15; the screen shows the current burner signal and average normalized thrust. To automatically record levels **15 down to 5**, put the ship in clear air, choose a safe starting height, then press **V** twice within 8 seconds (or tap **SWEEP** twice on the DirectGPU monitor). The app ramps toward each signal at no more than six redstone levels per second, then saves its signal, height, and pressure after vertical speed stays below 0.12 blocks/sec for four seconds. It skips a level if it cannot settle within 45 seconds. The sweep safety-stops after moving more than 24 blocks from its starting height or reaching 3 blocks/sec vertical speed; it also stops if GPS/pressure is lost or saving fails. The last stop reason and saved/skipped counts stay visible for 15 seconds, and previously saved levels remain. After the sweep it resumes altitude hold at the current height. **K** or **LOCK LEVEL** still saves one manually stabilized signal. The controller interpolates saved burner levels by live pressure during automatic altitude hold.

Press **M** again to leave manual mode; automatic altitude hold resumes at the current height. **P** or **ALT PRESET** cycles targets 10 blocks below, then 10, 25, and 50 blocks above the reference captured by the first preset selection. **HOLD HERE** resets that reference. The altitude PID still corrects small errors automatically. The sweep covers signals 15 through 5; it does not test 0–4.

The UI refreshes on a 20 Hz timer. The altitude PID reads the Avionics sensor every control-loop pass; sensor speed is sampled at server tick rate and the sensor documentation notes a one-tick lag. GPS calls use a 0.1-second timeout, horizontal navigation updates on fresh GPS fixes, and outputs are cut after 0.5 seconds without GPS. Basalt 2.5 requires HTTP access if you download it with `wget`; the app and `basalt.lua` must be in the same folder. GPS is still required for X/Z autopilot navigation. If GPS is lost during autopilot, outputs are cut and you must re-engage after the fix returns. The controller does not check obstacles or handle landing. The exact PID response depends on the ship, burner/link behavior, and sensor/GPS quality; the defaults have not been flight-tested.

## Collect altitude tuning data

In a safe test area, stop the controller with **Ctrl+T**, then install and start the diagnostic:

```text
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/diagnostic.lua diagnostic.lua
diagnostic.lua
```

It asks for ship/balloon capacity, burner count and max fill, and ship mass/load. Then it automatically logs time, altitude, vertical speed, air pressure, and burner strength five times per second while stepping the signal around the saved altitude bias (or 4 if no PID file exists): baseline, +1, baseline, -1, baseline, 20 seconds per step. It aborts if altitude changes by 5 blocks or vertical speed reaches 1.2 blocks/sec. Press **Q** at any time to abort. It saves a new CSV without overwriting older logs and automatically sends it to a **public** Pastebin paste using CC:Tweaked's built-in `pastebin put` program. Type **YES** at startup to authorize the test and upload. HTTP access must be enabled.

The prompts default to the facts provided so far: 15x15 footprint, 845 blocks of balloon capacity (please verify), four burners, and 210 max fill each. It calculates combined usable fill as the smaller of balloon capacity and burner capacity (840 with those defaults). Correct any value that differs on the ship. Run only in a clear test area; the safety cutoff and **Q** cut burner input, but existing hot air does not disappear instantly.
