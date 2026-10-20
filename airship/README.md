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

Attach a CC:Tweaked monitor (a 3-wide by 2-tall monitor works) to any computer side. The computer screen stays the control and text-entry console; the first attached monitor shows live flight data and touch controls. Enter all coordinates, altitude targets, and PID values on the computer itself. With no monitor attached, the computer screen shows the full interface.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## Using the UI

1. Type **G**, enter destination `X Z` (for example `200 -700`), and press Enter. Type **Y**, enter target altitude, and press Enter. Up/Down changes the altitude target by one block; **H** captures current height, **A** engages autopilot, and **S** cuts outputs.
2. In a clear area, test the four low-power vector buttons or **Left/Right/F/B** keys. The ship front is configured as **east**; X+ should move south and Y+ should move east. Press **H** to stop each test. After all four directions are confirmed, click **CALIBRATE** twice within eight seconds or press **C** twice.
3. **T** focuses the PID tuning field. Enter seven space-separated values in this order: `horizontal-P horizontal-I horizontal-D altitude-bias altitude-P altitude-I altitude-D`, then press Enter. The app validates and saves them in `airship-pid.cfg`.
4. Set a destination and height, then press **A**. The horizontal PID steers toward the X/Z destination and uses GPS movement to brake. Thruster throttle now varies smoothly with the PID command instead of switching between zero and a fixed level, capped at 35% for slower travel. It stops applying horizontal thrust once position and speed are both close to the target. The altitude PID adjusts burner redstone strength from altitude error and vertical speed, using the Avionics sensor when connected and GPS-derived altitude/speed otherwise.

## Initial PID values and tuning

Starting values: horizontal `0.035 0.0005 0.18`; altitude `bias 4, P 1.0, I 0.03, D 2.0`. These are conservative first-flight values, not empirically tuned for your ship. Valid ranges are horizontal P `0.005–0.15`, I `0–0.01`, D `0–0.8`; altitude bias `0–12`, P `0.1–3`, I `0–0.2`, D `0–5`.

Test in a backup/creative world with short moves. The 35% throttle cap is a cautious starting point, not a guarantee against overshoot: inertia, GPS noise, and update delay still matter, and a very low throttle may not overcome drag. If the ship overshoots horizontally, lower horizontal P a little or raise D; if it approaches too slowly, raise P slightly. For altitude, if it keeps rising at the target, lower bias; if it sinks, raise bias. Reduce altitude P or raise D if it bobs past the target. Make small changes and use **T** to save them.

The UI refreshes on a 20 Hz timer. The altitude PID reads the Avionics sensor every control-loop pass; sensor speed is sampled at server tick rate and the sensor documentation notes a one-tick lag. GPS calls use a 0.1-second timeout, horizontal navigation updates on fresh GPS fixes, and outputs are cut after 0.5 seconds without GPS. Basalt 2.5 requires HTTP access if you download it with `wget`; the app and `basalt.lua` must be in the same folder. GPS is still required for X/Z autopilot navigation. If GPS is lost during autopilot, outputs are cut and you must re-engage after the fix returns. The controller does not check obstacles or handle landing. The exact PID response depends on the ship, burner/link behavior, and sensor/GPS quality; the defaults have not been flight-tested.
