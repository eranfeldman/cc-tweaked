# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. It reads X/Y/Z from GPS, holds a selected altitude with the adjacent Create Redstone Link, and travels to an X/Z destination.

## Install

In the computer terminal, install Basalt 2.5, download the updated controller beside it, and run it:

```text
wget run https://basalt.madefor.cc/2.5/install.lua minified
wget -f https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight peripherals named `liquid_vector_thruster_0` through `_7`.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## Using the UI

1. Use the colored dashboard buttons or keyboard: **G** focuses the destination field, type `X Z` and press Enter; **Up/Down** adjusts altitude; **H** holds here; **A** starts autopilot; **S** stops outputs.
2. In a clear area, test the four low-power vector buttons or **Left/Right/F/B** keys. The ship front is configured as **east**; X+ should move south and Y+ should move east. Press **H** to stop each test.
3. After confirming all four directions, click **CALIBRATE** twice within eight seconds or press **C** twice. The saved calibration lets **A** engage autopilot.
4. Enter numeric X and Z coordinates and press Enter or click **SET DEST**, then engage autopilot. It cuts thrust within two blocks; momentum may carry the ship beyond the destination.

Basalt 2.5 requires HTTP access if you download it with `wget`; the app and `basalt.lua` must be in the same folder. The interface uses the computer's terminal; your peripheral list has no monitor. GPS is required for altitude holding and autopilot. If GPS is lost in flight, outputs are cut and you must re-engage after the fix returns. The controller does not check obstacles or handle landing. Test in a backup world first: it has not been tested against your live ship physics, fuel use or Create link range.
