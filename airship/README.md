# Airship Control

CC:Tweaked controller for eight Create Propulsion liquid vector thrusters. It reads X/Y/Z from GPS, holds a selected altitude with the adjacent Create Redstone Link, and travels to an X/Z destination.

## Install

On the ship computer, run:

```text
wget https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua airship.lua
airship.lua
```

Or copy `airship.lua` into a file on the computer and run `airship.lua`. The computer needs wireless GPS access and all eight peripherals named `liquid_vector_thruster_0` through `_7`.

Put a Create Redstone Link transmitter against the computer's **front** (monitor side). Put `create:analog_lever` in both frequency slots on this transmitter and the burner receiver, in the same order. The app sends analog power 0–15 out the computer's front face to control the burners.

## First flight

1. Use a clear area and make sure the ship has fuel. The app takes over each connected thruster's throttle.
2. Press **G** and enter destination `X Z` (example: `200 -700`).
3. Check GPS and altitude. Up/Down changes the altitude target one block at a time. **H** holds the current altitude; **X** stops thrusters and burner signal.
4. The ship front is configured as **east**. Autopilot assumes each thruster's local vector X moves ship-right (south) and Y moves ship-forward (east). Before enabling autopilot, make short low-thrust checks in an open area: **Right** tests X+, **Left** tests X−, **F** tests Y+, and **B** tests Y−. Press **H** to stop the test. Press **C** and type `YES` only after X+ moves south and Y+ moves east. The confirmation is saved on the computer.
5. Press **A** to travel. It cuts thrust within two blocks of the destination and keeps the altitude target. Momentum may carry the ship beyond that point. Up/Down changes the altitude target; left/right/F/B are low-power vector checks. Press **H** to return to altitude hold or **X** to stop outputs.

The interface expects a normal ComputerCraft terminal: the provided peripheral list has no monitor peripheral. A GPS constellation is required for autopilot and altitude holding; if GPS is lost during autopilot, outputs are cut and the pilot must re-engage. The controller does not check obstacles or handle landing. Test in a backup world first: it has not been tested against your live ship physics, fuel use or Create link range.
