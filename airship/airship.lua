-- CC:Tweaked + Create Propulsion liquid vector thruster flight panel.
-- Ship front is east; local vector X is assumed right (south), Y forward (east).

local ids = {
  "liquid_vector_thruster_0", "liquid_vector_thruster_1",
  "liquid_vector_thruster_2", "liquid_vector_thruster_3",
  "liquid_vector_thruster_4", "liquid_vector_thruster_5",
  "liquid_vector_thruster_6", "liquid_vector_thruster_7"
}
local thrust = {}
for _, name in ipairs(ids) do
  thrust[name] = peripheral.wrap(name)
  if not thrust[name] then error("Missing thruster: " .. name) end
end

local target, altitude, position, mode = nil, nil, nil, "HOLD"
local burner = 8
local vectorX, vectorY, power = 0, 0, 0
local calibrated = fs.exists("airship-calibrated")

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function apply()
  for _, t in pairs(thrust) do
    t.setVector(vectorX, vectorY)
    t.setThrust(power)
  end
  redstone.setAnalogOutput("front", burner)
end
local function stop()
  vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "STOPPED"
  pcall(apply)
end
local function locate()
  local x, y, z = gps.locate(1, false)
  if x then position = {x=x, y=y, z=z} end
  if not x and mode == "AUTOPILOT" then
    position = nil
    vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "GPS LOST"
  end
end
local function draw()
  term.setBackgroundColor(colors.black); term.setTextColor(colors.white)
  term.clear(); term.setCursorPos(1, 1)
  print(" AIRSHIP CONTROL  |  FRONT: EAST")
  print(string.rep("=", term.getSize()))
  if position then
    print(("GPS  X %7.1f   Y %5.1f   Z %7.1f"):format(position.x, position.y, position.z))
  else print("GPS  waiting for a fix (check GPS network)") end
  print(("ALTITUDE  %s     TARGET  %s"):format(position and ("" .. math.floor(position.y + .5)) or "--", altitude or "--"))
  print("MODE  " .. mode .. (calibrated and "  | calibration accepted" or "  | NOT CALIBRATED"))
  if target then print(("DEST  X %.0f   Z %.0f"):format(target.x, target.z)) else print("DEST  not set") end
  print(("THRUST %2d/15   VECTOR X %+.2f Y %+.2f   BURNER %2d/15"):format(power, vectorX, vectorY, burner))
  print(string.rep("-", term.getSize()))
  print("G set destination X Z   H hold here   A autopilot   X stop")
  print("Up/Down altitude +/-1   Left/Right X test   F/B Y test   C accept")
  print("Before C: test at low thrust in a clear area; confirm X=right and Y=forward.")
end
local function setDestination()
  mode = "SET DESTINATION"
  draw(); term.setCursorPos(1, 10); write("Destination X Z: ")
  local x, z = read():match("^%s*(-?[%d%.]+)%s+(-?[%d%.]+)%s*$")
  x, z = tonumber(x), tonumber(z)
  if x and z then target = {x=x, z=z} end
  mode = "HOLD"
end
local function updateFlight()
  if not position then
    vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "GPS LOST"
    return
  end
  if mode ~= "STOPPED" and mode ~= "GPS LOST" then
    local errorY = altitude - position.y
    burner = errorY > 1 and 15 or 0
    if mode == "AUTOPILOT" and target then
      local dx, dz = target.x - position.x, target.z - position.z
      local distance = math.sqrt(dx*dx + dz*dz)
      if distance < 2 then vectorX, vectorY, power, mode = 0, 0, 0, "HOLD"
      else
        vectorX = clamp((dz / distance) * .25, -.25, .25)
        vectorY = clamp((dx / distance) * .25, -.25, .25)
        power = 12
      end
    end
  end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(2, false)
  altitude = startY
  altitude = altitude and math.floor(altitude + .5) or 80
  while true do
    locate(); updateFlight(); apply(); draw()
    local ev, key = os.pullEvent()
    if ev == "key" then
      if key == keys.x then stop()
      elseif key == keys.g then setDestination()
      elseif key == keys.h and position then
        altitude = math.floor(position.y + .5); target = nil
        vectorX, vectorY, power, mode = 0, 0, 0, "HOLD"
      elseif key == keys.a then
        if calibrated and target and position then mode = "AUTOPILOT" else mode = "CALIBRATION REQUIRED" end
      elseif key == keys.c then
        print("Only accept after low-thrust checks confirmed X=right, Y=forward.")
        write("Type YES to accept: ")
        if read() == "YES" then fs.open("airship-calibrated", "w").close(); calibrated = true end
      elseif key == keys.up then altitude = altitude + 1; vectorX, vectorY, power, mode = 0, 0, 0, "HOLD"
      elseif key == keys.down then altitude = altitude - 1; vectorX, vectorY, power, mode = 0, 0, 0, "HOLD"
      elseif key == keys.left then vectorX, vectorY, power, mode = -.2, 0, 5, "TEST X-"
      elseif key == keys.right then vectorX, vectorY, power, mode = .2, 0, 5, "TEST X+"
      elseif key == keys.f then vectorX, vectorY, power, mode = 0, .2, 5, "TEST Y+"
      elseif key == keys.b then vectorX, vectorY, power, mode = 0, -.2, 5, "TEST Y-"
      end
    end
  end
end)
stop()
if not ok then term.setTextColor(colors.red); print("Controller stopped: " .. tostring(err)) end
