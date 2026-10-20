-- Basalt 2.5 flight panel for the east-facing eight-thruster ship.
local basalt = require("basalt")
local ids = {
  "liquid_vector_thruster_0", "liquid_vector_thruster_1",
  "liquid_vector_thruster_2", "liquid_vector_thruster_3",
  "liquid_vector_thruster_4", "liquid_vector_thruster_5",
  "liquid_vector_thruster_6", "liquid_vector_thruster_7"
}
local thrusters = {}
for _, name in ipairs(ids) do
  thrusters[name] = peripheral.wrap(name)
  if not thrusters[name] then error("Missing thruster: " .. name) end
end

local frame = basalt.getMainFrame()
frame.background = colors.black
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner, vectorX, vectorY, power = 0, 0, 0, 0
local calibrated = fs.exists("airship-calibrated")
local running, calibrationClick = true, nil
local axisTests = {}

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function outputs()
  for _, thruster in pairs(thrusters) do
    thruster.setVector(vectorX, vectorY)
    thruster.setThrust(power)
  end
  redstone.setAnalogOutput("front", burner)
end
local function stop()
  vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "STOPPED"
  pcall(outputs)
end
local function setFlightMode(nextMode)
  vectorX, vectorY, power, mode = 0, 0, 0, nextMode
end
local function locate()
  local x, y, z = gps.locate(1, false)
  if x then
    position = {x=x, y=y, z=z}
    if not altitude then altitude = math.floor(y + .5) end
    if mode == "WAITING FOR GPS" then mode = "HOLDING ALTITUDE" end
  elseif mode == "AUTOPILOT" then
    position = nil
    vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "GPS LOST - RE-ENGAGE"
  else
    position = nil
  end
end
local function updateFlight()
  if not position then
    vectorX, vectorY, power, burner = 0, 0, 0, 0
    if mode ~= "STOPPED" and mode ~= "GPS LOST - RE-ENGAGE" then mode = "WAITING FOR GPS" end
    return
  end
  if mode == "STOPPED" or mode == "GPS LOST - RE-ENGAGE" then return end
  if not altitude then burner = 0; mode = "SET ALTITUDE"; return end
  burner = altitude - position.y > 1 and 15 or 0
  if mode == "AUTOPILOT" and target then
    local dx, dz = target.x - position.x, target.z - position.z
    local distance = math.sqrt(dx*dx + dz*dz)
    if distance < 2 then
      vectorX, vectorY, power, mode = 0, 0, 0, "DESTINATION REACHED"
    else
      vectorX = clamp((dz / distance) * .25, -.25, .25)
      vectorY = clamp((dx / distance) * .25, -.25, .25)
      power = 12
    end
  end
end

frame:addLabel({x=2, y=1, width=47, height=1, text="AIRSHIP  /  FLIGHT DECK", foreground=colors.white, background=colors.blue})
local status = frame:addLabel({x=2, y=2, width=47, height=1, text="Starting controller...", foreground=colors.yellow})

local function card(x, width, title)
  local box = frame:addFrame({x=x, y=4, width=width, height=4, background=colors.gray})
  box:addLabel({x=2, y=1, width=width-2, height=1, text=title, foreground=colors.lightBlue, background=colors.gray})
  return box
end
local gpsCard = card(2, 16, "GPS POSITION")
local altitudeCard = card(19, 15, "ALTITUDE")
local systemCard = card(35, 15, "FLIGHT SYSTEM")
local posText = gpsCard:addLabel({x=2, y=2, width=13, height=3, text="X --\nY --\nZ --", foreground=colors.white, background=colors.gray})
local altText = altitudeCard:addLabel({x=2, y=2, width=12, height=2, text="Now --\nTarget --", foreground=colors.white, background=colors.gray})
local sysText = systemCard:addLabel({x=2, y=2, width=12, height=2, text="Burner 0/15\nThrust 0/15", foreground=colors.white, background=colors.gray})

frame:addLabel({x=2, y=9, width=22, height=1, text="DESTINATION  (X Z)", foreground=colors.lightBlue})
local xInput = frame:addInput({x=2, y=10, width=22, height=1, placeholder="Example: 200 -700", background=colors.white, foreground=colors.black})
local function setDestination()
  local xText, zText = xInput.text:match("^%s*(%S+)%s+(%S+)%s*$")
  local x, z = tonumber(xText), tonumber(zText)
  if not x or not z then status.text = "Enter numeric X and Z coordinates first."; status.foreground = colors.red; return end
  target = {x=x, z=z}
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Destination set: X %.0f, Z %.0f"):format(x, z)
  status.foreground = colors.lime
  frame:setFocused(nil)
end
frame:addButton({x=26, y=10, width=12, height=1, text="SET DEST", background=colors.blue}):onClick(setDestination)
xInput:onEnter(setDestination)

local function adjustAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  altitude = (altitude or math.floor(position.y + .5)) + delta
  setFlightMode("HOLDING ALTITUDE")
end

local function button(x, y, width, label, color, action)
  return frame:addButton({x=x, y=y, width=width, height=2, text=label, background=color}):onClick(action)
end
button(2, 12, 7, "ALT -", colors.orange, function() adjustAltitude(-1) end)
button(10, 12, 7, "ALT +", colors.orange, function() adjustAltitude(1) end)
local function holdHere()
  if position then altitude = math.floor(position.y + .5); target = nil; setFlightMode("HOLDING ALTITUDE") end
end
button(19, 12, 9, "HOLD HERE", colors.blue, holdHere)
local function engageAutopilot()
  if not calibrated then status.text = "Run low-power direction checks, then confirm calibration."; status.foreground = colors.orange
  elseif not position or not target then status.text = "GPS fix and destination are required."; status.foreground = colors.red
  else setFlightMode("AUTOPILOT"); status.text = "Autopilot engaged"; status.foreground = colors.lime end
end
button(29, 12, 9, "AUTOPILOT", colors.lime, engageAutopilot)
button(39, 12, 10, "STOP", colors.red, stop)

local function testVector(x, y, label)
  axisTests[label] = true
  vectorX, vectorY, power, mode = x, y, 5, label
  status.text = "Low-power test: " .. label .. "  |  Press HOLD to stop"
  status.foreground = colors.orange
end
button(2, 15, 11, "X- / LEFT", colors.gray, function() testVector(-.2, 0, "TEST X-") end)
button(14, 15, 11, "X+ / RIGHT", colors.gray, function() testVector(.2, 0, "TEST X+") end)
button(26, 15, 11, "Y+ / FWD", colors.gray, function() testVector(0, .2, "TEST Y+") end)
button(38, 15, 11, "Y- / BACK", colors.gray, function() testVector(0, -.2, "TEST Y-") end)

local calibrateButton
local function confirmCalibration()
  if calibrated then status.text = "Calibration already saved on this computer."; return end
  if not (axisTests["TEST X-"] and axisTests["TEST X+"] and axisTests["TEST Y+"] and axisTests["TEST Y-"]) then
    status.text = "Calibration needs all four direction tests."; status.foreground = colors.red; return
  end
  if calibrationClick and os.clock() - calibrationClick < 8 then
    local file = fs.open("airship-calibrated", "w"); file.write("east front, X right, Y forward"); file.close()
    calibrated, calibrationClick = true, nil
    calibrateButton.text, calibrateButton.background = "CALIBRATED", colors.green
    mode = "TEST CALIBRATED"
    status.text = "Calibration saved. Autopilot is ready."; status.foreground = colors.lime
  else
    calibrationClick = os.clock()
    mode = "CALIBRATION CONFIRM"
    status.text = "Confirm within 8s: X+ south, Y+ east."; status.foreground = colors.orange
  end
end
calibrateButton = button(2, 18, 14, calibrated and "CALIBRATED" or "CALIBRATE", calibrated and colors.green or colors.orange, confirmCalibration)
frame:addLabel({x=18, y=18, width=31, height=1, text="G dest  A auto  H hold  S stop", foreground=colors.lightGray})

local function refresh()
  posText.text = position and ("X " .. ("%.1f"):format(position.x) .. "  Y " .. ("%.1f"):format(position.y) .. "\nZ " .. ("%.1f"):format(position.z)) or "GPS not found"
  altText.text = ("Now %s\nTarget %s"):format(position and ("%.1f"):format(position.y) or "--", altitude and ("%.0f"):format(altitude) or "--")
  sysText.text = ("Burner %d/15\nThrust %d/15"):format(burner, power)
  if mode == "AUTOPILOT" then status.text, status.foreground = "AUTOPILOT ACTIVE  |  EAST-FACING SHIP", colors.lime
  elseif mode == "GPS LOST - RE-ENGAGE" then status.text, status.foreground = mode, colors.red
  elseif mode == "WAITING FOR GPS" then status.text, status.foreground = "Waiting for GPS signal...", colors.orange
  elseif mode == "HOLDING ALTITUDE" then status.text, status.foreground = "ALTITUDE HOLD ACTIVE", colors.lightBlue
  elseif mode == "DESTINATION REACHED" then status.text, status.foreground = "DESTINATION REACHED  |  HOLDING ALTITUDE", colors.lime
  elseif mode == "SET ALTITUDE" then status.text, status.foreground = "GPS ready  |  set an altitude target", colors.orange
  elseif mode == "CALIBRATION CONFIRM" then status.text, status.foreground = "Confirm calibration within 8 seconds", colors.orange
  elseif mode:match("^TEST ") then status.text, status.foreground = mode .. "  |  press HOLD to stop", colors.orange
  elseif mode == "STOPPED" then status.text, status.foreground = "OUTPUTS STOPPED", colors.red end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(2, false)
  altitude = startY and math.floor(startY + .5) or nil
  if altitude then mode = "HOLDING ALTITUDE" else mode = "WAITING FOR GPS" end
  basalt.schedule(function()
    while running do
      local success = pcall(function() locate(); updateFlight(); outputs(); refresh() end)
      if not success then stop(); running = false; status.text = "Peripheral error: outputs stopped"; status.foreground = colors.red; basalt.stop(); return end
      sleep(1)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, key = os.pullEvent("key")
      local focused = frame:getFocused()
      if focused == xInput then
        if key == keys.up then adjustAltitude(1) elseif key == keys.down then adjustAltitude(-1) end
      else
        if key == keys.up then adjustAltitude(1)
        elseif key == keys.down then adjustAltitude(-1)
        elseif key == keys.left then testVector(-.2, 0, "TEST X-")
        elseif key == keys.right then testVector(.2, 0, "TEST X+")
        elseif key == keys.f then testVector(0, .2, "TEST Y+")
        elseif key == keys.b then testVector(0, -.2, "TEST Y-")
        elseif key == keys.g then xInput:focus()
        elseif key == keys.h then holdHere()
        elseif key == keys.a then engageAutopilot()
        elseif key == keys.s then stop()
        elseif key == keys.c then confirmCalibration()
        end
      end
    end
  end)
  basalt.run()
end)
running = false
stop()
if not ok then term.setTextColor(colors.red); print("Airship UI stopped: " .. tostring(err)) end
