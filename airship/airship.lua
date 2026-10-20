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

local monitor = peripheral.find("monitor")
local frame = monitor and basalt.createFrame(monitor) or basalt.getMainFrame()
frame.background = colors.black
local screenWidth, screenHeight
if monitor then screenWidth, screenHeight = monitor.getSize() else screenWidth, screenHeight = term.getSize() end
local compact = screenWidth < 49 or screenHeight < 20
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner, vectorX, vectorY, power = 0, 0, 0, 0
local calibrated = fs.exists("airship-calibrated")
local running, calibrationClick = true, nil
local axisTests = {}
-- Conservative starting gains; GPS velocity feedback compensates for variable sample timing.
local pid = {
  horizontalX = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  horizontalZ = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  altitude = {p=1.0, i=.03, d=2.0, bias=4, limit=11, integral=0, integralLimit=60}
}
local gainLimits = {{.005,.15}, {0,.01}, {0,.8}, {0,12}, {.1,3}, {0,.2}, {0,5}}
local function applyGainValues(values)
  for i, bounds in ipairs(gainLimits) do
    if type(values[i]) ~= "number" or values[i] < bounds[1] or values[i] > bounds[2] then return false end
  end
  pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d = values[1], values[2], values[3]
  pid.horizontalZ.p, pid.horizontalZ.i, pid.horizontalZ.d = values[1], values[2], values[3]
  pid.altitude.bias, pid.altitude.p, pid.altitude.i, pid.altitude.d = values[4], values[5], values[6], values[7]
  pid.horizontalX.integral, pid.horizontalZ.integral, pid.altitude.integral = 0, 0, 0
  return true
end
local function readGainFile()
  if not fs.exists("airship-pid.cfg") then return end
  local file = fs.open("airship-pid.cfg", "r")
  local values = {}
  for token in file.readAll():gmatch("%S+") do
    local value = tonumber(token)
    if not value then file.close(); return end
    values[#values + 1] = value
  end
  file.close()
  if #values == #gainLimits then applyGainValues(values) end
end
local function gainValues()
  return {pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d, pid.altitude.bias, pid.altitude.p, pid.altitude.i, pid.altitude.d}
end
readGainFile()
local velocity = {x=0, y=0, z=0}
local previousPosition, previousSample
local sampleDt = 1

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
  pid.horizontalX.integral, pid.horizontalZ.integral, pid.altitude.integral = 0, 0, 0
end
local function locate()
  local x, y, z = gps.locate(1, false)
  if x then
    local now = os.clock()
    if previousPosition and previousSample then
      sampleDt = clamp(now - previousSample, .25, 2)
      velocity.x = velocity.x * .5 + ((x - previousPosition.x) / sampleDt) * .5
      velocity.y = velocity.y * .5 + ((y - previousPosition.y) / sampleDt) * .5
      velocity.z = velocity.z * .5 + ((z - previousPosition.z) / sampleDt) * .5
    end
    position = {x=x, y=y, z=z}
    previousPosition, previousSample = position, now
    if not altitude then altitude = math.floor(y + .5) end
    if mode == "WAITING FOR GPS" then mode = "HOLDING ALTITUDE" end
  elseif mode == "AUTOPILOT" then
    position = nil
    previousPosition, previousSample = nil, nil
    velocity = {x=0, y=0, z=0}
    vectorX, vectorY, power, burner, mode = 0, 0, 0, 0, "GPS LOST - RE-ENGAGE"
  else
    position = nil
    previousPosition, previousSample = nil, nil
    velocity = {x=0, y=0, z=0}
  end
end
local function pidStep(controller, error, measuredVelocity, dt)
  local integralLimit = controller.integralLimit or 20
  local candidate = clamp(controller.integral + error * dt, -integralLimit, integralLimit)
  local raw = controller.p * error + controller.i * candidate - controller.d * measuredVelocity
  local output = clamp(raw, -controller.limit, controller.limit)
  if raw == output or raw * error < 0 then controller.integral = candidate end
  return output
end
local function updateFlight()
  if not position then
    vectorX, vectorY, power, burner = 0, 0, 0, 0
    if mode ~= "STOPPED" and mode ~= "GPS LOST - RE-ENGAGE" then mode = "WAITING FOR GPS" end
    return
  end
  if mode == "STOPPED" or mode == "GPS LOST - RE-ENGAGE" then return end
  if not altitude then burner = 0; mode = "SET ALTITUDE"; return end
  local altitudeError = altitude - position.y
  burner = math.floor(clamp(pid.altitude.bias + pidStep(pid.altitude, altitudeError, velocity.y, sampleDt), 0, 15) + .5)
  if mode == "AUTOPILOT" and target then
    local dx, dz = target.x - position.x, target.z - position.z
    if math.abs(dx) < .5 and math.abs(dz) < .5 and math.abs(velocity.x) < .12 and math.abs(velocity.z) < .12 then
      vectorX, vectorY, power, mode = 0, 0, 0, "DESTINATION REACHED"
    else
      vectorX = pidStep(pid.horizontalZ, dz, velocity.z, sampleDt)
      vectorY = pidStep(pid.horizontalX, dx, velocity.x, sampleDt)
      power = (math.abs(vectorX) + math.abs(vectorY) > .015) and 12 or 0
    end
  end
end

local status, pidSummary, posText, altText, sysText, xInput, heightInput, pidInput, calibrateButton
local showGainValues
local function adjustAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  altitude = (altitude or math.floor(position.y + .5)) + delta
  setFlightMode("HOLDING ALTITUDE")
end
local function holdHere()
  if position then altitude = math.floor(position.y + .5); target = nil; setFlightMode("HOLDING ALTITUDE") end
end
local function engageAutopilot()
  if not calibrated then status.text = "Run low-power direction checks, then confirm calibration."; status.foreground = colors.orange
  elseif not position or not target then status.text = "GPS fix and destination are required."; status.foreground = colors.red
  else setFlightMode("AUTOPILOT"); status.text = "Autopilot engaged"; status.foreground = colors.lime end
end
local function testVector(x, y, label)
  axisTests[label] = true
  vectorX, vectorY, power, mode = x, y, 5, label
  status.text = "Low-power test: " .. label .. "  |  Press HOLD to stop"
  status.foreground = colors.orange
end
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

if compact then
  frame:addLabel({x=2, y=1, width=screenWidth-2, height=1, text="AIRSHIP / FLIGHT DECK", foreground=colors.white, background=colors.blue})
  status = frame:addLabel({x=2, y=2, width=screenWidth-2, height=1, text="Starting...", foreground=colors.yellow})
  posText = frame:addLabel({x=2, y=3, width=screenWidth-2, height=1, text="GPS --", foreground=colors.white})
  altText = frame:addLabel({x=2, y=4, width=screenWidth-2, height=1, text="ALT -- / TARGET --", foreground=colors.lightBlue})
  frame:addLabel({x=2, y=5, width=12, height=1, text="DESTINATION X Z", foreground=colors.lightBlue})
  xInput = frame:addInput({x=2, y=6, width=screenWidth-19, height=1, placeholder="X Z", background=colors.white, foreground=colors.black})
  local function setDestination()
    local xText, zText = xInput.text:match("^%s*(%S+)%s+(%S+)%s*$")
    local x, z = tonumber(xText), tonumber(zText)
    if not x or not z then status.text = "Enter X and Z coordinates"; status.foreground = colors.red; return end
    target = {x=x, z=z}; setFlightMode("HOLDING ALTITUDE"); frame:setFocused(nil)
    status.text = ("Destination %.0f, %.0f set"):format(x, z)
  end
  frame:addButton({x=screenWidth-16, y=6, width=14, height=1, text="SET DEST", background=colors.blue}):onClick(setDestination)
  xInput:onEnter(setDestination)
  frame:addLabel({x=2, y=7, width=12, height=1, text="ALTITUDE TARGET", foreground=colors.lightBlue})
  heightInput = frame:addInput({x=2, y=8, width=8, height=1, placeholder="Y", background=colors.white, foreground=colors.black})
  local function setHeight()
    local value = tonumber(heightInput.text)
    if not value then status.text = "Enter a numeric height"; status.foreground = colors.red; return end
    altitude = value; setFlightMode("HOLDING ALTITUDE"); frame:setFocused(nil)
  end
  heightInput:onEnter(setHeight)
  frame:addButton({x=11, y=8, width=10, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
  frame:addButton({x=22, y=8, width=6, height=1, text="ALT -", background=colors.orange}):onClick(function() adjustAltitude(-1) end)
  frame:addButton({x=29, y=8, width=6, height=1, text="ALT +", background=colors.orange}):onClick(function() adjustAltitude(1) end)
  frame:addButton({x=2, y=9, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  frame:addButton({x=13, y=9, width=10, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  frame:addButton({x=24, y=9, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  frame:addButton({x=2, y=10, width=8, height=1, text="TEST X-", background=colors.gray}):onClick(function() testVector(-.2, 0, "TEST X-") end)
  frame:addButton({x=11, y=10, width=8, height=1, text="TEST X+", background=colors.gray}):onClick(function() testVector(.2, 0, "TEST X+") end)
  frame:addButton({x=20, y=10, width=8, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  frame:addButton({x=29, y=10, width=8, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  calibrateButton = frame:addButton({x=2, y=11, width=13, height=1, text=calibrated and "CALIBRATED" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  frame:addLabel({x=16, y=11, width=8, height=1, text="PID gains", foreground=colors.lightBlue})
  pidInput = frame:addInput({x=2, y=12, width=screenWidth-13, height=1, placeholder="X/Z P I D  ALT bias P I D", background=colors.white, foreground=colors.black})
  showGainValues = function()
    pidInput.text = ("%.3g %.1g %.2g %.0f %.1g %.1g %.1g"):format(table.unpack(gainValues()))
  end
  showGainValues()
  local function setGains()
    local values = {}
    for token in pidInput.text:gmatch("%S+") do values[#values + 1] = tonumber(token) end
    if #values ~= #gainLimits or not applyGainValues(values) then status.text = "Invalid PID values"; status.foreground = colors.red; return end
    local file = fs.open("airship-pid.cfg", "w"); file.write(table.concat(values, " ")); file.close()
    showGainValues(); frame:setFocused(nil); status.text = "PID gains saved"
  end
  pidInput:onEnter(setGains)
  frame:addButton({x=screenWidth-10, y=12, width=8, height=1, text="SET PID", background=colors.blue}):onClick(setGains)
else
frame:addLabel({x=2, y=1, width=47, height=1, text="AIRSHIP  /  FLIGHT DECK", foreground=colors.white, background=colors.blue})
status = frame:addLabel({x=2, y=2, width=47, height=1, text="Starting controller...", foreground=colors.yellow})
pidSummary = frame:addLabel({x=2, y=3, width=47, height=1, text="PID  loading gains...", foreground=colors.lightGray})

local function card(x, width, title)
  local box = frame:addFrame({x=x, y=4, width=width, height=4, background=colors.gray})
  box:addLabel({x=2, y=1, width=width-2, height=1, text=title, foreground=colors.lightBlue, background=colors.gray})
  return box
end
local gpsCard = card(2, 16, "GPS POSITION")
local altitudeCard = card(19, 15, "ALTITUDE")
local systemCard = card(35, 15, "FLIGHT SYSTEM")
posText = gpsCard:addLabel({x=2, y=2, width=13, height=3, text="X --\nY --\nZ --", foreground=colors.white, background=colors.gray})
altText = altitudeCard:addLabel({x=2, y=2, width=12, height=2, text="Now --\nTarget --", foreground=colors.white, background=colors.gray})
sysText = systemCard:addLabel({x=2, y=2, width=12, height=2, text="Burner 0/15\nThrust 0/15", foreground=colors.white, background=colors.gray})

frame:addLabel({x=2, y=9, width=22, height=1, text="DESTINATION  (X Z)", foreground=colors.lightBlue})
xInput = frame:addInput({x=2, y=10, width=22, height=1, placeholder="Example: 200 -700", background=colors.white, foreground=colors.black})
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

frame:addLabel({x=2, y=11, width=16, height=1, text="ALTITUDE TARGET Y", foreground=colors.lightBlue})
heightInput = frame:addInput({x=2, y=12, width=12, height=1, placeholder="Height", background=colors.white, foreground=colors.black})
local function setHeight()
  local value = tonumber(heightInput.text)
  if not value then status.text = "Enter a numeric height target."; status.foreground = colors.red; return end
  altitude = value
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Height target set to Y %.0f"):format(value); status.foreground = colors.lime
  frame:setFocused(nil)
end
frame:addButton({x=16, y=12, width=12, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
heightInput:onEnter(setHeight)
frame:addLabel({x=2, y=13, width=47, height=1, text="PID TUNE: X/Z P I D   ALT bias P I D", foreground=colors.lightBlue})
pidInput = frame:addInput({x=2, y=14, width=35, height=1, placeholder="0.035 0.0005 0.18 4 1 0.03 2", background=colors.white, foreground=colors.black})
showGainValues = function()
  pidInput.text = ("%.4f %.5f %.2f %.1f %.2f %.3f %.2f"):format(table.unpack(gainValues()))
end
showGainValues()
local function setGains()
  local values = {}
  for token in pidInput.text:gmatch("%S+") do
    local value = tonumber(token)
    if not value then status.text = "Use seven numeric PID values."; status.foreground = colors.red; return end
    values[#values + 1] = value
  end
  if #values ~= #gainLimits or not applyGainValues(values) then
    status.text = "Invalid gains. Check ranges in the setup guide."; status.foreground = colors.red; return
  end
  local file = fs.open("airship-pid.cfg", "w")
  file.write(table.concat(values, " ")); file.close()
  showGainValues()
  frame:setFocused(nil)
  status.text = "PID gains applied and saved."; status.foreground = colors.lime
end
frame:addButton({x=39, y=14, width=10, height=1, text="SET PID", background=colors.blue}):onClick(setGains)
pidInput:onEnter(setGains)

local function button(x, y, width, label, color, action)
  return frame:addButton({x=x, y=y, width=width, height=2, text=label, background=color}):onClick(action)
end
button(2, 15, 12, "HOLD HERE", colors.blue, holdHere)
button(15, 15, 14, "AUTOPILOT", colors.lime, engageAutopilot)
button(32, 15, 10, "STOP", colors.red, stop)

button(2, 17, 11, "X- / LEFT", colors.gray, function() testVector(-.2, 0, "TEST X-") end)
button(14, 17, 11, "X+ / RIGHT", colors.gray, function() testVector(.2, 0, "TEST X+") end)
button(26, 17, 11, "Y+ / FWD", colors.gray, function() testVector(0, .2, "TEST Y+") end)
button(38, 17, 11, "Y- / BACK", colors.gray, function() testVector(0, -.2, "TEST Y-") end)

calibrateButton = frame:addButton({x=2, y=19, width=14, height=1, text=calibrated and "CALIBRATED" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
frame:addLabel({x=18, y=19, width=31, height=1, text="G dest  Y height  T tune  A auto", foreground=colors.lightGray})
end

local function refresh()
  if compact then
    posText.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    altText.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
  else
    posText.text = position and ("X " .. ("%.1f"):format(position.x) .. "  Y " .. ("%.1f"):format(position.y) .. "\nZ " .. ("%.1f"):format(position.z)) or "GPS not found"
    altText.text = ("Now %s\nTarget %s"):format(position and ("%.1f"):format(position.y) or "--", altitude and ("%.0f"):format(altitude) or "--")
    sysText.text = ("Burner %d/15\nThrust %d/15"):format(burner, power)
  end
  if frame:getFocused() ~= heightInput then heightInput.text = altitude and tostring(math.floor(altitude + .5)) or "" end
  if frame:getFocused() ~= pidInput then showGainValues() end
  if pidSummary then pidSummary.text = ("XY %.3f/%.4f/%.2f ALT %.1f/%.1f/%.2f/%.1f"):format(pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d, pid.altitude.bias, pid.altitude.p, pid.altitude.i, pid.altitude.d) end
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
      if focused == xInput or focused == heightInput or focused == pidInput then
        if key == keys.up then adjustAltitude(1) elseif key == keys.down then adjustAltitude(-1) end
      else
        if key == keys.up then adjustAltitude(1)
        elseif key == keys.down then adjustAltitude(-1)
        elseif key == keys.left then testVector(-.2, 0, "TEST X-")
        elseif key == keys.right then testVector(.2, 0, "TEST X+")
        elseif key == keys.f then testVector(0, .2, "TEST Y+")
        elseif key == keys.b then testVector(0, -.2, "TEST Y-")
        elseif key == keys.g then xInput:focus()
        elseif key == keys.y then heightInput:focus()
        elseif key == keys.t then pidInput:focus()
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
