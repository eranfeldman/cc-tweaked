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
local altitudeSensor = peripheral.find("altitude_sensor")
local frame = basalt.getMainFrame()
local monitorFrame = monitor and basalt.createFrame(monitor)
frame.background = colors.black
if monitorFrame then monitorFrame.background = colors.black end
local screenWidth, screenHeight
screenWidth, screenHeight = term.getSize()
local compact = screenWidth < 49 or screenHeight < 19
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner, burnerCommand, vectorX, vectorY, power = 0, 0, 0, 0, 0
local burnerSlewPerSecond = 6
local calibrated = fs.exists("airship-calibrated")
local running, calibrationClick, manualBurner = true, nil, nil
local altitudePresets = {-10, 10, 25, 50}
local presetIndex, presetBase = 0, nil
local learnedHover = {}
local altitudeSensorPressure
local measuredThrust = 0
local status, pidSummary, posText, altText, sysText, xInput, heightInput, pidInput, calibrateButton
local axisTests = {}
-- Conservative starting gains; GPS velocity feedback compensates for variable sample timing.
local pid = {
  horizontalX = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  horizontalZ = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  altitude = {p=.45, i=.005, d=2.5, bias=4, limit=11, integral=0, integralLimit=30}
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
if fs.exists("airship-hover.cfg") then
  local file = fs.open("airship-hover.cfg", "r")
  for line in file.readAll():gmatch("[^\r\n]+") do
    local height, bias = line:match("^([%-%.%d]+)%s+([%-%.%d]+)$")
    height, bias = tonumber(height), tonumber(bias)
    if height and bias and bias >= 0 and bias <= 15 then learnedHover[height] = bias end
  end
  file.close()
end
local function hoverBias(height)
  local points = {}
  for y, value in pairs(learnedHover) do points[#points + 1] = {y, value} end
  table.sort(points, function(a, b) return a[1] < b[1] end)
  height = altitudeSensorPressure or height
  if #points == 0 then return pid.altitude.bias end
  if height <= points[1][1] then return points[1][2] end
  if height >= points[#points][1] then return points[#points][2] end
  for i = 1, #points - 1 do
    local low, high = points[i], points[i + 1]
    if height >= low[1] and height <= high[1] then
      local fraction = (height - low[1]) / (high[1] - low[1])
      return low[2] + (high[2] - low[2]) * fraction
    end
  end
  return pid.altitude.bias
end
local function saveHoverMap()
  return pcall(function()
    local file = assert(fs.open("airship-hover.cfg.tmp", "w"))
    for pressure, value in pairs(learnedHover) do file.writeLine(("%.4f %.3f"):format(pressure, value)) end
    file.close()
    local hadPrevious = fs.exists("airship-hover.cfg")
    if fs.exists("airship-hover.cfg.bak") then fs.delete("airship-hover.cfg.bak") end
    if hadPrevious then fs.move("airship-hover.cfg", "airship-hover.cfg.bak") end
    local moved, err = pcall(fs.move, "airship-hover.cfg.tmp", "airship-hover.cfg")
    if not moved then
      if hadPrevious then fs.move("airship-hover.cfg.bak", "airship-hover.cfg") end
      error(err)
    end
    if fs.exists("airship-hover.cfg.bak") then fs.delete("airship-hover.cfg.bak") end
  end)
end
readGainFile()
local velocity = {x=0, y=0, z=0}
local previousPosition, previousSample
local sampleDt = .5
local altitudeSampleAt
local altitudeDt = .05
local altitudeSensorHeight
local altitudeSensorReady = false
local lastGpsAt

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function outputs()
  for _, thruster in pairs(thrusters) do
    thruster.setVector(vectorX, vectorY)
    thruster.setThrustNormalized(power)
  end
  redstone.setAnalogOutput("front", burner)
end
local function stop()
  manualBurner = nil
  vectorX, vectorY, power, burner, burnerCommand, mode = 0, 0, 0, 0, 0, "STOPPED"
  pcall(outputs)
end
local function setFlightMode(nextMode)
  manualBurner = nil
  vectorX, vectorY, power, mode = 0, 0, 0, nextMode
  pid.horizontalX.integral, pid.horizontalZ.integral, pid.altitude.integral = 0, 0, 0
end
local function locate()
  local now = os.clock()
  altitudeDt = clamp(now - (altitudeSampleAt or now - .05), .01, .5)
  altitudeSampleAt = now
  if altitudeSensor then
    local ok, height, verticalSpeed, pressure = pcall(function()
      return altitudeSensor.getHeight(), altitudeSensor.getVerticalSpeed(), altitudeSensor.getAirPressure()
    end)
    if ok and type(height) == "number" and height == height and math.abs(height) < math.huge then
      altitudeSensorHeight, altitudeSensorReady = height, true
      if type(pressure) == "number" and pressure == pressure and math.abs(pressure) < math.huge then altitudeSensorPressure = pressure end
      if position then position.y = height end
      if type(verticalSpeed) == "number" and verticalSpeed == verticalSpeed and math.abs(verticalSpeed) < math.huge then velocity.y = verticalSpeed end
    end
  end
  local x, y, z = gps.locate(.1, false)
  if x then
    if previousPosition and previousSample then
      sampleDt = clamp(now - previousSample, .01, 1.5)
      velocity.x = velocity.x * .5 + ((x - previousPosition.x) / sampleDt) * .5
      if not altitudeSensorReady then velocity.y = velocity.y * .5 + ((y - previousPosition.y) / sampleDt) * .5 end
      velocity.z = velocity.z * .5 + ((z - previousPosition.z) / sampleDt) * .5
    end
    position = {x=x, y=altitudeSensorReady and altitudeSensorHeight or y, z=z}
    previousPosition, previousSample = position, now
    lastGpsAt = now
    if not altitude then altitude = math.floor(position.y + .5) end
    if mode == "WAITING FOR GPS" then mode = "HOLDING ALTITUDE" end
    return true
  elseif lastGpsAt and os.clock() - lastGpsAt > .5 then
    position = nil
    previousPosition, previousSample = nil, nil
    velocity = {x=0, y=0, z=0}
    lastGpsAt = nil
    if mode ~= "STOPPED" then
      vectorX, vectorY, power, burner, burnerCommand = 0, 0, 0, 0, 0
      mode = mode == "AUTOPILOT" and "GPS LOST - RE-ENGAGE" or "WAITING FOR GPS"
    end
  end
  return false
end
local function pidStep(controller, error, measuredVelocity, dt)
  local integralLimit = controller.integralLimit or 20
  local candidate = clamp(controller.integral + error * dt, -integralLimit, integralLimit)
  local raw = controller.p * error + controller.i * candidate - controller.d * measuredVelocity
  local output = clamp(raw, -controller.limit, controller.limit)
  if raw == output or raw * error < 0 then controller.integral = candidate end
  return output
end
local function updateFlight(gpsFresh)
  if not position then
    vectorX, vectorY, power, burner, burnerCommand = 0, 0, 0, 0, 0
    if mode ~= "STOPPED" and mode ~= "GPS LOST - RE-ENGAGE" then mode = "WAITING FOR GPS" end
    return
  end
  if mode == "STOPPED" or mode == "GPS LOST - RE-ENGAGE" then return end
  if not altitude then burner, burnerCommand = 0, 0; mode = "SET ALTITUDE"; return end
  local altitudeError = altitude - position.y
  local dt = altitudeSensorReady and altitudeDt or sampleDt
  if manualBurner ~= nil then
    burnerCommand, burner = manualBurner, manualBurner
  else
    local desiredBurner = clamp(hoverBias(altitude) + pidStep(pid.altitude, altitudeError, velocity.y, dt), 0, 15)
    burnerCommand = burnerCommand + clamp(desiredBurner - burnerCommand, -burnerSlewPerSecond * dt, burnerSlewPerSecond * dt)
    burner = math.floor(burnerCommand + .5)
  end
  if gpsFresh and mode == "AUTOPILOT" and target then
    local dx, dz = target.x - position.x, target.z - position.z
    if math.abs(dx) < .5 and math.abs(dz) < .5 and math.abs(velocity.x) < .12 and math.abs(velocity.z) < .12 then
      vectorX, vectorY, power, mode = 0, 0, 0, "DESTINATION REACHED"
    else
      vectorX = pidStep(pid.horizontalZ, dz, velocity.z, sampleDt)
      vectorY = pidStep(pid.horizontalX, dx, velocity.x, sampleDt)
      power = clamp(math.sqrt(vectorX * vectorX + vectorY * vectorY) / .25 * .35, 0, .35)
    end
  end
end

local function selectAltitudePreset()
  if not position then status.text = "Waiting for GPS before selecting a height."; return end
  presetBase = presetBase or math.floor(position.y + .5)
  presetIndex = presetIndex % #altitudePresets + 1
  altitude = presetBase + altitudePresets[presetIndex]
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Preset %d: target Y %.0f"):format(presetIndex, altitude)
  status.foreground = colors.lightBlue
end
local function lockHoverLevel()
  if manualBurner == nil then
    status.text = "Press M to set the manual burner level before locking."; status.foreground = colors.orange; return
  end
  if not altitudeSensorReady or not altitudeSensorPressure then
    status.text = "Lock needs Avionics altitude and pressure readings."; status.foreground = colors.red; return
  end
  if not position or math.abs(velocity.y) >= .12 then
    status.text = "Wait until vertical speed is below 0.12, then lock."; status.foreground = colors.orange; return
  end
  local level = redstone.getAnalogOutput("front")
  learnedHover[altitudeSensorPressure] = level
  if saveHoverMap() then
    status.text = ("Locked burner level %d at Y %.1f / pressure %.3f"):format(level, altitudeSensorHeight, altitudeSensorPressure)
    status.foreground = colors.lime
  else
    status.text = "Level captured in memory, but could not save airship-hover.cfg."; status.foreground = colors.red
  end
end
local function toggleManualBurner()
  if manualBurner ~= nil then
    manualBurner = nil
    if position then altitude = math.floor(position.y + .5) end
    setFlightMode("HOLDING ALTITUDE")
    status.text = "Auto altitude hold restored at current height."; status.foreground = colors.lime
  else
    manualBurner = redstone.getAnalogOutput("front")
    vectorX, vectorY, power, target = 0, 0, 0, nil
    mode = "MANUAL BURNER"
    status.text = ("Manual burner: %d/15; Up/Down adjusts, K locks"):format(manualBurner)
    status.foreground = colors.orange
  end
end
local function adjustManualBurner(delta)
  if manualBurner == nil then return false end
  manualBurner = clamp(manualBurner + delta, 0, 15)
  status.text = ("Manual burner: %d/15; settle, then K to lock"):format(manualBurner)
  status.foreground = colors.orange
  return true
end
local function readThrusterPower()
  local total, count = 0, 0
  for _, thruster in pairs(thrusters) do
    local ok, value = pcall(thruster.getPower)
    if ok and type(value) == "number" and value == value then total, count = total + value, count + 1 end
  end
  measuredThrust = count > 0 and total / count * 15 or 0
end

local monitorStatus, monitorPos, monitorAlt, monitorSystem, monitorCalibrateButton
local showGainValues
local function adjustAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  altitude = (altitude or math.floor(position.y + .5)) + delta
  setFlightMode("HOLDING ALTITUDE")
end
local function holdHere()
  if position then altitude = math.floor(position.y + .5); presetBase = altitude; presetIndex = 0; target = nil; setFlightMode("HOLDING ALTITUDE") end
end
local function engageAutopilot()
  if not calibrated then status.text = "Run low-power direction checks, then confirm calibration."; status.foreground = colors.orange
  elseif not position or not target then status.text = "GPS fix and destination are required."; status.foreground = colors.red
  else setFlightMode("AUTOPILOT"); status.text = "Autopilot engaged"; status.foreground = colors.lime end
end
local function testVector(x, y, label)
  axisTests[label] = true
  vectorX, vectorY, power, mode = x, y, .33, label
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
    if calibrateButton then calibrateButton.text, calibrateButton.background = "CALIBRATED", colors.green end
    if monitorCalibrateButton then monitorCalibrateButton.text, monitorCalibrateButton.background = "CALIBRATED", colors.green end
    mode = "TEST CALIBRATED"
    status.text = "Calibration saved. Autopilot is ready."; status.foreground = colors.lime
  else
    calibrationClick = os.clock()
    mode = "CALIBRATION CONFIRM"
    status.text = "Confirm within 8s: X+ south, Y+ east."; status.foreground = colors.orange
  end
end

if monitorFrame then
  local width = monitor.getSize()
  monitorFrame:addLabel({x=2, y=1, width=width-2, height=1, text="AIRSHIP / FLIGHT DECK", foreground=colors.white, background=colors.blue})
  monitorStatus = monitorFrame:addLabel({x=2, y=2, width=width-2, height=1, text="Starting...", foreground=colors.yellow})
  monitorPos = monitorFrame:addLabel({x=2, y=3, width=width-2, height=1, text="GPS --", foreground=colors.white})
  monitorAlt = monitorFrame:addLabel({x=2, y=4, width=width-2, height=1, text="ALT -- / TARGET --", foreground=colors.lightBlue})
  monitorSystem = monitorFrame:addLabel({x=2, y=5, width=width-2, height=1, text="BURNER 0/15  THRUST 0/15", foreground=colors.lightGray})
  monitorFrame:addButton({x=2, y=6, width=8, height=1, text="ALT -", background=colors.orange}):onClick(function() adjustAltitude(-1) end)
  monitorFrame:addButton({x=11, y=6, width=8, height=1, text="ALT +", background=colors.orange}):onClick(function() adjustAltitude(1) end)
  monitorFrame:addButton({x=2, y=7, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  monitorFrame:addButton({x=13, y=7, width=11, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  monitorFrame:addButton({x=25, y=7, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  monitorFrame:addButton({x=2, y=8, width=8, height=1, text="TEST X-", background=colors.gray}):onClick(function() testVector(-.2, 0, "TEST X-") end)
  monitorFrame:addButton({x=11, y=8, width=8, height=1, text="TEST X+", background=colors.gray}):onClick(function() testVector(.2, 0, "TEST X+") end)
  monitorFrame:addButton({x=20, y=8, width=8, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  monitorFrame:addButton({x=29, y=8, width=8, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  monitorCalibrateButton = monitorFrame:addButton({x=2, y=9, width=14, height=1, text=calibrated and "CALIBRATED" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  monitorFrame:addLabel({x=17, y=9, width=width-17, height=1, text="type on PC", foreground=colors.lightGray})
  monitorFrame:addButton({x=2, y=10, width=14, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  monitorFrame:addButton({x=18, y=10, width=12, height=1, text="ALT PRESET", background=colors.blue}):onClick(selectAltitudePreset)
  monitorFrame:addButton({x=32, y=10, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
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
  frame:addButton({x=27, y=11, width=12, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  frame:addButton({x=40, y=11, width=10, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
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
frame:addButton({x=30, y=12, width=13, height=1, text="ALT PRESET", background=colors.blue}):onClick(selectAltitudePreset)
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
frame:addButton({x=18, y=19, width=14, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
frame:addButton({x=34, y=19, width=15, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
end

local function refresh()
  readThrusterPower()
  if compact then
    posText.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    altText.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
  else
    posText.text = position and ("X " .. ("%.1f"):format(position.x) .. "  Y " .. ("%.1f"):format(position.y) .. "\nZ " .. ("%.1f"):format(position.z)) or "GPS not found"
    altText.text = ("Now %s\nTarget %s"):format(position and ("%.1f"):format(position.y) or "--", altitude and ("%.0f"):format(altitude) or "--")
    sysText.text = ("Burner %d/15\nThrust %.1f/15"):format(burner, measuredThrust)
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
  elseif mode == "MANUAL BURNER" then status.text, status.foreground = ("MANUAL BURNER %d/15 | UP/DOWN adjust, K locks"):format(manualBurner or 0), colors.orange
  elseif mode:match("^TEST ") then status.text, status.foreground = mode .. "  |  press HOLD to stop", colors.orange
  elseif mode == "STOPPED" then status.text, status.foreground = "OUTPUTS STOPPED", colors.red end
  if monitorFrame then
    monitorPos.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    monitorAlt.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
    monitorSystem.text = ("BURNER %d/15 THRUST %.1f/15"):format(burner, measuredThrust)
    monitorStatus.text, monitorStatus.foreground = status.text, status.foreground
  end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(.1, false)
  altitude = startY and math.floor(startY + .5) or nil
  if altitude then mode = "HOLDING ALTITUDE" else mode = "WAITING FOR GPS" end
  basalt.schedule(function()
    while running do
      local success = pcall(function()
        local gpsFresh = locate()
        updateFlight(gpsFresh)
        outputs()
      end)
      if not success then stop(); running = false; status.text = "Peripheral error: outputs stopped"; status.foreground = colors.red; basalt.stop(); return end
      sleep(.05)
    end
  end)
  basalt.schedule(function()
    while running do
      refresh()
      sleep(.05)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, key = os.pullEvent("key")
      local focused = frame:getFocused()
      if focused == xInput or focused == heightInput or focused == pidInput then
        if key == keys.up then if not adjustManualBurner(1) then adjustAltitude(1) end
        elseif key == keys.down then if not adjustManualBurner(-1) then adjustAltitude(-1) end end
      else
        if key == keys.up then if not adjustManualBurner(1) then adjustAltitude(1) end
        elseif key == keys.down then if not adjustManualBurner(-1) then adjustAltitude(-1) end
        elseif key == keys.left then testVector(-.2, 0, "TEST X-")
        elseif key == keys.right then testVector(.2, 0, "TEST X+")
        elseif key == keys.f then testVector(0, .2, "TEST Y+")
        elseif key == keys.b then testVector(0, -.2, "TEST Y-")
        elseif key == keys.g then xInput:focus()
        elseif key == keys.y then heightInput:focus()
        elseif key == keys.t then pidInput:focus()
        elseif key == keys.h then holdHere()
        elseif key == keys.p then selectAltitudePreset()
        elseif key == keys.m then toggleManualBurner()
        elseif key == keys.k then lockHoverLevel()
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
