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
local monitorName
for _, name in ipairs(peripheral.getNames()) do
  for _, kind in ipairs({peripheral.getType(name)}) do
    if type(kind) == "string" and kind:find("monitor", 1, true) then
      monitorName = name
      break
    end
  end
  if monitorName then break end
end
local directGpu, directDisplay, directWidth, directHeight, directGpuError
local function setupDirectGpu()
  local gpuFound = peripheral.find("directgpu")
  if not gpuFound then directGpuError = "DirectGPU peripheral not found"; return false end
  monitorName = nil
  for _, name in ipairs(peripheral.getNames()) do
    for _, kind in ipairs({peripheral.getType(name)}) do
      if type(kind) == "string" and kind:find("monitor", 1, true) then monitorName = name; break end
    end
    if monitorName then break end
  end
  local listOk, displays = pcall(gpuFound.listDisplays)
  local display = listOk and displays and displays[1]
  local function getInfo(id)
    if id == nil then return nil, "no display ID returned" end
    local ok, info = pcall(gpuFound.getDisplayInfo, id)
    if not ok then return nil, tostring(info) end
    if not info or type(info.pixelWidth) ~= "number" or type(info.pixelHeight) ~= "number" then
      return nil, "display info has no pixel dimensions"
    end
    return info
  end
  local info, infoError = getInfo(display)
  local setupError
  if not info then
    local ok, newDisplay
    if monitorName then
      ok, newDisplay = pcall(gpuFound.autoDetectAndCreateDisplayWithResolution, monitorName, 1)
      if not ok then setupError = "create display for " .. monitorName .. ": " .. tostring(newDisplay) end
    end
    if not monitorName or not ok or newDisplay == nil then
      ok, newDisplay = pcall(gpuFound.autoDetectAndCreateDisplayWithResolution, 1)
      if ok then setupError = nil else setupError = "create display: " .. tostring(newDisplay) end
    end
    if ok then
      display = newDisplay
      info, infoError = getInfo(display)
    else
      setupError = setupError or ("create display: " .. tostring(newDisplay))
    end
  end
  if info then
    directGpu, directDisplay = gpuFound, display
    directWidth, directHeight = info.pixelWidth, info.pixelHeight
    directGpuError = nil
    return true
  else
    local scanOk, monitorInfo = pcall(gpuFound.autoDetectMonitor)
    if not scanOk then
      setupError = setupError or ("monitor scan: " .. tostring(monitorInfo))
      directGpuError = "Setup failed: " .. setupError
    elseif monitorInfo ~= nil then
      directGpuError = "Display setup: " .. tostring(setupError or infoError)
    else
      directGpuError = "No monitor near DirectGPU block; move it beside monitor"
    end
    directGpu = nil
    return false
  end
end
setupDirectGpu()
local altitudeSensor = peripheral.find("altitude_sensor")
local frame = basalt.getMainFrame()
local monitorFrame = monitor and not directGpu and basalt.createFrame(monitor)
frame.background = colors.black
if monitorFrame then monitorFrame.background = colors.black end
local screenWidth, screenHeight
screenWidth, screenHeight = term.getSize()
local compact = screenWidth < 49 or screenHeight < 19
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner, burnerCommand, vectorX, vectorY, power = 0, 0, 0, 0, 0
local burnerSlewPerSecond = 6
local calibrated = fs.exists("airship-calibrated")
local running, calibrationClick, manualBurner, burnerSweep = true, nil, nil, nil
local sweepConfirmAt
local sweepSummary
local sweepSummaryUntil
local gpuButtons = {}
local savedGpuButtons = {}
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
    local pressure, level, height = line:match("^([%-%.%d]+)%s+([%-%.%d]+)%s*([%-%.%d]*)$")
    pressure, level, height = tonumber(pressure), tonumber(level), tonumber(height)
    if pressure and level and level >= 0 and level <= 15 then
      local key = ("%.3f"):format(height or pressure)
      learnedHover[key] = {pressure=pressure, level=level, height=height}
    end
  end
  file.close()
end
local function hoverBias(height)
  local points = {}
  for _, sample in pairs(learnedHover) do
    if type(sample.height) == "number" then
      points[#points + 1] = {sample.height, sample.level}
    end
  end
  table.sort(points, function(a, b) return a[1] < b[1] end)
  if #points == 0 then return nil end
  if height <= points[1][1] then return points[1][2] end
  if height >= points[#points][1] then return points[#points][2] end
  for i = 1, #points - 1 do
    local low, high = points[i], points[i + 1]
    if height >= low[1] and height <= high[1] then
      if high[1] == low[1] then return high[2] end
      local fraction = (height - low[1]) / (high[1] - low[1])
      return low[2] + (high[2] - low[2]) * fraction
    end
  end
  return nil
end
local function saveHoverMap()
  return pcall(function()
    local file = assert(fs.open("airship-hover.cfg.tmp", "w"))
    for _, sample in pairs(learnedHover) do file.writeLine(("%.4f %.3f %.2f"):format(sample.pressure, sample.level, sample.height or 0)) end
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
  if burnerSweep then
    sweepSummary = ("Sweep canceled. Saved %d/11; skipped %d."):format(burnerSweep.saved, burnerSweep.skipped)
    sweepSummaryUntil = os.clock() + 15
  end
  manualBurner = nil
  burnerSweep = nil
  vectorX, vectorY, power, burner, burnerCommand, mode = 0, 0, 0, 0, 0, "STOPPED"
  pcall(outputs)
end
local updateRequested = false
local function requestProgramUpdate()
  if not shell or type(shell.openTab) ~= "function" then
    status.text = "Update button needs CC:Tweaked Multishell."
    status.foreground = colors.orange
    return
  end
  if not fs.exists("/airship-updater.lua") then
    status.text = "Updater missing; install airship-updater.lua once."
    status.foreground = colors.red
    return
  end
  updateRequested = true
  stop()
  basalt.stop()
end
local function setFlightMode(nextMode)
  manualBurner = nil
  burnerSweep = nil
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
      if burnerSweep then
        sweepSummary = ("Sweep aborted: GPS lost. Saved %d/11; skipped %d."):format(burnerSweep.saved, burnerSweep.skipped)
        sweepSummaryUntil = os.clock() + 15
      end
      burnerSweep, manualBurner = nil, nil
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
  if burnerSweep then
    burnerCommand = burnerCommand + clamp(burnerSweep.level - burnerCommand, -burnerSlewPerSecond * dt, burnerSlewPerSecond * dt)
    burner = math.floor(burnerCommand + .5)
  elseif manualBurner ~= nil then
    burnerCommand, burner = manualBurner, manualBurner
  else
    local mappedBurner = hoverBias(altitude)
    local desiredBurner = clamp(mappedBurner or ((hoverBias(altitude) or pid.altitude.bias) + pidStep(pid.altitude, altitudeError, velocity.y, dt)), 0, 15)
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

local function savedAltitudes()
  local heights = {}
  for _, sample in pairs(learnedHover) do
    if type(sample.height) == "number" then heights[#heights + 1] = sample.height end
  end
  table.sort(heights)
  return heights
end
local function selectAltitudePreset()
  if not position then status.text = "Waiting for GPS before selecting a height."; return end
  local heights = savedAltitudes()
  if #heights == 0 then
    status.text = "No saved calibration altitudes. Run a sweep or lock a burner level."
    status.foreground = colors.orange
    return
  end
  local selected = 1
  for i, height in ipairs(heights) do
    if altitude and height > altitude + .001 then selected = i; break end
  end
  altitude = heights[selected]
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Saved altitude %d/%d: target Y %.1f"):format(selected, #heights, altitude)
  status.foreground = colors.lightBlue
end
local function selectSavedAltitude(index)
  local height = savedAltitudes()[index]
  if height then
    altitude = height
    setFlightMode("HOLDING ALTITUDE")
    status.text = ("Saved target selected: Y %.1f"):format(height)
    status.foreground = colors.lightBlue
  end
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
  local pressure = tonumber(("%.4f"):format(altitudeSensorPressure))
  local key = ("%.3f"):format(altitudeSensorHeight)
  learnedHover[key] = {pressure=pressure, level=level, height=altitudeSensorHeight}
  if saveHoverMap() then
    status.text = ("Locked signal %d at Y %.1f / pressure %.3f"):format(level, altitudeSensorHeight, altitudeSensorPressure)
    status.foreground = colors.lime
  else
    status.text = "Level captured in memory, but could not save airship-hover.cfg."; status.foreground = colors.red
  end
end
local function finishBurnerSweep(message)
  local sweep = burnerSweep
  burnerSweep, manualBurner = nil, nil
  if position then altitude = math.floor(position.y + .5) end
  setFlightMode("HOLDING ALTITUDE")
  sweepSummary = (message and (message .. " ") or "Sweep finished. ") .. ("Saved %d/11; skipped %d."):format(sweep.saved, sweep.skipped)
  sweepSummaryUntil = os.clock() + 15
  status.text = sweepSummary
  status.foreground = message and colors.red or colors.lime
end
local function startBurnerSweep()
  if type(burnerSweep) == "table" then status.text = "Burner sweep is already running."; return end
  if mode == "STOPPED" then status.text = "Resume flight control before starting the sweep."; status.foreground = colors.red; return end
  if not position or not altitudeSensorReady or not altitudeSensorPressure then
    status.text = "Sweep needs GPS and Avionics altitude/pressure."; status.foreground = colors.red; return
  end
  if sweepConfirmAt and os.clock() - sweepConfirmAt < 8 then
    sweepSummary = nil
    burnerSweep = {level=15, changedAt=os.clock(), stableSince=nil, saved=0, skipped=0}
    manualBurner, vectorX, vectorY, power, target = nil, 0, 0, 0, nil
    mode = "BURNER SWEEP"
    status.text = "Sweep started: levels 15 down to 5"; status.foreground = colors.orange
    sweepConfirmAt = nil
  else
    sweepConfirmAt = os.clock()
    status.text = "Sweep armed. Confirm again within 8 seconds."; status.foreground = colors.orange
  end
end
local function updateBurnerSweep()
  if type(burnerSweep) ~= "table" then return end
  local now = os.clock()
  if not position or not altitudeSensorPressure then finishBurnerSweep("Sweep aborted: GPS or pressure reading lost."); return end
  local atLevel = math.abs(burnerCommand - burnerSweep.level) < .05
  if atLevel and math.abs(velocity.y) < .12 then burnerSweep.stableSince = burnerSweep.stableSince or now
  else burnerSweep.stableSince = nil end
  if burnerSweep.stableSince and now - burnerSweep.stableSince >= 4 then
    local pressure = tonumber(("%.4f"):format(altitudeSensorPressure))
    local key = ("%.3f"):format(altitudeSensorHeight)
    learnedHover[key] = {pressure=pressure, level=burnerSweep.level, height=altitudeSensorHeight}
    if not saveHoverMap() then finishBurnerSweep("Sweep stopped: calibration file could not be saved."); return end
    burnerSweep.saved = burnerSweep.saved + 1
    if burnerSweep.level == 5 then finishBurnerSweep(); return end
    burnerSweep.level = burnerSweep.level - 1
    burnerSweep.changedAt, burnerSweep.stableSince = now, nil
  elseif now - burnerSweep.changedAt >= 45 then
    burnerSweep.skipped = burnerSweep.skipped + 1
    if burnerSweep.level == 5 then finishBurnerSweep(); return end
    burnerSweep.level = burnerSweep.level - 1
    burnerSweep.changedAt, burnerSweep.stableSince = now, nil
  end
  status.text = ("SWEEP %d/15 | saved %d, skipped %d"):format(burnerSweep.level, burnerSweep.saved, burnerSweep.skipped)
  status.foreground = colors.orange
end
local function toggleManualBurner()
  if type(burnerSweep) == "table" then status.text = "Finish or stop the burner sweep first."; return end
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

local function addGpuButton(x, y, w, h, label, color, action)
  gpuButtons[#gpuButtons + 1] = {x=x, y=y, w=w, h=h, label=label, color=color, action=action}
end
local function renderDirectMonitor()
  if not directGpu then return end
  local gpu, display = directGpu, directDisplay
  local w, h = directWidth, directHeight
  local function rect(x, y, rw, rh, color) gpu.fillRect(display, x, y, rw, rh, color[1], color[2], color[3]) end
  local function txt(value, x, y, size, color, style)
    gpu.drawText(display, tostring(value), x, y, color[1], color[2], color[3], "Arial", size, style or "plain")
  end
  local bg, panel, ink, muted = {10,17,29}, {20,32,48}, {229,240,250}, {143,164,184}
  savedGpuButtons = {}
  if h >= 320 then
    local heights = savedAltitudes()
    local buttonGap, buttonY = 6, 168
    local buttonW = math.floor((w - 32 - 3 * buttonGap) / 4)
    for i = 1, math.min(4, #heights) do
      local index = i
      local height = heights[i]
      savedGpuButtons[#savedGpuButtons + 1] = {
        x=16 + (i - 1) * (buttonW + buttonGap), y=buttonY, w=buttonW, h=26,
        label=("Y %.0f"):format(height), color={35,62,91}, action=function() selectSavedAltitude(index) end,
      }
    end
  end
  gpu.clear(display, bg[1], bg[2], bg[3])
  rect(0, 0, w, 42, {15,29,47})
  txt("A I R S H I P", 16, 10, 22, ink, "bold")
  txt(mode == "BURNER SWEEP" and "CALIBRATING" or mode, math.floor(w * .47), 14, 15, mode == "BURNER SWEEP" and {255,190,90} or {88,220,190}, "bold")
  local gap, cardW = 12, math.floor((w - 36) / 3)
  local cardY, cardH = 54, 76
  for i=0,2 do rect(12 + i * (cardW + gap), cardY, cardW, cardH, panel) end
  txt("ALTITUDE", 23, cardY + 10, 12, muted, "bold")
  txt(position and ("%.1f m"):format(position.y) or "--", 23, cardY + 33, 27, ink, "bold")
  local mappedLevel
  for _, sample in pairs(learnedHover) do if sample.height and altitude and math.abs(sample.height-altitude) < .01 then mappedLevel = sample.level; break end end
  txt(("TARGET %s  |  BURNER %s"):format(altitude and ("%.0f"):format(altitude) or "--", mappedLevel and tostring(mappedLevel) or "--"), 23, cardY + 60, 12, muted)
  txt("VERTICAL SPEED", 23 + cardW + gap, cardY + 10, 12, muted, "bold")
  txt(("%.2f"):format(velocity.y), 23 + cardW + gap, cardY + 33, 27, ink, "bold")
  txt("blocks / sec", 23 + cardW + gap, cardY + 60, 12, muted)
  local speedX, speedY, speedW = 23 + cardW + gap, cardY + 75, cardW - 22
  rect(speedX, speedY, speedW, 6, {57,73,91})
  local speedRatio = math.max(-1, math.min(1, velocity.y / 1.5))
  local centerX = speedX + math.floor(speedW / 2)
  rect(centerX, speedY, 2, 6, {143,164,184})
  if speedRatio ~= 0 then
    local barW = math.max(2, math.floor(math.abs(speedRatio) * speedW / 2))
    rect(speedRatio > 0 and centerX or centerX - barW, speedY, barW, 6, speedRatio > 0 and {55,205,160} or {255,190,90})
  end
  txt("BURNER", 23 + 2 * (cardW + gap), cardY + 10, 12, muted, "bold")
  txt(("%d / 15"):format(burner), 23 + 2 * (cardW + gap), cardY + 33, 27, {255,190,90}, "bold")
  txt(("THRUST  %.1f / 15"):format(measuredThrust), 23 + 2 * (cardW + gap), cardY + 60, 12, muted)
  txt(position and ("GPS  X %.1f     Z %.1f"):format(position.x, position.z) or "GPS SIGNAL LOST", 16, 145, 15, position and ink or {255,110,110})
  local baseY = math.floor(h * .62)
  txt("HOVER CALIBRATION   15  →  5", 16, baseY, 14, muted, "bold")
  for i=0,10 do
    local level = 15 - i
    local x = 16 + i * (math.floor((w - 32) / 11))
    local marked = false
    for _, sample in pairs(learnedHover) do if sample.level == level then marked = true; break end end
    local active = burnerSweep and burnerSweep.level == level
    rect(x, baseY + 24, math.max(12, math.floor((w - 48) / 13)), 12, active and {255,180,72} or marked and {55,205,160} or {57,73,91})
    txt(level, x + 2, baseY + 42, 10, marked and {115,235,195} or muted)
  end
  if burnerSweep then
    txt(("LEVEL %d   SAVED %d/11   SKIPPED %d"):format(burnerSweep.level, burnerSweep.saved, burnerSweep.skipped), 16, baseY + 62, 13, {255,190,90}, "bold")
  elseif sweepSummary then txt(sweepSummary, 16, baseY + 62, 12, {255,190,90})
  else txt("Tap SWEEP twice to confirm. Levels save as soon as stable.", 16, baseY + 62, 12, muted) end
  local learnedCount = 0
  for _ in pairs(learnedHover) do learnedCount = learnedCount + 1 end
  txt(("GPS %s   AVIONICS %s   SAVED %d"):format(position and "OK" or "LOST", altitudeSensorReady and "OK" or "MISSING", learnedCount), 16, baseY + 80, 12, muted)
  for _, b in ipairs(savedGpuButtons) do
    rect(b.x, b.y, b.w, b.h, b.color)
    txt(b.label, b.x + 8, b.y + math.floor((b.h - 16) / 2), 12, ink, "bold")
  end
  for _, b in ipairs(gpuButtons) do
    rect(b.x, b.y, b.w, b.h, b.color)
    txt(b.label, b.x + math.max(4, math.floor((b.w - #b.label * 8) / 2)), b.y + math.floor((b.h - 18) / 2), 14, ink, "bold")
  end
  gpu.updateDisplay(display)
end
local monitorStatus, monitorPos, monitorAlt, monitorSystem, monitorCalibrateButton, monitorGpuNotice, compactGpuNotice
local showGainValues
local lastGpuRender = 0
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

local gpuButtonsReady = false
local function createGpuButtons()
  if not directGpu or gpuButtonsReady then return end
  gpuButtonsReady = true
  local y, h, gap = directHeight - 48, 36, 6
  local x, bw = 12, math.floor((directWidth - 24 - 9 * gap) / 10)
  addGpuButton(x, y, bw, h, "ALT -", {35,62,91}, function() adjustAltitude(-1) end)
  addGpuButton(x+(bw+gap), y, bw, h, "ALT +", {35,62,91}, function() adjustAltitude(1) end)
  addGpuButton(x+2*(bw+gap), y, bw, h, "SAVED", {43,90,132}, selectAltitudePreset)
  addGpuButton(x+3*(bw+gap), y, bw, h, "HOLD", {35,62,91}, holdHere)
  addGpuButton(x+4*(bw+gap), y, bw, h, "AUTO", {42,112,83}, engageAutopilot)
  addGpuButton(x+5*(bw+gap), y, bw, h, "SWEEP", {130,78,31}, startBurnerSweep)
  addGpuButton(x+6*(bw+gap), y, bw, h, "MAN", {100,75,35}, toggleManualBurner)
  addGpuButton(x+7*(bw+gap), y, bw, h, "LOCK", {100,75,35}, lockHoverLevel)
  addGpuButton(x+8*(bw+gap), y, bw, h, "UPD", {46,107,130}, requestProgramUpdate)
  addGpuButton(x+9*(bw+gap), y, bw, h, "STOP", {125,48,56}, stop)
end
createGpuButtons()

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
  monitorFrame:addButton({x=18, y=10, width=12, height=1, text="SAVED ALT", background=colors.blue}):onClick(selectAltitudePreset)
  monitorFrame:addButton({x=32, y=10, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
  monitorFrame:addButton({x=2, y=11, width=16, height=1, text="SWEEP 15-5", background=colors.orange}):onClick(startBurnerSweep)
  monitorFrame:addButton({x=20, y=11, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
  if directGpuError then monitorGpuNotice = monitorFrame:addLabel({x=2, y=12, width=width-2, height=1, text="GPU: " .. directGpuError, foreground=colors.orange}) end
end

local flightPage, calibrationPage
local savedAltitudeButtons = {}
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
  frame:addButton({x=16, y=11, width=10, height=1, text="SWEEP V", background=colors.orange}):onClick(startBurnerSweep)
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
  if directGpuError then compactGpuNotice = frame:addLabel({x=2, y=13, width=screenWidth-3, height=2, text="DirectGPU: " .. directGpuError, foreground=colors.orange}) end
  frame:addButton({x=screenWidth-12, y=15, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
else
frame:addLabel({x=2, y=1, width=30, height=1, text="AIRSHIP  /  FLIGHT DECK", foreground=colors.white, background=colors.blue})
status = frame:addLabel({x=2, y=2, width=screenWidth-3, height=1, text="Starting controller...", foreground=colors.yellow})
frame:addButton({x=screenWidth-12, y=1, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
local tabs = frame:addTabControl({x=1, y=3, width=screenWidth, height=screenHeight-2})
flightPage = tabs:addTab("FLIGHT")
calibrationPage = tabs:addTab("CALIBRATION")

local function card(parent, x, width, title)
  local box = parent:addFrame({x=x, y=1, width=width, height=4, background=colors.gray})
  box:addLabel({x=2, y=1, width=width-2, height=1, text=title, foreground=colors.lightBlue, background=colors.gray})
  return box
end
local gpsCard = card(flightPage, 2, 16, "GPS POSITION")
local altitudeCard = card(flightPage, 19, 15, "ALTITUDE")
local systemCard = card(flightPage, 35, 15, "FLIGHT SYSTEM")
posText = gpsCard:addLabel({x=2, y=2, width=13, height=3, text="X --\nY --\nZ --", foreground=colors.white, background=colors.gray})
altText = altitudeCard:addLabel({x=2, y=2, width=12, height=2, text="Now --\nTarget --", foreground=colors.white, background=colors.gray})
sysText = systemCard:addLabel({x=2, y=2, width=12, height=2, text="Burner 0/15\nThrust 0/15", foreground=colors.white, background=colors.gray})

flightPage:addLabel({x=2, y=6, width=22, height=1, text="DESTINATION  (X Z)", foreground=colors.lightBlue})
xInput = flightPage:addInput({x=2, y=7, width=22, height=1, placeholder="Example: 200 -700", background=colors.white, foreground=colors.black})
local function setDestination()
  local xText, zText = xInput.text:match("^%s*(%S+)%s+(%S+)%s*$")
  local x, z = tonumber(xText), tonumber(zText)
  if not x or not z then status.text = "Enter numeric X and Z coordinates first."; status.foreground = colors.red; return end
  target = {x=x, z=z}
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Destination set: X %.0f, Z %.0f"):format(x, z)
  status.foreground = colors.lime
  flightPage:setFocused(nil)
end
flightPage:addButton({x=26, y=7, width=12, height=1, text="SET DEST", background=colors.blue}):onClick(setDestination)
xInput:onEnter(setDestination)

flightPage:addLabel({x=2, y=9, width=16, height=1, text="TARGET ALTITUDE Y", foreground=colors.lightBlue})
heightInput = flightPage:addInput({x=2, y=10, width=12, height=1, placeholder="Height", background=colors.white, foreground=colors.black})
local function setHeight()
  local value = tonumber(heightInput.text)
  if not value then status.text = "Enter a numeric height target."; status.foreground = colors.red; return end
  altitude = value
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Height target set to Y %.0f"):format(value); status.foreground = colors.lime
  flightPage:setFocused(nil)
end
flightPage:addButton({x=16, y=10, width=12, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
heightInput:onEnter(setHeight)
flightPage:addButton({x=30, y=10, width=13, height=1, text="SAVED ALT", background=colors.blue}):onClick(selectAltitudePreset)
for i=1,4 do
  local index = i
  local x, y = index % 2 == 1 and 2 or 25, index <= 2 and 12 or 13
  savedAltitudeButtons[index] = flightPage:addButton({x=x, y=y, width=22, height=1, text="NO SAVED ALTITUDE", background=colors.gray}):onClick(function() selectSavedAltitude(index) end)
end
local function flightButton(x, width, label, color, action)
  return flightPage:addButton({x=x, y=14, width=width, height=2, text=label, background=color}):onClick(action)
end
flightButton(2, 7, "ALT -", colors.orange, function() adjustAltitude(-1) end)
flightButton(10, 7, "ALT +", colors.orange, function() adjustAltitude(1) end)
flightButton(18, 9, "HOLD", colors.blue, holdHere)
flightButton(28, 12, "AUTOPILOT", colors.lime, engageAutopilot)
flightButton(41, 9, "STOP", colors.red, stop)

pidSummary = calibrationPage:addLabel({x=2, y=1, width=47, height=1, text="PID  loading gains...", foreground=colors.lightGray})
calibrationPage:addLabel({x=2, y=3, width=47, height=1, text="PID TUNE: X/Z P I D   ALT bias P I D", foreground=colors.lightBlue})
pidInput = calibrationPage:addInput({x=2, y=4, width=35, height=1, placeholder="0.035 0.0005 0.18 4 1 0.03 2", background=colors.white, foreground=colors.black})
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
  calibrationPage:setFocused(nil)
  status.text = "PID gains applied and saved."; status.foreground = colors.lime
end
calibrationPage:addButton({x=39, y=4, width=10, height=1, text="SET PID", background=colors.blue}):onClick(setGains)
pidInput:onEnter(setGains)

local function calibrationButton(x, y, width, label, color, action)
  return calibrationPage:addButton({x=x, y=y, width=width, height=2, text=label, background=color}):onClick(action)
end
calibrationButton(2, 6, 11, "X- / LEFT", colors.gray, function() testVector(-.2, 0, "TEST X-") end)
calibrationButton(14, 6, 11, "X+ / RIGHT", colors.gray, function() testVector(.2, 0, "TEST X+") end)
calibrationButton(26, 6, 11, "Y+ / FWD", colors.gray, function() testVector(0, .2, "TEST Y+") end)
calibrationButton(38, 6, 11, "Y- / BACK", colors.gray, function() testVector(0, -.2, "TEST Y-") end)
calibrateButton = calibrationPage:addButton({x=2, y=9, width=14, height=1, text=calibrated and "CALIBRATED" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
calibrationPage:addButton({x=18, y=9, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
calibrationPage:addButton({x=32, y=9, width=15, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
calibrationButton(2, 12, 16, "SWEEP 15-5", colors.orange, startBurnerSweep)
calibrationPage:addLabel({x=20, y=12, width=29, height=1, text="Sweep saves each stable altitude", foreground=colors.lightGray})
end

local function isFocused(input)
  for _, parent in ipairs({frame, flightPage, calibrationPage}) do
    if parent then
      local ok, focused = pcall(function() return parent:getFocused() end)
      if ok and focused == input then return true end
    end
  end
  return false
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
  if not isFocused(heightInput) then heightInput.text = altitude and tostring(math.floor(altitude + .5)) or "" end
  if not isFocused(pidInput) then showGainValues() end
  if #savedAltitudeButtons > 0 then
    local heights = savedAltitudes()
    for i, button in ipairs(savedAltitudeButtons) do
      button.text = heights[i] and ("TARGET Y " .. ("%.1f"):format(heights[i])) or "NO SAVED ALTITUDE"
      button.background = heights[i] and colors.blue or colors.gray
    end
  end
  if pidSummary then pidSummary.text = directGpuError and ("GPU: " .. directGpuError) or ("XY %.3f/%.4f/%.2f ALT %.1f/%.1f/%.2f/%.1f"):format(pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d, pid.altitude.bias, pid.altitude.p, pid.altitude.i, pid.altitude.d) end
  if compactGpuNotice then compactGpuNotice.text = directGpuError and ("DirectGPU: " .. directGpuError) or "DirectGPU ready" end
  if monitorGpuNotice then monitorGpuNotice.text = directGpuError and ("GPU: " .. directGpuError) or "GPU ready" end
  if mode == "AUTOPILOT" then status.text, status.foreground = "AUTOPILOT ACTIVE  |  EAST-FACING SHIP", colors.lime
  elseif mode == "GPS LOST - RE-ENGAGE" then status.text, status.foreground = mode, colors.red
  elseif mode == "WAITING FOR GPS" then status.text, status.foreground = "Waiting for GPS signal...", colors.orange
  elseif mode == "HOLDING ALTITUDE" then status.text, status.foreground = "ALTITUDE HOLD ACTIVE", colors.lightBlue
  elseif mode == "DESTINATION REACHED" then status.text, status.foreground = "DESTINATION REACHED  |  HOLDING ALTITUDE", colors.lime
  elseif mode == "SET ALTITUDE" then status.text, status.foreground = "GPS ready  |  set an altitude target", colors.orange
  elseif mode == "CALIBRATION CONFIRM" then status.text, status.foreground = "Confirm calibration within 8 seconds", colors.orange
  elseif mode == "BURNER SWEEP" and burnerSweep then status.text, status.foreground = ("SWEEP %d/15 | saved %d, skipped %d | press S to stop"):format(burnerSweep.level, burnerSweep.saved, burnerSweep.skipped), colors.orange
  end
  if mode == "MANUAL BURNER" then status.text, status.foreground = ("MANUAL BURNER %d/15 | UP/DOWN adjust, K locks"):format(manualBurner or 0), colors.orange
  elseif mode:match("^TEST ") then status.text, status.foreground = mode .. "  |  press HOLD to stop", colors.orange
  elseif mode == "STOPPED" then status.text, status.foreground = "OUTPUTS STOPPED", colors.red end
  if sweepSummary and os.clock() < sweepSummaryUntil then status.text, status.foreground = sweepSummary, colors.orange
  elseif sweepSummary then sweepSummary = nil end
  if monitorFrame then
    monitorPos.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    monitorAlt.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
    local savedCount = 0
    for _ in pairs(learnedHover) do savedCount = savedCount + 1 end
    monitorSystem.text = ("B%d/15 T%.1f GPS %s AV %s H:%d"):format(burner, measuredThrust, position and "OK" or "LOST", altitudeSensorReady and "OK" or "--", savedCount)
    monitorStatus.text, monitorStatus.foreground = status.text, status.foreground
  end
  if directGpu and os.clock() - lastGpuRender >= .25 then
    local ok = pcall(renderDirectMonitor)
    if ok then lastGpuRender = os.clock()
    else directGpu = nil; directGpuError = "DirectGPU connection lost; reconnecting" end
  end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(.1, false)
  altitude = startY and math.floor(startY + .5) or nil
  if altitude then mode = "HOLDING ALTITUDE" else mode = "WAITING FOR GPS" end
  basalt.schedule(function()
    while running do
      local success, failure = pcall(function()
        local gpsFresh = locate()
        updateFlight(gpsFresh)
        updateBurnerSweep()
        outputs()
      end)
      if not success then
        local sweep = burnerSweep
        stop()
        sweepSummary = "Controller error: " .. tostring(failure) .. (sweep and (" Sweep saved %d/11; skipped %d."):format(sweep.saved, sweep.skipped) or "")
        sweepSummaryUntil = os.clock() + 15
        running = false; status.text = sweepSummary; status.foreground = colors.red; refresh(); basalt.stop(); return
      end
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
      if directGpu then
        local alive, info = pcall(directGpu.getDisplayInfo, directDisplay)
        if not alive or not info then directGpu = nil; directGpuError = "DirectGPU connection lost; reconnecting" end
      end
      if not directGpu then
        pcall(setupDirectGpu)
        createGpuButtons()
        if not monitorGpuNotice and monitorFrame then
          monitorGpuNotice = monitorFrame:addLabel({x=2, y=12, width=monitor.getSize()-2, height=1, text="GPU: " .. (directGpuError or "ready"), foreground=colors.orange})
        end
        if not compactGpuNotice and compact then
          compactGpuNotice = frame:addLabel({x=2, y=13, width=screenWidth-3, height=2, text="DirectGPU: " .. (directGpuError or "ready"), foreground=colors.orange})
        end
      end
      if directGpu then
        local eventOk, hasEvents = pcall(directGpu.hasEvents, directDisplay)
        if not eventOk then directGpu = nil; directGpuError = "DirectGPU connection lost; reconnecting"
        elseif hasEvents then
          local pollOk, event = pcall(directGpu.pollEvent, directDisplay)
          if not pollOk then directGpu = nil; directGpuError = "DirectGPU connection lost; reconnecting" end
          if event and event.type == "mouse_click" then
            for _, button in ipairs(gpuButtons) do
              if event.x >= button.x and event.x <= button.x + button.w and event.y >= button.y and event.y <= button.y + button.h then
                button.action()
                if button.label == "SWEEP" then sweepSummary = nil end
                break
              end
            end
            for _, button in ipairs(savedGpuButtons) do
              if event.x >= button.x and event.x <= button.x + button.w and event.y >= button.y and event.y <= button.y + button.h then
                button.action()
                break
              end
            end
          end
        end
      end
      sleep(directGpu and .05 or 2)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, key = os.pullEvent("key")
      local typing = isFocused(xInput) or isFocused(heightInput) or isFocused(pidInput)
      if typing then
        if key == keys.escape then
          frame:setFocused(nil)
          if flightPage then flightPage:setFocused(nil) end
          if calibrationPage then calibrationPage:setFocused(nil) end
        end
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
        elseif key == keys.v then startBurnerSweep()
        elseif key == keys.u then requestProgramUpdate()
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
if updateRequested then
  local ok, err = pcall(shell.openTab, "/airship-updater.lua")
  if not ok then term.setTextColor(colors.red); print("Could not open updater: " .. tostring(err)) end
end
if not ok then term.setTextColor(colors.red); print("Airship UI stopped: " .. tostring(err)) end
