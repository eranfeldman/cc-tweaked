-- Basalt 2.5 flight panel for the east-facing eight-thruster ship.
local AIRSHIP_BUILD = "1.1.5"
local basalt = require("basalt")

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
local findAvionics
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
-- Resolve devices behind the Malahit hub nested behind the wired peripheral hub.
findAvionics = function(kind)
  local direct = peripheral.find(kind)
  if direct then return direct end
  for _, rootName in ipairs(peripheral.getNames()) do
    local root = peripheral.wrap(rootName)
    if root and type(root.getNamesRemote) == "function" then
      local ok, names = pcall(root.getNamesRemote)
      if ok and type(names) == "table" then
        for _, remoteName in ipairs(names) do
          local typeOk, remoteType = pcall(root.getTypeRemote, remoteName)
          local types = type(remoteType) == "table" and remoteType or {remoteType}
          for _, remoteKind in ipairs(types) do
            if remoteKind == kind then
              return setmetatable({}, {__index=function(_, method)
                return function(...) return root.callRemote(remoteName, method, ...) end
              end})
            elseif remoteKind == "malahit_hub" or remoteKind == "peripheral_hub" then
              local childOk, childNames = pcall(root.callRemote, remoteName, "getNamesRemote")
              if childOk and type(childNames) == "table" then
                for _, childName in ipairs(childNames) do
                  local childTypeOk, childType = pcall(root.callRemote, remoteName, "getTypeRemote", childName)
                  local childTypes = type(childType) == "table" and childType or {childType}
                  for _, childKind in ipairs(childTypes) do
                    if childKind == kind then
                      return setmetatable({}, {__index=function(_, method)
                        return function(...) return root.callRemote(remoteName, "callRemote", childName, method, ...) end
                      end})
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
  end
end
monitor = monitor or findAvionics("monitor")
local function findThrusters()
  local found = {}
  local function add(name, device)
    local id = name:match("([^/]+)$") or name
    if device and not found[id] then found[id] = device end
  end
  for _, name in ipairs(peripheral.getNames()) do
    local kinds = {peripheral.getType(name)}
    for _, kind in ipairs(kinds) do
      if kind == "liquid_vector_thruster" then
        add(name, peripheral.wrap(name))
      end
    end
  end
  local count = 0
  for _ in pairs(found) do count = count + 1 end
  if count >= 8 then return found end
  for _, name in ipairs(peripheral.getNames()) do
    local hub = peripheral.wrap(name)
    if hub and type(hub.getNamesRemote) == "function" then
      local function scan(remoteHub, path, depth)
        if depth > 2 then return end
        local ok, names = pcall(remoteHub.getNamesRemote)
        if not ok or type(names) ~= "table" then return end
        for _, childName in ipairs(names) do
          local typeOk, remoteType = pcall(remoteHub.getTypeRemote, childName)
          local types = type(remoteType) == "table" and remoteType or {remoteType}
          local child = setmetatable({}, {__index=function(_, method)
            return function(...) return remoteHub.callRemote(childName, method, ...) end
          end})
          for _, kind in ipairs(types) do
            if kind == "liquid_vector_thruster" then
              add(path .. "/" .. childName, child)
            elseif (kind == "peripheral_hub" or kind == "malahit_hub") and depth < 2 then
              scan(child, path .. "/" .. childName, depth + 1)
            end
          end
        end
      end
      scan(hub, name, 1)
    end
  end
  return found
end
local thrusters = findThrusters()
local thrusterCount = 0
for _ in pairs(thrusters) do thrusterCount = thrusterCount + 1 end
if thrusterCount ~= 8 then
  error("Expected 8 liquid_vector_thruster peripherals, found " .. thrusterCount .. "; check the wired hub")
end
local altitudeSensor = findAvionics("altitude_sensor")
local navigationTable = findAvionics("navigation_table")
local gimbalSensor = findAvionics("gimbal_sensor")
setupDirectGpu()
local frame = basalt.getMainFrame()
local monitorFrame = monitor and not directGpu and basalt.createFrame(monitor)
frame.background = colors.black
if monitorFrame then monitorFrame.background = colors.black end
local screenWidth, screenHeight
screenWidth, screenHeight = term.getSize()
local compact = screenWidth < 49 or screenHeight < 19
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner = redstone.getAnalogOutput("front") or 0
local burnerCommand, vectorX, vectorY, power = burner, 0, 0, 0
local burnerSlewPerSecond = 6
local calibrated = fs.exists("airship-calibrated")
local running, manualBurner, burnerSweep, calibrationRunning = true, nil, nil, false
local sweepConfirmAt
local sweepSummary
local sweepSummaryUntil
local gpuButtons = {}
local gpuTab, gpuHeightPage = "FLIGHT", 1
local learnedHover = {}
local altitudeSensorPressure
local shipHeading
local yawRate
local measuredThrust = 0
local status, pidSummary, posText, altText, sysText, xInput, heightInput, pidInput, calibrateButton
-- Conservative starting gains; GPS velocity feedback compensates for variable sample timing.
local pid = {
  horizontalX = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  horizontalZ = {p=.035, i=.0005, d=.18, limit=.25, integral=0}
}
local gainLimits = {{.005,.15}, {0,.01}, {0,.8}}
local function applyGainValues(values)
  for i, bounds in ipairs(gainLimits) do
    if type(values[i]) ~= "number" or values[i] < bounds[1] or values[i] > bounds[2] then return false end
  end
  pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d = values[1], values[2], values[3]
  pid.horizontalZ.p, pid.horizontalZ.i, pid.horizontalZ.d = values[1], values[2], values[3]
  pid.horizontalX.integral, pid.horizontalZ.integral = 0, 0
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
  if #values >= 3 then applyGainValues(values) end
end
local function gainValues()
  return {pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d}
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
local clearHoverConfirmAt
local function clearHoverMap()
  if not clearHoverConfirmAt or os.clock() - clearHoverConfirmAt >= 8 then
    clearHoverConfirmAt = os.clock()
    status.text = "Tap CLEAR again within 8 seconds to erase saved heights."
    status.foreground = colors.orange
    return
  end
  clearHoverConfirmAt = nil
  local previous = learnedHover
  learnedHover = {}
  local ok, err = saveHoverMap()
  if ok then
    gpuHeightPage = 1
    status.text = "All saved calibration heights cleared."
    status.foreground = colors.lime
  else
    learnedHover = previous
    status.text = "Could not clear saved heights: " .. tostring(err)
    status.foreground = colors.red
  end
end
readGainFile()
local velocity = {x=0, y=0, z=0}
local previousPosition, previousSample
local sampleDt = .5
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
  calibrationRunning = false
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
  calibrationRunning = false
  manualBurner = nil
  burnerSweep = nil
  vectorX, vectorY, power, mode = 0, 0, 0, nextMode
  pid.horizontalX.integral, pid.horizontalZ.integral = 0, 0
end
local function locate()
  local now = os.clock()
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
  if navigationTable then
    local ok, heading = pcall(navigationTable.getHeadingRad)
    if ok and type(heading) == "number" and heading == heading and math.abs(heading) < math.huge then shipHeading = heading end
  end
  if gimbalSensor then
    local ok, rates = pcall(gimbalSensor.getAngularRatesRad)
    if ok and type(rates) == "table" and type(rates[2]) == "number" then yawRate = rates[2] end
  end
  local x, y, z = gps.locate(.5, false)
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
  elseif lastGpsAt and os.clock() - lastGpsAt > 2 then
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
  if not altitude then mode = "SET ALTITUDE"; return end
  if burnerSweep then
    burnerCommand = burnerCommand + clamp(burnerSweep.level - burnerCommand, -burnerSlewPerSecond * sampleDt, burnerSlewPerSecond * sampleDt)
    burner = math.floor(burnerCommand + .5)
  elseif manualBurner ~= nil then
    burnerCommand, burner = manualBurner, manualBurner
  else
    local mappedBurner = hoverBias(altitude)
    if mappedBurner then
      burnerCommand = burnerCommand + clamp(mappedBurner - burnerCommand, -burnerSlewPerSecond * sampleDt, burnerSlewPerSecond * sampleDt)
      burner = math.floor(burnerCommand + .5)
    end
  end
  if gpsFresh and mode == "AUTOPILOT" and target then
    local dx, dz = target.x - position.x, target.z - position.z
    if not shipHeading then
      vectorX, vectorY, power = 0, 0, 0
      mode = "HEADING SENSOR MISSING"
      return
    end
    if math.abs(dx) < .5 and math.abs(dz) < .5 and math.abs(velocity.x) < .12 and math.abs(velocity.z) < .12 then
      vectorX, vectorY, power, mode = 0, 0, 0, "DESTINATION REACHED"
    else
      -- Nozzle-vector direction is opposite the craft's thrust direction.
      local c, s = math.cos(shipHeading), math.sin(shipHeading)
      local localX, localZ = c * dx - s * dz, s * dx + c * dz
      local localVX, localVZ = c * velocity.x - s * velocity.z, s * velocity.x + c * velocity.z
      vectorX = -pidStep(pid.horizontalZ, localZ, localVZ, sampleDt)
      vectorY = -pidStep(pid.horizontalX, localX, localVX, sampleDt)
      power = clamp(math.sqrt(vectorX * vectorX + vectorY * vectorY) / .25 * .35, 0, .35)
    end
  end
end

local function savedHoverPoints()
  local points = {}
  for _, sample in pairs(learnedHover) do if type(sample.height) == "number" then points[#points + 1] = sample end end
  table.sort(points, function(a,b) return a.height < b.height end)
  return points
end
local function savedAltitudes()
  local heights = {}
  for _, sample in ipairs(savedHoverPoints()) do heights[#heights + 1] = sample.height end
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

local function gpuLayout()
  local w, h = directWidth, directHeight
  local margin = math.max(12, math.floor(w * .02))
  local gap = math.max(8, math.floor(w * .012))
  local titleH = math.max(40, math.floor(h * .08))
  local tabY, tabH = titleH, math.max(38, math.floor(h * .07))
  local contentY = tabY + tabH + math.max(12, math.floor(h * .025))
  local buttonH = math.max(34, math.floor(h * .07))
  local buttonY = h - margin - buttonH
  return {w=w,h=h,margin=margin,gap=gap,titleH=titleH,tabY=tabY,tabH=tabH,contentY=contentY,buttonH=buttonH,buttonY=buttonY}
end
local function addGpuButton(x, y, w, h, label, color, action, align)
  gpuButtons[#gpuButtons + 1] = {x=x,y=y,w=w,h=h,label=label,color=color,action=action,align=align}
end
local function renderDirectMonitor()
  if not directGpu then return end
  local gpu, display = directGpu, directDisplay
  local l = gpuLayout()
  local w, h = l.w, l.h
  local function rect(x,y,rw,rh,color) gpu.fillRect(display,x,y,rw,rh,color[1],color[2],color[3]) end
  local function txt(value,x,y,size,color,style) gpu.drawText(display,tostring(value),x,y,color[1],color[2],color[3],"Arial",size,style or "plain") end
  local bg,panel,ink,muted = {10,17,29},{20,32,48},{229,240,250},{143,164,184}
  gpu.clear(display,bg[1],bg[2],bg[3])
  rect(0,0,w,l.titleH,{15,29,47})
  local titleSize = math.max(18,math.floor(h*.032))
  txt("AIRSHIP v" .. AIRSHIP_BUILD,l.margin,math.floor((l.titleH-titleSize)/2),titleSize,ink,"bold")
  txt(mode == "BURNER SWEEP" and "CALIBRATING" or mode,math.floor(w*.28),math.floor((l.titleH-titleSize)/2),math.max(14,math.floor(titleSize*.7)),mode == "BURNER SWEEP" and {255,190,90} or {88,220,190},"bold")
  local contentY = l.contentY
  if gpuTab == "FLIGHT" then
    local gap = l.gap
    local cardW = math.floor((w-2*l.margin-2*gap)/3)
    local cardY, cardH = contentY, math.floor(h*.29)
    local xs = {l.margin,l.margin+cardW+gap,l.margin+2*(cardW+gap)}
    for _,x in ipairs(xs) do rect(x,cardY,cardW,cardH,panel) end
    local small,big = math.max(14,math.floor(h*.022)),math.max(28,math.floor(h*.055))
    txt("ALTITUDE",xs[1]+14,cardY+12,small,muted,"bold")
    txt(position and ("%.1f m"):format(position.y) or "--",xs[1]+14,cardY+math.floor(cardH*.34),big,ink,"bold")
    local mappedLevel
    for _,sample in pairs(learnedHover) do if sample.height and altitude and math.abs(sample.height-altitude)<.01 then mappedLevel=sample.level; break end end
    txt(("TARGET %s  |  BURNER %s"):format(altitude and ("%.0f"):format(altitude) or "--",mappedLevel and tostring(mappedLevel) or "--"),xs[1]+14,cardY+math.floor(cardH*.70),small,muted)
    txt("VERTICAL SPEED",xs[2]+14,cardY+12,small,muted,"bold")
    txt(("%.2f"):format(velocity.y),xs[2]+14,cardY+math.floor(cardH*.34),big,ink,"bold")
    txt("blocks / sec",xs[2]+14,cardY+math.floor(cardH*.70),small,muted)
    local speedX,speedY,speedW=xs[2]+14,cardY+math.floor(cardH*.84),cardW-28
    rect(speedX,speedY,speedW,math.max(6,math.floor(h*.01)),{57,73,91})
    local ratio=math.max(-1,math.min(1,velocity.y/1.5)); local center=speedX+math.floor(speedW/2)
    rect(center,speedY,2,math.max(6,math.floor(h*.01)),{143,164,184})
    if ratio~=0 then local bar=math.max(2,math.floor(math.abs(ratio)*speedW/2)); rect(ratio>0 and center or center-bar,speedY,bar,math.max(6,math.floor(h*.01)),ratio>0 and {55,205,160} or {255,190,90}) end
    txt("BURNER",xs[3]+14,cardY+12,small,muted,"bold")
    txt(("%d / 15"):format(burner),xs[3]+14,cardY+math.floor(cardH*.34),big,{255,190,90},"bold")
    txt(("THRUST %.1f / 15"):format(measuredThrust),xs[3]+14,cardY+math.floor(cardH*.70),small,muted)
    local detailY=cardY+cardH+math.max(14,math.floor(h*.09)); local detailH=l.buttonY-detailY-l.margin
    rect(l.margin,detailY,w-2*l.margin,detailH,panel)
    local detailFont=math.max(11,math.min(small,math.floor(detailH*.19)))
    txt("FLIGHT STATUS",l.margin+14,detailY+8,detailFont,muted,"bold")
    txt(position and ("GPS  X %.1f     Y %.1f     Z %.1f"):format(position.x,position.y,position.z) or "GPS SIGNAL LOST",l.margin+14,detailY+math.floor(detailH*.34),detailFont,position and ink or {255,110,110})
    txt(("MODE  %s     AV %s     HDG %s°     YAW %s rad/s"):format(mode,altitudeSensorReady and "READY" or "MISSING",shipHeading and ("%.0f"):format(math.deg(shipHeading)) or "--",yawRate and ("%.2f"):format(yawRate) or "--"),l.margin+14,detailY+math.floor(detailH*.62),detailFont,ink)
    txt(target and ("DESTINATION  X %.0f   Z %.0f"):format(target.x,target.z) or "NO DESTINATION SET",l.margin+14,detailY+math.floor(detailH*.82),detailFont,muted)
  elseif gpuTab == "CALIBRATION" then
    local titleSize=math.max(18,math.floor(h*.032))
    txt("HOVER CALIBRATION BY BURNER LEVEL",l.margin,contentY,titleSize,ink,"bold")
    txt("15  →  5",l.margin,contentY+titleSize+10,math.max(14,math.floor(h*.023)),muted)
    local barY=contentY+titleSize+math.max(34,math.floor(h*.07))
    local barGap=math.max(6,math.floor(w*.008)); local barW=math.floor((w-2*l.margin-10*barGap)/11)
    for i=0,10 do
      local level=15-i; local x=l.margin+i*(barW+barGap); local marked=false
      for _,sample in pairs(learnedHover) do if sample.level==level then marked=true; break end end
      local active=burnerSweep and burnerSweep.level==level
      rect(x,barY,barW,math.max(18,math.floor(h*.045)),active and {255,180,72} or marked and {55,205,160} or {57,73,91})
      txt(level,x+4,barY+math.max(22,math.floor(h*.05)),math.max(12,math.floor(h*.02)),marked and {115,235,195} or muted)
    end
    local saved=0; for _ in pairs(learnedHover) do saved=saved+1 end
    txt(("Saved points: %d     Burner: %d / 15     Vertical speed: %.2f"):format(saved,burner,velocity.y),l.margin,barY+math.floor(h*.11),math.max(15,math.floor(h*.025)),ink)
    if burnerSweep then txt(("SWEEP LEVEL %d   SAVED %d/11   SKIPPED %d"):format(burnerSweep.level,burnerSweep.saved,burnerSweep.skipped),l.margin,barY+math.floor(h*.17),math.max(15,math.floor(h*.025)),{255,190,90},"bold")
    elseif sweepSummary then txt(sweepSummary,l.margin,barY+math.floor(h*.17),math.max(14,math.floor(h*.022)),{255,190,90})
    else txt("SWEEP requires two taps to confirm. Each settled level saves automatically.",l.margin,barY+math.floor(h*.17),math.max(14,math.floor(h*.022)),muted) end
    local detailY=barY+math.floor(h*.25); local detailH=l.buttonY-detailY-l.margin
    rect(l.margin,detailY,w-2*l.margin,detailH,panel)
    local detailFont=math.max(11,math.min(math.floor(h*.023),math.floor(detailH*.2)))
    txt("SENSOR / SWEEP DETAIL",l.margin+14,detailY+8,detailFont,muted,"bold")
    txt(("AVIONICS ALTITUDE  %s     PRESSURE  %s"):format(altitudeSensorHeight and ("%.1f"):format(altitudeSensorHeight) or "--",altitudeSensorPressure and ("%.3f"):format(altitudeSensorPressure) or "--"),l.margin+14,detailY+math.floor(detailH*.38),detailFont,ink)
    local stable=burnerSweep and burnerSweep.stableSince and ("%.1f sec"):format(os.clock()-burnerSweep.stableSince) or "--"
    txt(("CURRENT OUTPUT  %d / 15     SETTLED  %s     GPS  %s"):format(burner,stable,position and "OK" or "LOST"),l.margin+14,detailY+math.floor(detailH*.68),detailFont,ink)
  else
    local heights=savedHoverPoints()
    local rowH=math.max(32,math.floor(h*.052)); local rowGap=math.max(4,math.floor(h*.008))
    local rows=math.max(1,math.floor((l.buttonY-contentY-math.max(32,math.floor(h*.05))-l.margin)/(rowH+rowGap)))
    local pages=math.max(1,math.ceil(#heights/rows)); gpuHeightPage=math.max(1,math.min(gpuHeightPage,pages))
    txt(("SAVED ALTITUDES   %d POINTS   PAGE %d / %d"):format(#heights,gpuHeightPage,pages),l.margin,contentY,math.max(18,math.floor(h*.032)),ink,"bold")
    if #heights==0 then txt("No saved calibration heights yet.",l.margin,contentY+math.floor(h*.09),math.max(16,math.floor(h*.026)),muted) end
  end
  for _,b in ipairs(gpuButtons) do
    rect(b.x,b.y,b.w,b.h,b.color)
    local textSize=math.max(13,math.floor(h*.022))
    local tx=b.align=="left" and b.x+12 or b.x+math.max(6,math.floor((b.w-#b.label*textSize*.55)/2))
    txt(b.label,tx,b.y+math.floor((b.h-textSize)/2),textSize,ink,"bold")
  end
  gpu.updateDisplay(display)
end
local monitorStatus, monitorPos, monitorAlt, monitorSystem, monitorCalibrateButton, monitorGpuNotice, compactGpuNotice
local showGainValues
local lastGpuRender = 0
local lastGpuCheck = 0
local lastThrustRead = 0
local function adjustAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  altitude = (altitude or math.floor(position.y + .5)) + delta
  setFlightMode("HOLDING ALTITUDE")
end
local function stepSavedAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  local heights = savedAltitudes()
  if #heights == 0 then adjustAltitude(delta); return end
  local current = altitude or position.y
  local index
  if delta > 0 then
    for i, height in ipairs(heights) do if height > current + .001 then index = i; break end end
    index = index or 1
  else
    for i = #heights, 1, -1 do if heights[i] < current - .001 then index = i; break end end
    index = index or #heights
  end
  selectSavedAltitude(index)
end
local function holdHere()
  if position then
    local canceledCalibration = calibrationRunning
    altitude = math.floor(position.y + .5); target = nil; setFlightMode("HOLDING ALTITUDE")
    if canceledCalibration then status.text = "Direction calibration canceled; holding altitude."; status.foreground = colors.orange end
  end
end
local function engageAutopilot()
  if calibrationRunning then status.text = "Wait for direction calibration to finish."; status.foreground = colors.orange; return end
  if not navigationTable or not shipHeading then status.text = "Navigation table heading unavailable; autopilot cannot steer while ship rotates."; status.foreground = colors.red; return end
  if not calibrated then status.text = "Run low-power direction calibration first."; status.foreground = colors.orange
  elseif not position or not target then status.text = "GPS fix and destination are required."; status.foreground = colors.red
  else setFlightMode("AUTOPILOT"); status.text = "Autopilot engaged"; status.foreground = colors.lime end
end
local function testVector(x, y, label)
  calibrationRunning = false
  vectorX, vectorY, power, mode = -x, -y, .33, label
  status.text = "Low-power test: " .. label .. "  |  Press HOLD to stop"
  status.foreground = colors.orange
end
local function confirmCalibration()
  if calibrationRunning then status.text = "Direction calibration is already running."; return end
  if mode == "STOPPED" then status.text = "Resume flight control before calibrating."; status.foreground = colors.red; return end
  if not position then status.text = "Wait for GPS before calibrating."; status.foreground = colors.red; return end
  calibrationRunning = true
  local tests = {{.2,0,"X-"},{-.2,0,"X+"},{0,-.2,"Y+"},{0,.2,"Y-"}}
  basalt.schedule(function()
    for i, test in ipairs(tests) do
      if not calibrationRunning then return end
      vectorX, vectorY, power, mode = test[1], test[2], .33, "CAL " .. test[3]
      status.text = ("Direction check %d/4: %s; press HOLD to cancel."):format(i, test[3])
      status.foreground = colors.orange
      sleep(.75)
      if not calibrationRunning then return end
      vectorX, vectorY, power = 0, 0, 0
      sleep(.35)
    end
    if not calibrationRunning then return end
    local file = fs.open("airship-calibrated", "w")
    if not file then
      calibrationRunning = false
      status.text = "Tests finished, but calibration could not be saved."; status.foreground = colors.red
      return
    end
    file.write("east front, X right, Y forward")
    file.close()
    calibrationRunning, calibrated = false, true
    vectorX, vectorY, power, mode = 0, 0, 0, "HOLDING ALTITUDE"
    if calibrateButton then calibrateButton.text, calibrateButton.background = "RECALIBRATE", colors.green end
    if monitorCalibrateButton then monitorCalibrateButton.text, monitorCalibrateButton.background = "RECALIBRATE", colors.green end
    status.text = "Four low-power direction checks finished. Calibration saved."; status.foreground = colors.lime
  end)
end

local function createGpuButtons()
  if not directGpu then return end
  gpuButtons = {}
  local l=gpuLayout()
  local tabs={"FLIGHT","CALIBRATION","HEIGHTS"}
  local tabW=math.floor((l.w-2*l.margin-2*l.gap)/3)
  for i,tab in ipairs(tabs) do
    local selected=tab
    addGpuButton(l.margin+(i-1)*(tabW+l.gap),l.tabY,tabW,l.tabH,tab,tab==gpuTab and {43,90,132} or {28,43,60},function() gpuTab=selected end)
  end
  local function buttonRow(items,y)
    local gap=math.max(5,math.floor(l.w*.006)); local bw=math.floor((l.w-2*l.margin-(#items-1)*gap)/#items)
    for i,item in ipairs(items) do addGpuButton(l.margin+(i-1)*(bw+gap),y,bw,l.buttonH,item[1],item[3],item[4]) end
  end
  if gpuTab=="FLIGHT" then
    buttonRow({
      {"ALT -",nil,{35,62,91},function() stepSavedAltitude(-1) end},
      {"ALT +",nil,{35,62,91},function() stepSavedAltitude(1) end},
      {"SAVED",nil,{43,90,132},selectAltitudePreset},
      {"HOLD",nil,{35,62,91},holdHere},
      {"AUTO",nil,{42,112,83},engageAutopilot},
      {"STOP",nil,{125,48,56},stop},
    },l.buttonY)
  elseif gpuTab=="CALIBRATION" then
    buttonRow({
      {"FWD",nil,{70,82,96},function() testVector(0,.2,"TEST Y+") end},
      {"BACK",nil,{70,82,96},function() testVector(0,-.2,"TEST Y-") end},
      {calibrated and "RECAL" or "CAL",nil,calibrated and {55,130,90} or {130,78,31},confirmCalibration},
      {"MANUAL",nil,{100,75,35},toggleManualBurner},
      {"LOCK",nil,{100,75,35},lockHoverLevel},
      {"SWEEP 15-5",nil,{130,78,31},startBurnerSweep},
      {"UPD",nil,{46,107,130},requestProgramUpdate},
      {"STOP",nil,{125,48,56},stop},
      {clearHoverConfirmAt and os.clock()-clearHoverConfirmAt<8 and "CLEAR?" or "CLEAR",nil,{125,48,56},clearHoverMap},
    },l.buttonY)
  else
    local heights=savedHoverPoints()
    local rowH=math.max(32,math.floor(l.h*.052)); local rowGap=math.max(4,math.floor(l.h*.008))
    local topGap=math.max(32,math.floor(l.h*.05))
    local rows=math.max(1,math.floor((l.buttonY-l.contentY-topGap-l.margin)/(rowH+rowGap)))
    local pages=math.max(1,math.ceil(#heights/rows)); gpuHeightPage=math.max(1,math.min(gpuHeightPage,pages))
    local first=(gpuHeightPage-1)*rows+1
    for i=first,math.min(#heights,first+rows-1) do
      local sample=heights[i]; local y=l.contentY+topGap+(i-first)*(rowH+rowGap); local index=i
      addGpuButton(l.margin,y,l.w-2*l.margin,rowH,("Y %.1f     BURNER %d / 15     PRESSURE %.3f"):format(sample.height,sample.level,sample.pressure or 0),{20,32,48},function() selectSavedAltitude(index); gpuTab="FLIGHT" end,"left")
    end
    local navW=math.floor((l.w-2*l.margin-l.gap)/2)
    addGpuButton(l.margin,l.buttonY,navW,l.buttonH,"PREVIOUS",gpuHeightPage>1 and {35,62,91} or {40,48,57},function() gpuHeightPage=math.max(1,gpuHeightPage-1) end)
    addGpuButton(l.margin+navW+l.gap,l.buttonY,navW,l.buttonH,"NEXT",gpuHeightPage<pages and {35,62,91} or {40,48,57},function() gpuHeightPage=math.min(pages,gpuHeightPage+1) end)
  end
end
createGpuButtons()

if monitorFrame then
  local width = monitor.getSize()
  monitorFrame:addLabel({x=2, y=1, width=width-2, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
  monitorStatus = monitorFrame:addLabel({x=2, y=2, width=width-2, height=1, text="Starting...", foreground=colors.yellow})
  monitorPos = monitorFrame:addLabel({x=2, y=3, width=width-2, height=1, text="GPS --", foreground=colors.white})
  monitorAlt = monitorFrame:addLabel({x=2, y=4, width=width-2, height=1, text="ALT -- / TARGET --", foreground=colors.lightBlue})
  monitorSystem = monitorFrame:addLabel({x=2, y=5, width=width-2, height=1, text="BURNER 0/15  THRUST 0/15", foreground=colors.lightGray})
  monitorFrame:addButton({x=2, y=6, width=8, height=1, text="ALT -", background=colors.orange}):onClick(function() stepSavedAltitude(-1) end)
  monitorFrame:addButton({x=11, y=6, width=8, height=1, text="ALT +", background=colors.orange}):onClick(function() stepSavedAltitude(1) end)
  monitorFrame:addButton({x=2, y=7, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  monitorFrame:addButton({x=13, y=7, width=11, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  monitorFrame:addButton({x=25, y=7, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  monitorFrame:addButton({x=2, y=8, width=12, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  monitorFrame:addButton({x=15, y=8, width=12, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  monitorCalibrateButton = monitorFrame:addButton({x=2, y=9, width=14, height=1, text=calibrated and "RECALIBRATE" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  monitorFrame:addLabel({x=17, y=9, width=width-17, height=1, text="type on PC", foreground=colors.lightGray})
  monitorFrame:addButton({x=2, y=10, width=14, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  monitorFrame:addButton({x=18, y=10, width=12, height=1, text="SAVED ALT", background=colors.blue}):onClick(selectAltitudePreset)
  monitorFrame:addButton({x=32, y=10, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
  monitorFrame:addButton({x=2, y=11, width=16, height=1, text="SWEEP 15-5", background=colors.orange}):onClick(startBurnerSweep)
  monitorFrame:addButton({x=20, y=11, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
  monitorFrame:addButton({x=32, y=11, width=12, height=1, text="CLEAR MAP", background=colors.red}):onClick(clearHoverMap)
  if directGpuError then monitorGpuNotice = monitorFrame:addLabel({x=2, y=12, width=width-2, height=1, text="GPU: " .. directGpuError, foreground=colors.orange}) end
end

local flightPage, calibrationPage
local savedAltitudeButtons = {}
if compact then
  frame:addLabel({x=2, y=1, width=screenWidth-2, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
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
    status.text = hoverBias(value) and ("Height target Y %.1f uses calibrated burner output."):format(value) or "No saved burner calibration; output remains at its current amount."
    status.foreground = hoverBias(value) and colors.lime or colors.orange
  end
  heightInput:onEnter(setHeight)
  frame:addButton({x=11, y=8, width=10, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
  frame:addButton({x=22, y=8, width=6, height=1, text="ALT -", background=colors.orange}):onClick(function() stepSavedAltitude(-1) end)
  frame:addButton({x=29, y=8, width=6, height=1, text="ALT +", background=colors.orange}):onClick(function() stepSavedAltitude(1) end)
  frame:addButton({x=2, y=9, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  frame:addButton({x=13, y=9, width=10, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  frame:addButton({x=24, y=9, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  frame:addButton({x=2, y=10, width=12, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  frame:addButton({x=15, y=10, width=12, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  calibrateButton = frame:addButton({x=2, y=11, width=13, height=1, text=calibrated and "RECALIBRATE" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  frame:addButton({x=16, y=11, width=10, height=1, text="SWEEP V", background=colors.orange}):onClick(startBurnerSweep)
  frame:addButton({x=27, y=11, width=12, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  frame:addButton({x=40, y=11, width=10, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
  pidInput = frame:addInput({x=2, y=12, width=screenWidth-13, height=1, placeholder="Horizontal P I D only", background=colors.white, foreground=colors.black})
  showGainValues = function()
    pidInput.text = ("%.3g %.1g %.2g"):format(table.unpack(gainValues()))
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
frame:addLabel({x=2, y=1, width=30, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
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
  status.text = hoverBias(value) and ("Height Y %.0f uses calibrated burner output."):format(value) or "No saved burner calibration; output stays at its current amount."
  status.foreground = hoverBias(value) and colors.lime or colors.orange
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
flightButton(2, 7, "ALT -", colors.orange, function() stepSavedAltitude(-1) end)
flightButton(10, 7, "ALT +", colors.orange, function() stepSavedAltitude(1) end)
flightButton(18, 9, "HOLD", colors.blue, holdHere)
flightButton(28, 12, "AUTOPILOT", colors.lime, engageAutopilot)
flightButton(41, 9, "STOP", colors.red, stop)

pidSummary = calibrationPage:addLabel({x=2, y=1, width=47, height=1, text="PID  loading gains...", foreground=colors.lightGray})
calibrationPage:addLabel({x=2, y=3, width=47, height=1, text="HORIZONTAL PID: P I D   |   HEIGHT: BURNER AMOUNT", foreground=colors.lightBlue})
pidInput = calibrationPage:addInput({x=2, y=4, width=35, height=1, placeholder="0.035 0.0005 0.18", background=colors.white, foreground=colors.black})
showGainValues = function()
  pidInput.text = ("%.4f %.5f %.2f"):format(table.unpack(gainValues()))
end
showGainValues()
local function setGains()
  local values = {}
  for token in pidInput.text:gmatch("%S+") do
    local value = tonumber(token)
    if not value then status.text = "Enter horizontal P I D values only."; status.foreground = colors.red; return end
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
calibrationButton(2, 6, 22, "Y+ / FWD", colors.gray, function() testVector(0, .2, "TEST Y+") end)
calibrationButton(26, 6, 22, "Y- / BACK", colors.gray, function() testVector(0, -.2, "TEST Y-") end)
calibrateButton = calibrationPage:addButton({x=2, y=9, width=14, height=1, text=calibrated and "RECALIBRATE" or "CALIBRATE", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
calibrationPage:addButton({x=18, y=9, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
calibrationPage:addButton({x=32, y=9, width=15, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
calibrationButton(2, 12, 16, "SWEEP 15-5", colors.orange, startBurnerSweep)
calibrationPage:addLabel({x=20, y=12, width=13, height=1, text="Sweep saves stable heights", foreground=colors.lightGray})
calibrationPage:addButton({x=35, y=12, width=14, height=1, text="CLEAR HEIGHTS", background=colors.red}):onClick(clearHoverMap)
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
  if os.clock() - lastThrustRead >= 1 then
    readThrusterPower()
    lastThrustRead = os.clock()
  end
  if compact then
    posText.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    altText.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
  else
    posText.text = position and ("X " .. ("%.1f"):format(position.x) .. "  Y " .. ("%.1f"):format(position.y) .. "\nZ " .. ("%.1f"):format(position.z)) or "GPS not found"
    altText.text = ("Now %s\nTarget %s"):format(position and ("%.1f"):format(position.y) or "--", altitude and ("%.0f"):format(altitude) or "--")
    sysText.text = ("Burner %d/15\nThrust %.1f/15\nHeading %s°"):format(burner, measuredThrust, shipHeading and ("%.0f"):format(math.deg(shipHeading)) or "--")
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
  if pidSummary then pidSummary.text = directGpuError and ("GPU: " .. directGpuError) or ("Horizontal PID %.3f / %.4f / %.2f | Height from burner map"):format(pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d) end
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
    createGpuButtons()
    local ok = pcall(renderDirectMonitor)
    if ok then lastGpuRender = os.clock()
    else directGpu = nil; directGpuError = "DirectGPU connection lost; reconnecting" end
  end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(.5, false)
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
      if directGpu and os.clock() - lastGpuCheck >= 5 then
        lastGpuCheck = os.clock()
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
                if button.label:match("^SWEEP") then sweepSummary = nil end
                break
              end
            end
          end
        end
      end
      sleep(directGpu and .2 or 2)
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
