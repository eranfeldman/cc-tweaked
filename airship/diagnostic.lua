-- Logs Avionics altitude response while manually stepping the burner Redstone Link.
local sensor = peripheral.find("altitude_sensor")
if not sensor then error("Connect an Avionics altitude_sensor to this computer first") end
local sensorOk, height, verticalSpeed, pressure = pcall(function()
  return sensor.getHeight(), sensor.getVerticalSpeed(), sensor.getAirPressure()
end)
if not sensorOk or type(height) ~= "number" or type(verticalSpeed) ~= "number" or type(pressure) ~= "number" then
  error("Altitude sensor did not return height, vertical speed, and air pressure")
end

local function ask(label, default)
  write(label .. " [" .. default .. "]: ")
  local value = read()
  return value == "" and default or value
end

local function askNumber(label, default, minimum)
  while true do
    local value = tonumber(ask(label, tostring(default)))
    if value and value >= minimum and value < math.huge then return value end
    print("Enter a number at least " .. minimum .. ".")
  end
end

print("Automatic airship altitude diagnostic; no GPS coordinates are recorded.")
local capacity = askNumber("Balloon inside capacity in blocks (verify)", 845, 1)
local burnerCount = askNumber("Number of burners", 4, 1)
local fillPerBurner = askNumber("Max fill per burner", 210, 1)
local totalFill = math.min(capacity, burnerCount * fillPerBurner)
local mass = ask("Ship mass/weight if known", "unknown"):gsub("[\r\n]", " ")
local load = ask("Cargo/load during test", "none"):gsub("[\r\n]", " ")
local footprint = ask("Ship footprint", "15x15"):gsub("[\r\n]", " ")

local baseline = 4
if fs.exists("airship-pid.cfg") then
  local config = fs.open("airship-pid.cfg", "r")
  local values = {}
  for token in config.readAll():gmatch("%S+") do values[#values + 1] = tonumber(token) end
  config.close()
  if values[4] and values[4] >= 0 and values[4] <= 15 then baseline = math.floor(values[4] + .5) end
end

print(("Test will use signal %d, %d, and %d around the saved bias %d for 20 seconds each."):format(
  math.max(0, baseline - 1), baseline, math.min(15, baseline + 1), baseline))
print("It aborts if height changes by 5 blocks or vertical speed reaches 1.2 blocks/sec.")
print("Run only in clear, safe airspace; Pastebin upload is public and omits GPS coordinates.")
write("Type YES to start the automatic test: ")
if read() ~= "YES" then print("Cancelled; no test started."); return end

local filename = "airship-diagnostic.csv"
local suffix = 1
while fs.exists(filename) do
  suffix = suffix + 1
  filename = "airship-diagnostic-" .. suffix .. ".csv"
end

local log = assert(fs.open(filename, "w"), "Could not create diagnostic file")
log.writeLine("# Airship altitude diagnostic; GPS coordinates omitted")
log.writeLine("# Ship footprint: " .. footprint)
log.writeLine("# Balloon capacity: " .. capacity .. " blocks")
log.writeLine("# Burner count: " .. burnerCount)
log.writeLine("# Max fill per burner: " .. fillPerBurner)
log.writeLine("# Total usable fill: " .. totalFill)
log.writeLine("# Ship mass/weight: " .. mass)
log.writeLine("# Cargo/load: " .. load)
log.writeLine("# Test signals: " .. math.max(0, baseline - 1) .. "," .. baseline .. "," .. math.min(15, baseline + 1))
log.writeLine("elapsed_s,height,vertical_speed,air_pressure,burner_signal,phase")
log.flush()

local phase = "starting"
local started = os.clock()
local startOk, startingHeight = pcall(sensor.getHeight)
if not startOk or type(startingHeight) ~= "number" then log.close(); error("Could not read starting altitude") end
local aborted = false
local function record()
  while true do
    local ok, height, verticalSpeed, pressure = pcall(function()
      return sensor.getHeight(), sensor.getVerticalSpeed(), sensor.getAirPressure()
    end)
    if not ok then error("Altitude sensor read failed: " .. tostring(height)) end
    local signal = redstone.getAnalogOutput("front") or 0
    log.writeLine(("%.2f,%.3f,%.4f,%.5f,%d,%s"):format(
      os.clock() - started, height, verticalSpeed, pressure, signal, phase))
    log.flush()
    if math.abs(height - startingHeight) >= 5 or math.abs(verticalSpeed) >= 1.2 then
      aborted = true
      phase = "SAFETY ABORT"
      return
    end
    sleep(.2)
  end
end

local function runTest()
  local profile = {
    {baseline, "baseline"},
    {math.min(15, baseline + 1), "plus_one"},
    {baseline, "return_baseline"},
    {math.max(0, baseline - 1), "minus_one"},
    {baseline, "final_baseline"}
  }
  for _, step in ipairs(profile) do
    redstone.setAnalogOutput("front", step[1])
    phase = step[2]
    sleep(20)
  end
end

local function waitForAbort()
  while true do
    local _, key = os.pullEvent("key")
    if key == keys.q then aborted = true; phase = "MANUAL ABORT"; return end
  end
end

redstone.setAnalogOutput("front", baseline)
local ok, err = pcall(parallel.waitForAny, record, runTest, waitForAbort)
redstone.setAnalogOutput("front", 0)
log.close()
print("Saved " .. filename .. "; burner signal is now 0.")
if not ok then printError(err); return end
if aborted then print("Test stopped early by the safety limit or Q key.") else print("Automatic test finished.") end
shell.run("pastebin", "put", filename)
