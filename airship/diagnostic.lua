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

print("Airship altitude diagnostic; no GPS coordinates are recorded.")
local metadata = {
  {"Ship footprint", "15x15"},
  {"Balloon inside capacity in blocks (verify)", "845"},
  {"Burner count", "4"},
  {"Max fill per burner", "210"},
  {"Total usable burner fill", "840"},
  {"Ship mass/weight if known", "unknown"},
  {"Cargo/load during test", "none"},
  {"Test altitude or notes", ""}
}

local filename = "airship-diagnostic.csv"
local suffix = 1
while fs.exists(filename) do
  suffix = suffix + 1
  filename = "airship-diagnostic-" .. suffix .. ".csv"
end

local log = assert(fs.open(filename, "w"), "Could not create diagnostic file")
log.writeLine("# Airship altitude diagnostic; GPS coordinates omitted")
for _, field in ipairs(metadata) do
  local value = ask(field[1], field[2]):gsub("[\r\n]", " ")
  log.writeLine("# " .. field[1] .. ": " .. value)
end
log.writeLine("elapsed_s,height,vertical_speed,air_pressure,burner_signal,phase")
log.flush()

local phase = "start"
local started = os.clock()
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
    sleep(.2)
  end
end

local function controls()
  print("Set burner signal: 0-15 and optional label, e.g. 4 hover; q ends and cuts signal.")
  while true do
    write("signal> ")
    local input = read()
    if input:lower() == "q" then return end
    local level, label = input:match("^%s*(%d+)%s*(.-)%s*$")
    level = tonumber(level)
    if level and level >= 0 and level <= 15 then
      redstone.setAnalogOutput("front", level)
      phase = ("%d %s"):format(level, label):gsub("[\r\n,]", " ")
      print("Signal " .. level .. "/15  phase: " .. phase)
    else
      print("Enter a whole signal strength from 0 to 15, or q to finish.")
    end
  end
end

redstone.setAnalogOutput("front", 0)
local ok, err = pcall(parallel.waitForAny, record, controls)
redstone.setAnalogOutput("front", 0)
log.close()
print("Saved " .. filename .. "; burner signal is now 0.")
if not ok then printError(err); return end

print("Pastebin guest uploads are public. This file contains no GPS coordinates or player/server name.")
write("Type UPLOAD to publish the log, or press Enter to keep it local: ")
if read() == "UPLOAD" then
  shell.run("pastebin", "put", filename)
else
  print("Kept locally as " .. filename)
end
