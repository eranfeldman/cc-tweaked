-- Read-only report for CC:Tweaked peripherals and Malahit peripheral hubs.
local report = {}
local function line(value) report[#report + 1] = tostring(value) end

local function show(value)
  local ok, text = pcall(textutils.serialize, value, { compact = true })
  return ok and text or ("<serialize error: " .. tostring(text) .. ">")
end

local function call(fn, ...)
  local result = table.pack(pcall(fn, ...))
  if not result[1] then return nil, tostring(result[2]) end
  local values = { n = result.n - 1 }
  for i = 2, result.n do values[i - 1] = result[i] end
  return values
end

local function reportCall(label, fn, ...)
  local values, err = call(fn, ...)
  line("  " .. label .. ": " .. (err and ("ERROR " .. err) or show(values)))
  return values
end

local probes = {
  "getHeading", "getHeadingRad", "getOrientation", "getAngles", "getAnglesRad",
  "getAngularRates", "getAngularRatesRad", "getGravity", "getLinearAcceleration",
  "getHeight", "getVerticalSpeed", "getAirPressure", "getPower", "getThrust",
  "getVectorX", "getVectorY", "getTargetVectorX", "getTargetVectorY",
}

line("CC:Tweaked peripheral and hub diagnostic")
line("Read-only: no redstone, thruster, or other controls are changed.")
line("No GPS or world coordinates are read.")
line("Time: " .. (os.date and os.date("%Y-%m-%d %H:%M:%S") or "unavailable"))

local ok, names = pcall(peripheral.getNames)
if not ok then
  line("peripheral.getNames ERROR: " .. tostring(names))
  names = {}
end
line("Attached peripheral count: " .. #names)

local hubs = {}
local remoteNames = {}
for _, name in ipairs(names) do
  line("\nPERIPHERAL " .. name)
  local typeValues = reportCall("types", peripheral.getType, name)
  local types = typeValues and typeValues[1] or {}
  if type(types) ~= "table" then
    local packed = typeValues or { n = 0 }
    types = {}
    for i = 1, packed.n do types[#types + 1] = packed[i] end
  end
  line("  methods: " .. show(peripheral.getMethods(name) or {}))

  local isHub = false
  for _, kind in ipairs(types) do
    if kind == "peripheral_hub" or kind == "malahit_hub" then isHub = true end
  end
  local methodList = peripheral.getMethods(name) or {}
  for _, method in ipairs(methodList) do
    if method == "getNamesRemote" then isHub = true end
  end

  local wrapped = peripheral.wrap(name)
  if isHub and wrapped then
    hubs[#hubs + 1] = { name = name, peripheral = wrapped }
    local got = reportCall("getNamesRemote", wrapped.getNamesRemote)
    local list = got and got[1]
    if type(list) == "table" then
      for _, remoteName in ipairs(list) do remoteNames[remoteName] = true end
    end
    reportCall("getLinks", wrapped.getLinks)
    reportCall("getGeometry", wrapped.getGeometry)
  end

  local methods = {}
  for _, method in ipairs(probes) do
    for _, available in ipairs(methodList) do
      if method == available then methods[#methods + 1] = method end
    end
  end
  for _, method in ipairs(methods) do
    reportCall(method, wrapped[method])
  end
end

for _, hub in ipairs(hubs) do
  local remote = reportCall("remote names via " .. hub.name, hub.peripheral.getNamesRemote)
  local list = remote and remote[1]
  if type(list) == "table" then
    for _, name in ipairs(list) do
      line("\nREMOTE PERIPHERAL " .. name .. " via " .. hub.name)
      reportCall("type", hub.peripheral.getTypeRemote, name)
      local gotMethods = reportCall("methods", hub.peripheral.getMethodsRemote, name)
      local available = gotMethods and gotMethods[1]
      if type(available) == "table" then
        for _, method in ipairs(probes) do
          for _, candidate in ipairs(available) do
            if method == candidate then
              reportCall(method, hub.peripheral.callRemote, name, method)
            end
          end
        end
      end
    end
  end
end

if #hubs == 0 then line("\nNo attached peripheral exposed a hub type or getNamesRemote method.") end
if next(remoteNames) == nil then line("\nNo remote peripheral names were returned by any hub.") end

local filename = "airship-hub-diagnostic.txt"
local suffix = 1
while fs.exists(filename) do
  suffix = suffix + 1
  filename = "airship-hub-diagnostic-" .. suffix .. ".txt"
end
local file = assert(fs.open(filename, "w"), "Could not create diagnostic report")
file.write(table.concat(report, "\n") .. "\n")
file.close()
print("Saved " .. filename)
print("Uploading report to public Pastebin...")
local okUpload, result = pcall(shell.run, "pastebin", "put", filename)
if not okUpload or not result then
  print("Pastebin upload failed. Send the local file instead: " .. filename)
else
  print("Upload complete. Copy the Pastebin URL shown above and send it here.")
end
