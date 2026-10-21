-- Refreshes the controller/updater and removes only known temporary airship files.
local controllerUrl = "https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua"
local updaterUrl = "https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship-updater.lua"
local controllerTemp, updaterTemp = "/airship.lua.cleaning", "/airship-updater.lua.cleaning"
local controllerBackup, updaterBackup = "/airship.lua.bak", "/airship-updater.lua.bak"

local function buildAtLeast(version, minimum)
  local a,b,c = version and version:match("^(%d+)%.(%d+)%.(%d+)$")
  local x,y,z = minimum:match("^(%d+)%.(%d+)%.(%d+)$")
  if not a or not x then return false end
  a,b,c,x,y,z = tonumber(a),tonumber(b),tonumber(c),tonumber(x),tonumber(y),tonumber(z)
  return a>x or a==x and (b>y or b==y and c>=z)
end

local function download(url, path, marker, minimum)
  if fs.exists(path) then fs.delete(path) end
  if not shell.run("wget", url .. "?cache=" .. tostring(os.epoch("utc")), path) then return nil, "download failed" end
  local program, err = loadfile(path)
  if not program then fs.delete(path); return nil, "invalid Lua: " .. tostring(err) end
  local file = fs.open(path, "r")
  local source = file.readAll()
  file.close()
  local version = source:match(marker)
  if not buildAtLeast(version, minimum) then
    fs.delete(path)
    return nil, "version check failed: " .. tostring(version)
  end
  return true
end

local function replace(path, temp, backup)
  if fs.exists(backup) then fs.delete(backup) end
  if fs.exists(path) then fs.move(path, backup) end
  local ok, err = pcall(fs.move, temp, path)
  if not ok then
    if fs.exists(backup) then fs.move(backup, path) end
    return false, tostring(err)
  end
  return true
end

local function stale(name)
  if name == "airship-cleanup.lua" then return false end
  if name == "airship.lua.update" or name == "airship.lua.cleaning" or name == "airship-updater.lua.cleaning" or name == "airship-updater-new.lua" or name == "airship-new.lua" or name == "airship-next.lua" then return true end
  if name:match("^airship%-v%d+%.%d+%.%d+%.lua$") then return true end
  if name:match("^airship%-hub%-diagnostic[%-%d]*%.txt$") then return true end
  for _, prefix in ipairs({"airship-before-", "airship-previous", "airship-old", "airship-failed"}) do
    if name:sub(1, #prefix) == prefix and name:match("%.lua$") then return true end
  end
  return false
end

term.clear()
term.setCursorPos(1,1)
print("Airship maintenance and cleanup")
print("This updates airship.lua and its updater, then removes old temporary copies and hub diagnostic reports.")
print("Calibration, PID settings, hover map, CSV diagnostics, and current controller/updater backups are preserved.")
local candidates = {}
for _, name in ipairs(fs.list("/")) do
  if stale(name) and not fs.isDir("/" .. name) then candidates[#candidates+1] = name end
end
if #candidates == 0 then print("No leftover temporary files found.") end
for _, name in ipairs(candidates) do print("Will remove: " .. name) end
write("Type CLEAN to continue: ")
if read() ~= "CLEAN" then print("Cancelled; no files changed."); return end

local ok, err = download(controllerUrl, controllerTemp, 'local AIRSHIP_BUILD = "([^\"]+)"', "1.1.6")
if not ok then printError("Controller refresh stopped: " .. tostring(err)); return end
ok, err = download(updaterUrl, updaterTemp, 'local minimumBuild = "([^\"]+)"', "1.1.5")
if not ok then fs.delete(controllerTemp); printError("Updater refresh stopped: " .. tostring(err)); return end

ok, err = replace("/airship.lua", controllerTemp, controllerBackup)
if not ok then fs.delete(updaterTemp); printError("Controller install failed: " .. tostring(err)); return end
ok, err = replace("/airship-updater.lua", updaterTemp, updaterBackup)
if not ok then printError("Updater install failed: " .. tostring(err)); return end

local removed = 0
for _, name in ipairs(candidates) do
  if fs.exists("/" .. name) then
    local deleted, deleteError = pcall(fs.delete, "/" .. name)
    if deleted then removed = removed + 1 else printError("Could not remove " .. name .. ": " .. tostring(deleteError)) end
  end
end
print("Installed airship v1.1.6 and current updater.")
print("Removed " .. removed .. " temporary files. Calibration and flight data remain.")
print("The UPDATE button now runs the refreshed updater; launch airship.lua when ready.")
