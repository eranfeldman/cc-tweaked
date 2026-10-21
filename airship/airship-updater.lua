local updaterBuild = "1.2.0"
local minimumBuild, minimumUpdaterBuild = "1.1.9", "1.2.0"
local apiUrl = "https://api.github.com/repos/eranfeldman/cc-tweaked/commits/main"
local baseUrl = "https://raw.githubusercontent.com/eranfeldman/cc-tweaked/"
local controller, controllerTemp, controllerBackup = "/airship.lua", "/airship.lua.update", "/airship.lua.bak"
local updater, updaterTemp, updaterBackup = "/airship-updater.lua", "/airship-updater.lua.update", "/airship-updater.lua.bak"
local commitFile = "/airship-latest.json"

local function atLeast(version, minimum)
  if type(version) ~= "string" then return false end
  local a, b, c = version:match("^(%d+)%.(%d+)%.(%d+)$")
  local x, y, z = minimum:match("^(%d+)%.(%d+)%.(%d+)$")
  if not a or not x then return false end
  a, b, c, x, y, z = tonumber(a), tonumber(b), tonumber(c), tonumber(x), tonumber(y), tonumber(z)
  return a > x or a == x and (b > y or b == y and c >= z)
end

local function download(url, path, checkLua)
  if fs.exists(path) then fs.delete(path) end
  if not shell.run("wget", url, path) or not fs.exists(path) or fs.getSize(path) == 0 then
    if fs.exists(path) then fs.delete(path) end
    return nil, "download failed: " .. url
  end
  if checkLua then
    local program, err = loadfile(path)
    if not program then fs.delete(path); return nil, "invalid Lua: " .. tostring(err) end
  end
  local file = fs.open(path, "r")
  local source = file.readAll()
  file.close()
  return source
end

local function install(temp, path, backup)
  if fs.exists(backup) then fs.delete(backup) end
  if fs.exists(path) then
    local saved, saveError = pcall(fs.move, path, backup)
    if not saved then return false, "could not back up " .. path .. ": " .. tostring(saveError) end
  end
  local moved, moveError = pcall(fs.move, temp, path)
  if not moved then
    if fs.exists(backup) then pcall(fs.move, backup, path) end
    return false, "could not install " .. path .. ": " .. tostring(moveError)
  end
  return true
end

term.clear()
term.setCursorPos(1, 1)
print("Airship updater " .. updaterBuild)
print("Resolving latest main commit...")
local apiSource, err = download(apiUrl, commitFile)
if not apiSource then printError(err); return end
local decoded, commit = pcall(textutils.unserializeJSON, apiSource)
fs.delete(commitFile)
if not decoded or type(commit) ~= "table" or type(commit.sha) ~= "string" or #commit.sha ~= 40 or not commit.sha:match("^%x+$") then
  printError("GitHub did not return a valid commit SHA; installed files are unchanged.")
  return
end

local controllerUrl = baseUrl .. commit.sha .. "/airship/airship.lua"
local updaterUrl = baseUrl .. commit.sha .. "/airship/airship-updater.lua"
print("Downloading commit " .. commit.sha:sub(1, 10) .. "...")
local controllerSource, controllerError = download(controllerUrl, controllerTemp, true)
if not controllerSource then printError(controllerError); return end
local controllerVersion = controllerSource:match('local AIRSHIP_BUILD = "([^"]+)"')
if not atLeast(controllerVersion, minimumBuild) then
  fs.delete(controllerTemp)
  printError("Latest main has unsupported controller version " .. tostring(controllerVersion) .. "; installed files are unchanged.")
  return
end

local updaterSource, updaterError = download(updaterUrl, updaterTemp, true)
if not updaterSource then fs.delete(controllerTemp); printError(updaterError); return end
local updaterVersion = updaterSource:match('local updaterBuild = "([^"]+)"')
if not atLeast(updaterVersion, minimumUpdaterBuild) then
  fs.delete(controllerTemp); fs.delete(updaterTemp)
  printError("Latest main has an old or invalid updater; installed files are unchanged.")
  return
end

local installed, installError = install(controllerTemp, controller, controllerBackup)
if not installed then fs.delete(updaterTemp); printError(installError); return end
installed, installError = install(updaterTemp, updater, updaterBackup)
if not installed then printError(installError .. ". Controller updated; updater backup retained.") end

print("Installed airship v" .. controllerVersion .. " from commit " .. commit.sha:sub(1, 10) .. ".")
sleep(1)
shell.run(controller)
