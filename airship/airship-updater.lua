local updaterBuild = "1.3.0"
local apiUrl = "https://api.github.com/repos/eranfeldman/cc-tweaked/commits/main"
local baseUrl = "https://raw.githubusercontent.com/eranfeldman/cc-tweaked/"
local commitFile = "/airship-latest.json"
local files = {
  {name="airship.lua", marker='local AIRSHIP_BUILD = "([^\"]+)"', minimum="1.2.19"},
  {name="diagnostic.lua"},
  {name="hub-diagnostic.lua"},
  {name="pid-tuner.lua"},
  {name="airship-updater.lua", marker='local updaterBuild = "([^\"]+)"', minimum="1.3.0"},
}

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

local function removeTemps()
  for _, file in ipairs(files) do
    if file.temp and fs.exists(file.temp) then fs.delete(file.temp) end
  end
end

local function install(file)
  if fs.exists(file.backup) then fs.delete(file.backup) end
  if fs.exists(file.path) then
    local saved, saveError = pcall(fs.move, file.path, file.backup)
    if not saved then return false, "could not back up " .. file.path .. ": " .. tostring(saveError) end
  end
  local moved, moveError = pcall(fs.move, file.temp, file.path)
  if not moved then
    if fs.exists(file.backup) then pcall(fs.move, file.backup, file.path) end
    return false, "could not install " .. file.path .. ": " .. tostring(moveError)
  end
  return true
end

local function rollback(installed)
  for i = #installed, 1, -1 do
    local file = installed[i]
    if fs.exists(file.path) then fs.delete(file.path) end
    if fs.exists(file.backup) then pcall(fs.move, file.backup, file.path) end
  end
end

term.clear()
term.setCursorPos(1, 1)
print("Airship updater " .. updaterBuild)
print("Resolving latest main commit...")
local apiSource, err = download(apiUrl, commitFile, false)
if not apiSource then printError(err); return end
local decoded, commit = pcall(textutils.unserializeJSON, apiSource)
fs.delete(commitFile)
if not decoded or type(commit) ~= "table" or type(commit.sha) ~= "string" or #commit.sha ~= 40 or not commit.sha:match("^%x+$") then
  printError("GitHub did not return a valid commit SHA; installed files are unchanged.")
  return
end

local commitUrl = baseUrl .. commit.sha .. "/airship/"
print("Downloading airship files from commit " .. commit.sha:sub(1, 10) .. "...")
for _, file in ipairs(files) do
  file.path = "/" .. file.name
  file.temp = file.path .. ".update"
  file.backup = file.path .. ".bak"
  local source, downloadError = download(commitUrl .. file.name, file.temp, true)
  if not source then removeTemps(); printError(downloadError .. "; installed files are unchanged."); return end
  if file.marker then
    local version = source:match(file.marker)
    if not atLeast(version, file.minimum) then
      removeTemps()
      printError(file.name .. " has unsupported version " .. tostring(version) .. "; installed files are unchanged.")
      return
    end
    file.version = version
  end
end

local installed = {}
for _, file in ipairs(files) do
  local ok, installError = install(file)
  if not ok then
    rollback(installed)
    removeTemps()
    printError(installError .. "; installed files were rolled back where possible.")
    return
  end
  installed[#installed + 1] = file
end

print("Installed airship files from commit " .. commit.sha:sub(1, 10) .. ".")
print("Updated controller v" .. files[1].version .. ", diagnostic, hub diagnostic, PID tuner, and updater.")
print("Previous copies are saved beside each file as .bak.")
sleep(1)
shell.run("/airship.lua")
