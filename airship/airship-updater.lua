local url = "https://raw.githubusercontent.com/eranfeldman/cc-tweaked/main/airship/airship.lua"
local expectedBuild = "1.1.0"
local target, temporary, backup = "/airship.lua", "/airship.lua.update", "/airship.lua.bak"

term.clear()
term.setCursorPos(1, 1)
print("Airship controller updater")
print("Downloading latest controller (expected v" .. expectedBuild .. ")...")
if fs.exists(temporary) then fs.delete(temporary) end
if not shell.run("wget", url .. "?cache=" .. tostring(os.epoch("utc")), temporary) then
  printError("Download failed. The old controller is unchanged.")
  return
end

if not fs.exists(temporary) or fs.getSize(temporary) == 0 then
  printError("Downloaded file is empty. The old controller is unchanged.")
  return
end
local program, syntaxError = loadfile(temporary)
if not program then
  fs.delete(temporary)
  printError("Downloaded controller is invalid: " .. tostring(syntaxError))
  return
end
local downloadedFile = fs.open(temporary, "r")
local source = downloadedFile.readAll()
downloadedFile.close()
local downloadedBuild = source:match('local AIRSHIP_BUILD = "([^"]+)"')
if downloadedBuild ~= expectedBuild then
  fs.delete(temporary)
  printError("Downloaded version mismatch: got " .. tostring(downloadedBuild) .. ", expected " .. expectedBuild .. ".")
  return
end

if fs.exists(backup) then fs.delete(backup) end
if fs.exists(target) then fs.move(target, backup) end
local moved, moveError = pcall(fs.move, temporary, target)
if not moved then
  if fs.exists(backup) then fs.move(backup, target) end
  printError("Could not install update: " .. tostring(moveError))
  return
end

print("Installed airship v" .. downloadedBuild .. ". Launching controller...")
sleep(1)
shell.run(target)
