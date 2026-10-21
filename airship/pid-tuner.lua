-- Analyze the last altitude-step diagnostic and propose conservative altitude PID gains.
local latest, latestTime
for _, name in ipairs(fs.list("/")) do
  if name:match("^airship%-pid%-diagnostic.*%.csv$") then
    local time = fs.getLastModified("/" .. name) or 0
    if not latestTime or time > latestTime then latest, latestTime = name, time end
  end
end
if not latest then error("No airship-pid-diagnostic*.csv found. Run diagnostic.lua first.") end

local f = assert(fs.open("/" .. latest, "r"))
local text = f.readAll(); f.close()
if text:find("MAPPED BURNER ACTIVE (altitude PID bypassed)", 1, true) then
  error("This older log bypassed altitude PID. Run the updated diagnostic.lua before tuning.")
end

local peaks = {}
for line in text:gmatch("[^\r\n]+") do
  local phase, target, height = line:match("^[^,]+,([^,]+),[^,]*,([%-%.%d]+),([%-%.%d]+),")
  target, height = tonumber(target), tonumber(height)
  if phase and target and height and (phase == "ALT_STEP_UP_3" or phase == "ALT_STEP_DOWN_3") then
    local error = height - target
    peaks[phase] = math.max(peaks[phase] or 0, phase == "ALT_STEP_UP_3" and math.max(0, error) or math.max(0, -error))
  end
end
if not peaks.ALT_STEP_UP_3 or not peaks.ALT_STEP_DOWN_3 then
  error("The diagnostic log is missing one or both altitude step phases.")
end

local gains = {.035,.0005,.18,4,.45,.005,2.5}
if fs.exists("/airship-pid.cfg") then
  local cfg=fs.open("/airship-pid.cfg","r")
  local values={}; for token in cfg.readAll():gmatch("%S+") do values[#values+1]=tonumber(token) end; cfg.close()
  if #values==7 then for i,v in ipairs(values) do if v then gains[i]=v end end end
end
local peak = math.max(peaks.ALT_STEP_UP_3, peaks.ALT_STEP_DOWN_3)
local ratio = peak / 3
if ratio > .08 then
  gains[5] = math.max(.1, gains[5] * math.max(.65, 1 - math.min(.35, ratio * .4)))
  gains[7] = math.min(5, gains[7] * (1 + math.min(.5, ratio * .3)))
elseif ratio < .025 then
  gains[5] = math.min(3, gains[5] * 1.08)
end

print("Analyzed: " .. latest)
print(("Peak step overshoot: %.2f blocks (%.0f%% of 3-block step)"):format(peak, ratio * 100))
print(("Suggested altitude bias/P/I/D: %.3f %.4f %.5f %.4f"):format(gains[4],gains[5],gains[6],gains[7]))
print("This is a conservative next-test suggestion, not a guaranteed optimum.")
write("Save suggested gains to airship-pid.cfg? [y/N] ")
if read():lower() ~= "y" then print("No changes made."); return end

local tmp="/airship-pid.cfg.tmp"
local out=assert(fs.open(tmp,"w")); out.write(table.concat(gains," ")); out.writeLine(""); out.close()
if fs.exists("/airship-pid.cfg.bak") then fs.delete("/airship-pid.cfg.bak") end
if fs.exists("/airship-pid.cfg") then fs.move("/airship-pid.cfg","/airship-pid.cfg.bak") end
local moved, err=pcall(fs.move,tmp,"/airship-pid.cfg")
if not moved then
  if fs.exists("/airship-pid.cfg.bak") then fs.move("/airship-pid.cfg.bak","/airship-pid.cfg") end
  error("Could not save PID file: "..tostring(err))
end
if fs.exists("/airship-pid.cfg.bak") then fs.delete("/airship-pid.cfg.bak") end
print("Saved. Restart airship.lua and repeat the altitude step diagnostic before relying on the change.")
