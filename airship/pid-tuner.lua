-- Analyze relative autopilot runs and propose a small horizontal PID adjustment.
local latest, latestTime
for _, name in ipairs(fs.list("/")) do
  if name:match("^airship%-pid%-diagnostic.*%.csv$") then
    local time=fs.getLastModified("/"..name) or 0
    if not latestTime or time>latestTime then latest,latestTime=name,time end
  end
end
if not latest then error("No airship-pid-diagnostic*.csv found. Run diagnostic.lua first.") end
local file=assert(fs.open("/"..latest,"r")); local text=file.readAll(); file.close()
local overshoot={X=0,Z=0}; local samples={X=0,Z=0}
for line in text:gmatch("[^\r\n]+") do
  local c={}; for value in (line..","):gmatch("(.-),") do c[#c+1]=value end
  local phase=c[2]
  if phase and phase:match("^AUTO_[XZ]_") then
    local axis=phase:sub(6,6)
    local actual=tonumber(axis=="X" and c[15] or c[16])
    local target=tonumber(axis=="X" and c[19] or c[20])
    if actual and target then
      samples[axis]=samples[axis]+1
      local past=target>=0 and math.max(0,actual-target) or math.max(0,target-actual)
      overshoot[axis]=math.max(overshoot[axis],past)
    end
  end
end
if samples.X==0 or samples.Z==0 then error("Latest diagnostic is missing X or Z autopilot runs; no gains changed.") end

local gains={.035,.0005,.18}
if fs.exists("/airship-pid.cfg") then
  local cfg=fs.open("/airship-pid.cfg","r"); local values={}
  for token in cfg.readAll():gmatch("%S+") do values[#values+1]=tonumber(token) end; cfg.close()
  if #values>=3 and values[1] and values[2] and values[3] then gains={values[1],values[2],values[3]} end
end
local peak=math.max(overshoot.X,overshoot.Z)
if peak>.25 then
  gains[1]=math.max(.005,gains[1]*math.max(.65,1-math.min(.35,peak*.04)))
  gains[3]=math.min(.8,gains[3]*(1+math.min(.5,peak*.03)))
elseif peak<.05 then gains[1]=math.min(.15,gains[1]*1.08) end
print("Analyzed: "..latest)
print(("Peak overshoot: X %.2f blocks, Z %.2f blocks"):format(overshoot.X,overshoot.Z))
print(("Suggested horizontal P I D: %.4f %.5f %.3f"):format(gains[1],gains[2],gains[3]))
print("Heuristic starting point only; repeat the diagnostic and compare results.")
write("Save suggested horizontal gains to airship-pid.cfg? [y/N] ")
if read():lower()~="y" then print("No changes made."); return end
local tmp="/airship-pid.cfg.tmp"
local out=assert(fs.open(tmp,"w")); out.write(table.concat(gains," ")); out.writeLine(""); out.close()
if fs.exists("/airship-pid.cfg.bak") then fs.delete("/airship-pid.cfg.bak") end
if fs.exists("/airship-pid.cfg") then fs.move("/airship-pid.cfg","/airship-pid.cfg.bak") end
local moved,err=pcall(fs.move,tmp,"/airship-pid.cfg")
if not moved then
  if fs.exists("/airship-pid.cfg.bak") then fs.move("/airship-pid.cfg.bak","/airship-pid.cfg") end
  error("Could not save gains: "..tostring(err))
end
if fs.exists("/airship-pid.cfg.bak") then fs.delete("/airship-pid.cfg.bak") end
print("Saved. Restart airship.lua to apply the gains.")
