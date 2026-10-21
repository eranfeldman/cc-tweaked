-- Resolve Avionics sensors directly or through the nested Malahit peripheral hub.
local function findAvionics(kind)
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
              return setmetatable({}, {__index=function(_, method) return function(...) return root.callRemote(remoteName,method,...) end end})
            elseif remoteKind == "malahit_hub" or remoteKind == "peripheral_hub" then
              local childOk, childNames = pcall(root.callRemote, remoteName, "getNamesRemote")
              if childOk and type(childNames) == "table" then
                for _, childName in ipairs(childNames) do
                  local childTypeOk, childType = pcall(root.callRemote, remoteName, "getTypeRemote", childName)
                  local childTypes = type(childType) == "table" and childType or {childType}
                  for _, childKind in ipairs(childTypes) do
                    if childKind == kind then
                      return setmetatable({}, {__index=function(_, method) return function(...) return root.callRemote(remoteName,"callRemote",childName,method,...) end end})
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
local sensor = findAvionics("altitude_sensor")
local navigation = findAvionics("navigation_table")
local gimbal = findAvionics("gimbal_sensor")
if not sensor then error("Connect an Avionics altitude_sensor first") end
local thrusters, seen = {}, {}
local function addThruster(name, device)
  if device and not seen[name] then
    seen[name] = true
    thrusters[#thrusters+1] = {name=name,device=device}
  end
end
for _,name in ipairs(peripheral.getNames()) do
  local kinds={peripheral.getType(name)}
  for _,kind in ipairs(kinds) do
    if kind=="liquid_vector_thruster" then addThruster(name,peripheral.wrap(name)) end
  end
  local root=peripheral.wrap(name)
  if root and type(root.getNamesRemote)=="function" then
    local function scan(hub,path,depth)
      if depth>2 then return end
      local ok,names=pcall(hub.getNamesRemote)
      if not ok or type(names)~="table" then return end
      for _,childName in ipairs(names) do
        local typeOk,remoteType=pcall(hub.getTypeRemote,childName)
        local types=type(remoteType)=="table" and remoteType or {remoteType}
        local child=setmetatable({}, {__index=function(_,method)
          return function(...) return hub.callRemote(childName,method,...) end
        end})
        for _,kind in ipairs(types) do
          if kind=="liquid_vector_thruster" then
            addThruster(path.."/"..childName,child)
          elseif (kind=="peripheral_hub" or kind=="malahit_hub") and depth<2 then
            scan(child,path.."/"..childName,depth+1)
          end
        end
      end
    end
    scan(root,name,1)
  end
end
table.sort(thrusters,function(a,b)
  local ai=tonumber(a.name:match("(%d+)$")) or math.huge
  local bi=tonumber(b.name:match("(%d+)$")) or math.huge
  return ai==bi and a.name<b.name or ai<bi
end)
if #thrusters==0 then error("No liquid_vector_thruster peripherals found") end
local x0,y0,z0=gps.locate(1,false)
if not x0 then error("GPS fix required before starting") end
local ok,startY,startVy,startPressure=pcall(function()
  return sensor.getHeight(),sensor.getVerticalSpeed(),sensor.getAirPressure()
end)
if not ok or type(startY)~="number" or type(startVy)~="number" or type(startPressure)~="number" then
  error("Altitude sensor must return height, vertical speed, and pressure")
end

local function ask(label,default)
  write(label.." ["..default.."]: ")
  local value=read()
  return value=="" and default or value:gsub("[\r\n]"," ")
end
local function askNumber(label,default,min)
  while true do
    local value=tonumber(ask(label,tostring(default)))
    if value and value>=min and value<math.huge then return value end
    print("Enter a number at least "..min..".")
  end
end
print("Airship diagnostic: automatic individual thruster direction survey, altitude steps, and autopilot axis runs.")
print("Detected "..#thrusters.." liquid vector thrusters: "..table.concat((function() local n={} for _,t in ipairs(thrusters) do n[#n+1]=t.name end return n end)(),", "))
print("Stop airship.lua first. Keep the ship clear of obstacles and stay at the controls.")
print("Q aborts the test and restores the burner signal that was active at start.")
local footprint=ask("Ship footprint","15x15")
local balloon=askNumber("Balloon inside capacity (blocks)",845,1)
local burnerCount=askNumber("Burner count",4,1)
local fillPerBurner=askNumber("Max fill per burner",210,1)
local mass=ask("Ship mass/weight if known","unknown")
local cargo=ask("Cargo/load during test","none")
local heading="east (side with thrusters 0 and 6)"

local gains={.035,.0005,.18}
if fs.exists("airship-pid.cfg") then
  local f=fs.open("airship-pid.cfg","r"); local values={}
  for token in f.readAll():gmatch("%S+") do values[#values+1]=tonumber(token) end
  f.close()
  local valid=#values>=3
  for i=1,3 do local v=values[i]; if not v or v~=v or math.abs(v)==math.huge then valid=false end end
  if valid then gains={values[1],values[2],values[3]} end
end
local hoverPoints={}
if fs.exists("airship-hover.cfg") then
  local f=fs.open("airship-hover.cfg","r")
  for line in f.readAll():gmatch("[^\r\n]+") do
    local pressure,level,height=line:match("^([%-%.%d]+)%s+([%-%.%d]+)%s*([%-%.%d]*)$")
    pressure,level,height=tonumber(pressure),tonumber(level),tonumber(height)
    if pressure and level and level>=0 and level<=15 and height then
      hoverPoints[#hoverPoints+1]={height=height,level=level,pressure=pressure}
    end
  end
  f.close()
end
table.sort(hoverPoints,function(a,b) return a.height<b.height end)
local function hoverBias(height)
  if #hoverPoints==0 then return nil end
  if height<=hoverPoints[1].height then return hoverPoints[1].level end
  if height>=hoverPoints[#hoverPoints].height then return hoverPoints[#hoverPoints].level end
  for i=1,#hoverPoints-1 do
    local a,b=hoverPoints[i],hoverPoints[i+1]
    if height>=a.height and height<=b.height then
      if b.height==a.height then return b.level end
      return a.level+(b.level-a.level)*(height-a.height)/(b.height-a.height)
    end
  end
end
local mode=#hoverPoints==0 and "NO HEIGHT PID; BURNER HELD AT STARTUP LEVEL" or "HEIGHT CONTROL FROM SAVED BURNER MAP"
print(("Start Y %.1f, vertical speed %.2f, pressure %.4f"):format(startY,startVy,startPressure))
print(("Horizontal PID gains: %.4f %.5f %.3f; height uses calibrated burner amounts only."):format(gains[1],gains[2],gains[3]))
print(mode.."; saved hover-map points: "..#hoverPoints)
print("Test automatically pulses each thruster, steps altitude, then runs X+, X-, Z+, Z- autopilot moves.")
print("No speed, altitude-change, or distance cutoff is applied. GPS loss still stops navigation.")
print("Starting automatically. The Pastebin upload is public and contains relative X/Z only.")

local filename="airship-pid-diagnostic.csv"
local suffix=1
while fs.exists(filename) do suffix=suffix+1; filename="airship-pid-diagnostic-"..suffix..".csv" end
local log=assert(fs.open(filename,"w"),"Could not create diagnostic log")
log.writeLine("# Airship PID diagnostic v4; east-facing ship; relative GPS only; no absolute X/Z")
log.writeLine("# Detected thrusters: "..table.concat((function() local n={} for _,t in ipairs(thrusters) do n[#n+1]=t.name end return n end)(),","))
log.writeLine("# Footprint: "..footprint)
log.writeLine("# Balloon capacity: "..balloon.." blocks")
log.writeLine("# Burners: "..burnerCount.."; max fill each: "..fillPerBurner)
log.writeLine("# Combined usable fill estimate: "..math.min(balloon,burnerCount*fillPerBurner))
log.writeLine("# Mass/weight: "..mass)
log.writeLine("# Cargo/load: "..cargo)
log.writeLine("# Front direction: "..heading)
log.writeLine("# Horizontal PID gains: "..table.concat(gains,","))
log.writeLine("# Height control: burner map only; no altitude PID")
log.writeLine("# Hover map points: "..#hoverPoints)
log.writeLine("# Automatic movement limits: none; GPS loss stops the test")
log.writeLine("elapsed_s,phase,dt_s,target_y,altitude_y,vertical_speed,air_pressure,burner_signal,active_thruster,vector_x,vector_y,thrust_command,avg_thruster_power,thruster_read_count,relative_x,relative_z,velocity_x,velocity_z,target_relative_x,target_relative_z,heading_rad,yaw_rate_rad_s")
log.flush()

local state={x=x0,y=startY,z=z0,vx=0,vy=startVy,vz=0,pressure=startPressure}
local headingRad
local yawRate
local prevX,prevZ,prevAt=x0,z0,os.clock()
local lastGpsAt=prevAt
local targetY=startY
local originalBurner=redstone.getAnalogOutput("front") or 0
local burnerCommand=originalBurner
local burner=burnerCommand
local vectorX,vectorY,power=0,0,0
local activeThruster=nil
local targetRelX,targetRelZ
local started,lastLog,lastFlush=prevAt,prevAt,prevAt
local phase="START"
local aborted,abortReason=false,nil
local xy={p=gains[1],i=gains[2],d=gains[3],limit=.25,integral=0,integralLimit=20}
local zpid={p=gains[1],i=gains[2],d=gains[3],limit=.25,integral=0,integralLimit=20}
local function clamp(v,lo,hi) return math.max(lo,math.min(hi,v)) end
local function pidStep(c,e,v,dt)
  local candidate=clamp(c.integral+e*dt,-c.integralLimit,c.integralLimit)
  local raw=c.p*e+c.i*candidate-c.d*v
  local output=clamp(raw,-c.limit,c.limit)
  if raw==output or raw*e<0 then c.integral=candidate end
  return output
end
local function outputs()
  for _,t in ipairs(thrusters) do
    local tx,ty,tp=vectorX,vectorY,power
    if activeThruster and t.name~=activeThruster then tx,ty,tp=0,0,0 end
    local success,err=pcall(function() t.device.setVector(tx,ty); t.device.setThrustNormalized(tp) end)
    if not success then error("Thruster output failed: "..tostring(err)) end
  end
  local success,err=pcall(redstone.setAnalogOutput,"front",burner)
  if not success then error("Burner output failed: "..tostring(err)) end
end
local function readSensors()
  local now=os.clock()
  local dt=clamp(now-prevAt,.01,1.5); prevAt=now
  local success,y,vy,pressure=pcall(function()
    return sensor.getHeight(),sensor.getVerticalSpeed(),sensor.getAirPressure()
  end)
  if not success or type(y)~="number" or type(vy)~="number" or type(pressure)~="number" then error("Altitude sensor read failed") end
  state.y,state.vy,state.pressure=y,vy,pressure
  if navigation then
    local headingOk, heading = pcall(navigation.getHeadingRad)
    if headingOk and type(heading) == "number" then headingRad = heading end
  end
  if gimbal then
    local ratesOk, rates = pcall(gimbal.getAngularRatesRad)
    if ratesOk and type(rates) == "table" and type(rates[2]) == "number" then yawRate = rates[2] end
  end
  local x,_,z=gps.locate(.5,false)
  if x then
    state.vx=state.vx*.5+((x-prevX)/dt)*.5
    state.vz=state.vz*.5+((z-prevZ)/dt)*.5
    prevX,prevZ,lastGpsAt=x,z,now
    state.x,state.z=x,z
  elseif now-lastGpsAt>2 then error("GPS lost for over 2 seconds") end
  return now,dt
end
local function record(now,dt)
  if now-lastLog<.1 then return end
  lastLog=now
  local total,count=0,0
  for _,t in ipairs(thrusters) do
    local success,value=pcall(t.device.getPower)
    if success and type(value)=="number" then total=total+value; count=count+1 end
  end
  log.writeLine(("%.3f,%s,%.4f,%.3f,%.3f,%.4f,%.5f,%d,%s,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.4f,%.4f,%s,%s,%s,%s"):format(
    now-started,phase,dt,targetY,state.y,state.vy,state.pressure,burner,activeThruster or "ALL",vectorX,vectorY,power,count>0 and total/count or 0,count,
    state.x-x0,state.z-z0,state.vx,state.vz,
    targetRelX and ("%.3f"):format(targetRelX) or "",targetRelZ and ("%.3f"):format(targetRelZ) or "",
    headingRad and ("%.5f"):format(headingRad) or "",yawRate and ("%.5f"):format(yawRate) or ""))
  if now-lastFlush>=1 then log.flush(); lastFlush=now end
end
local function control(now,dt,autopilot)
  local mapped=hoverBias(targetY)
  if mapped then
    burnerCommand=burnerCommand+clamp(mapped-burnerCommand,-6*dt,6*dt)
    burner=math.floor(burnerCommand+.5)
  end
  if autopilot then
    local dx=targetRelX-(state.x-x0); local dz=targetRelZ-(state.z-z0)
    if type(headingRad) ~= "number" then error("Navigation table heading unavailable") end
    if math.abs(dx)<.5 and math.abs(dz)<.5 and math.abs(state.vx)<.12 and math.abs(state.vz)<.12 then vectorX,vectorY,power=0,0,0
    else
      local c,s=math.cos(headingRad),math.sin(headingRad)
      local localX,localZ=c*dx-s*dz,s*dx+c*dz
      local localVX,localVZ=c*state.vx-s*state.vz,s*state.vx+c*state.vz
      vectorX=-pidStep(zpid,localZ,localVZ,dt)
      vectorY=-pidStep(xy,localX,localVX,dt)
      power=clamp(math.sqrt(vectorX*vectorX+vectorY*vectorY)/.25*.35,0,.35)
    end
  else vectorX,vectorY,power=0,0,0 end
  outputs()
  record(now,dt)
end
local function resetIntegrals() xy.integral,zpid.integral=0,0 end
local function pulseThruster(t,vx,vy,axis)
  phase="THRUSTER_MAP_"..t.name.."_"..axis
  targetY=startY; activeThruster=t.name; vectorX,vectorY,power=vx,vy,.35
  local now,dt=readSensors()
  local pulseX,pulseZ=state.x,state.z
  local pulseStart=os.clock()
  while os.clock()-pulseStart<2.5 do
    now,dt=readSensors(); outputs(); record(now,dt); sleep(.05)
  end
  local dx,dz=state.x-pulseX,state.z-pulseZ
  log.writeLine(("# Thruster response: %s vector_%s dx=%.3f dz=%.3f"):format(t.name,axis,dx,dz))
  print(("%s vector_%s: relative movement X %.2f, Z %.2f"):format(t.name,axis,dx,dz))
  activeThruster=nil; vectorX,vectorY,power=0,0,0; outputs()
  local coastStart=os.clock()
  while os.clock()-coastStart<1 do now,dt=readSensors(); record(now,dt); sleep(.05) end
end
local function mapThrusters()
  print("Thruster map: individually pulsing each detected peripheral on both vector axes.")
  for _,t in ipairs(thrusters) do
    pulseThruster(t,.25,0,"X")
    pulseThruster(t,0,.25,"Y")
  end
  log.flush()
end
local function runPhase(name,duration,yTarget,xTarget,zTarget)
  phase=name; targetY=yTarget; targetRelX,targetRelZ=xTarget,zTarget; resetIntegrals()
  local phaseStart=os.clock()
  while os.clock()-phaseStart<duration do
    local now,dt=readSensors()
    control(now,dt,xTarget~=nil)
    if xTarget and os.clock()-phaseStart>=8 and
      math.abs(xTarget-(state.x-x0))<.5 and math.abs(zTarget-(state.z-z0))<.5 and
      math.abs(state.vx)<.12 and math.abs(state.vz)<.12 then return end
    sleep(.05)
  end
  if xTarget then error("Autopilot did not settle at "..name.." within "..duration.." seconds") end
end
local function runDiagnostic()
  mapThrusters()
  runPhase("ALT_HOLD_BASE",8,startY)
  runPhase("ALT_STEP_UP_3",25,startY+3)
  runPhase("ALT_RETURN_BASE_1",20,startY)
  runPhase("ALT_STEP_DOWN_3",25,startY-3)
  runPhase("ALT_RETURN_BASE_2",20,startY)
  -- Thruster mapping can move the ship; start navigation tests from its current position.
  x0,z0=state.x,state.z
  local axes={{"AUTO_X_PLUS",10,0},{"AUTO_X_MINUS",-10,0},{"AUTO_Z_PLUS",0,10},{"AUTO_Z_MINUS",0,-10}}
  for _,axis in ipairs(axes) do
    runPhase(axis[1],60,startY,axis[2],axis[3])
  end
end
local function waitForQ()
  while true do local _,key=os.pullEvent("key"); if key==keys.q then aborted=true; abortReason="Q pressed"; return end end
end
local ok,err=pcall(function() parallel.waitForAny(runDiagnostic,waitForQ) end)
if not ok then aborted=true; abortReason=tostring(err) end
vectorX,vectorY,power,burner=0,0,0,originalBurner
for _,t in ipairs(thrusters) do pcall(function() t.device.setVector(0,0); t.device.setThrustNormalized(0) end) end
pcall(redstone.setAnalogOutput,"front",originalBurner)
log.writeLine("# Finished: "..tostring(abortReason or (aborted and "aborted" or "complete")))
log.writeLine("# Burner signal restored to startup value: "..originalBurner)
log.flush(); log.close()
print("Saved "..filename.."; thrusters are off and burner signal restored to "..originalBurner..".")
if aborted then print("Diagnostic stopped: "..tostring(abortReason)) else print("Thruster survey, altitude steps, and four axis tests completed.") end
if http and http.checkURL and not http.checkURL("https://pastebin.com") then
  print("Pastebin HTTP unavailable; CSV remains local."); return
end
print("Uploading to public Pastebin...")
local uploaded,result=pcall(shell.run,"pastebin","put",filename)
if not uploaded or not result then print("Upload failed; send the local CSV instead.") end
