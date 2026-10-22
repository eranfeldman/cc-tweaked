-- Run from the repository root with Lua 5.2 or newer.
local file=assert(io.open("airship/airship.lua","r"))
local source=file:read("*a"):gsub("\r\n","\n"); file:close()
local core=assert(source:match("%-%- BEGIN AUTOPILOT CORE\n(.-)\n%-%- END AUTOPILOT CORE"))
local A=assert(load(core.."\nreturn Autopilot","@autopilot-core"))()
local names={"t0","t1","t2","t3","t4","t5","t6","t7"}
do
  local up={.3,math.sqrt(1-.3^2-.2^2),.2}
  local rates,gravity={},{}
  for i=1,3 do rates[i]=up[i]*.1; gravity[i]=-up[i]*9.81 end
  assert(math.abs(A.headingRate(rates,gravity)-.1)<1e-9,"Tilt must not contaminate measured heading rate")
  assert(A.headingRate({0,.12,0},nil)==.12,"Older sensors must retain body-rate fallback")
  assert(A.headingRate({0,.1,0},{0,0,-9.81})==nil,"A vertical bow has no usable horizontal heading")
  print("PASS: gravity-corrected gimbal heading rate and unavailable-gravity fallback")
end
local mounts={
  {-6,-6,0,-1},{-6,-6,-1,0},{6,-6,0,-1},{6,-6,1,0},
  {-6,6,0,1},{-6,6,-1,0},{6,6,0,1},{6,6,1,0},
}
local function plant(mass,period,noise,uneven)
  local p={t=0,x=0,z=0,vx=0,vz=0,heading=.4,yaw=0,pressure=.85,period=period,noise=noise or 0,uneven=uneven,filteredX=0,filteredZ=0,rock=0,rockRate=0,maxRock=0,actuators={}}
  function p:gpsX() return self.x+2*math.sin(self.rock)+self.noise*math.sin(self.t*3.7) end
  function p:gpsZ() return self.z-self.rock+self.noise*math.sin(self.t*4.1) end
  for _,name in ipairs(names) do p.actuators[name]={x=0,y=0,power=0} end
  function p:sample()
    return {t=self.t,x=self:gpsX(),z=self:gpsZ(),vx=self.filteredX,vz=self.filteredZ,
      heading=self.heading,yaw=self.yaw+self.noise*.1*math.sin(self.t*.6),pressure=self.pressure,tilt=math.abs(self.rock)}
  end
  function p:advance(commands)
    local oldX,oldZ=self:gpsX(),self:gpsZ()
    local count=math.ceil(self.period/.025)
    local dt=self.period/count
    for _=1,count do
      local ax,az,ayaw=0,0,0
      for i,name in ipairs(names) do
        local command=commands and commands[name] or {x=0,y=0,power=0}
        assert(command.power>=0 and command.power<=1.000001,"Throttle exceeds normalized range")
        assert(math.abs(command.x)<=1 and math.abs(command.y)<=1,"Invalid nozzle command")
        local a=self.actuators[name]
        local alpha=1-math.exp(-dt/.25)
        a.x=a.x+(command.x-a.x)*alpha; a.y=a.y+(command.y-a.y)*alpha
        a.power=a.power+(command.power-a.power)*alpha
        local r=mounts[i]
        local normalization=math.sqrt(1+(.57735*a.x)^2+(.57735*a.y)^2)
        local fx=(r[3]-r[4]*a.x*.57735)*a.power*1.5/mass/normalization
        local fz=(r[4]+r[3]*a.x*.57735)*a.power*1.5/mass/normalization
        ax,az=ax+fx,az+fz
        ayaw=ayaw+((r[2]-(uneven and 1.4 or 0))*fx-(r[1]-(uneven and -.9 or 0))*fz)/40
      end
      local c,s=math.cos(self.heading),math.sin(self.heading)
      if uneven then
        self.rockRate=self.rockRate+(.4*(ax-az)-2*self.rock-.45*self.rockRate)*dt
        self.rock=self.rock+self.rockRate*dt; self.maxRock=math.max(self.maxRock,math.abs(self.rock))
        ax,az=ax*math.cos(self.rock),az*math.cos(self.rock)
      end
      self.vx=self.vx+(c*ax+s*az-.18*self.pressure/.85*self.vx+(uneven and .012 or 0))*dt
      self.vz=self.vz+(-s*ax+c*az-.18*self.pressure/.85*self.vz+(uneven and -.008 or 0))*dt
      self.yaw=self.yaw+(ayaw-.25*self.pressure/.85*self.yaw+(uneven and .0015 or 0))*dt
      self.heading=self.heading+self.yaw*dt
      self.x,self.z=self.x+self.vx*dt,self.z+self.vz*dt
      self.t=self.t+dt
    end
    self.filteredX=.5*self.filteredX+.5*(self:gpsX()-oldX)/self.period
    self.filteredZ=.5*self.filteredZ+.5*(self:gpsZ()-oldZ)/self.period
  end
  return p
end

local profiles={}
local scenarios={{1,.2,0},{1.8,.35,0},{1.4,.3,.01},{1.4,.3,.18,true}}
for _,scenario in ipairs(scenarios) do
  local p=plant(table.unpack(scenario))
  local controller=A.new(names)
  assert(controller:begin(p:sample()))
  local commands={}
  local completed
  for _=1,5000 do
    p:advance(commands)
    local message
    commands,message,completed=controller:calibrate(p:sample())
    assert(commands,message)
    if completed then break end
  end
  assert(completed,"Automatic calibration never completed")
  assert(A.validProfile(completed,names),"Calibration saved an invalid profile")
  assert(completed.verification.overshoot<=.5,"An overshooting calibration was accepted")
  assert(#completed.channels==#names*4,"Not all detected thrusters and four nozzle directions were surveyed")
  assert(completed.maxSpeed>.75,"Fast calibration retained the old crawl speed")
  assert(completed.maxThrottle and completed.maxThrottle<=1,"Calibrated power must stay normalized")
  if scenario[4] then
    assert(p.maxRock>.005,"Uneven plant must actually rock under thrust")
    assert(math.abs(completed.drag[1]-.18)<.08 and math.abs(completed.drag[2]-.18)<.08,"Steady imbalance must not be mistaken for drag")
    assert(completed.maxThrottle==1,"Pitch/roll must not lower the calibrated power ceiling")
    print(("PASS: displaced COM, persistent force/torque, rocking %.3frad does not lower calibrated power"):format(p.maxRock))
  end
  profiles[#profiles+1]=completed
  print(("PASS: full calibration mass %.1f, sample %.2fs, duration %.1fs, peak overshoot %.3f"):format(scenario[1],scenario[2],p.t,completed.verification.overshoot))
end

do
  local candidate={}; for key,value in pairs(profiles[1]) do candidate[key]=value end
  candidate.gains={table.unpack(profiles[1].gains)}; candidate.validated=false
  local controller=A.new(names,profiles[1])
  local s={t=0,x=0,z=-4,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}
  assert(controller:begin(s))
  local j=controller.job
  j.phase,j.candidate,j.leg,j.overshoot="VERIFY",candidate,5,0
  j.goals={{x=4,z=0},{x=-4,z=0},{x=0,z=4},{x=0,z=-4},{x=0,z=0}}
  j.legStart={x=0,z=-4}
  controller.navigate=function() return {},false end
  local completed
  for i=1,35 do
    s.t=i*.2; s.x=(i%2==0) and .45 or -.45; s.z=0; s.vx=.05
    local commands,message
    commands,message,completed=controller:calibrate(s)
    if completed then break end
    assert(commands,message)
  end
  assert(completed and completed.validated,"Small bounded rocking/GPS variation should not reset the settle window")
  local moving=A.new(names,profiles[1])
  s={t=0,x=0,z=-4,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}
  assert(moving:begin(s))
  j=moving.job
  j.phase,j.candidate,j.leg,j.overshoot="VERIFY",candidate,5,0
  j.goals={{x=4,z=0},{x=-4,z=0},{x=0,z=4},{x=0,z=-4},{x=0,z=0}}
  j.legStart={x=0,z=-4}
  moving.navigate=function() return {},false end
  for i=1,12 do
    s.t=i*.2; s.x=.8; s.z=0; s.vx=.2
    moving:calibrate(s)
  end
  assert(moving.job and moving.job.stable==nil,"Large raw motion must never pass filtered settling")
  moving=A.new(names,profiles[1])
  s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}
  assert(moving:begin(s))
  j=moving.job
  j.phase,j.candidate,j.leg,j.overshoot="VERIFY",candidate,1,0
  j.goals={{x=0,z=0}}; j.legStart={x=-4,z=0}
  moving.navigate=function() return {},false end
  for i=1,15 do
    s.t=i*.2; s.x=-.6+s.t*.2; s.vx=.2
    local _,_,saved=moving:calibrate(s)
    assert(not saved,"Sustained drift through the destination must not pass calibration")
  end
  print("PASS: calibration validation tolerates bounded rocking while rejecting large movement")
end

do
  local controller=A.new(names,profiles[1])
  local captured
  controller.allocate=function(_,_,desired) captured=desired; return {} end
  local s={t=0,x=0,z=0,vx=.5,vz=0,heading=0,yaw=0,pressure=profiles[1].pressure,tilt=0}
  controller:navigate(s,{x=10,z=0})
  local dense=captured[1]
  controller:reset(0); s.pressure=s.pressure*.3
  controller:navigate(s,{x=10,z=0})
  assert(dense>captured[1],"Thin air must reduce learned drag compensation")
  local manual={}; for key,value in pairs(profiles[1]) do manual[key]=value end
  manual.gains={.04,0,0}
  controller:reset(0); s.vx=0; s.pressure=profiles[1].pressure
  controller:navigate(s,{x=10,z=0},manual)
  assert(captured[1]>0,"A valid manual D=0 setting must retain proportional steering")
  print("PASS: measured air pressure changes drag feedforward")
end

do
  local old=profiles[1]
  local candidate={}; for key,value in pairs(old) do candidate[key]=value end
  candidate.gains={table.unpack(old.gains)}; candidate.validated=false
  local controller=A.new(names,old)
  local s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}
  assert(controller:begin(s))
  local j=controller.job
  j.phase,j.candidate,j.leg,j.stable,j.overshoot="VERIFY",candidate,5,0,1
  j.verifyWindow={{t=-2,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}}
  j.goals={{x=4,z=0},{x=-4,z=0},{x=0,z=4},{x=0,z=-4},{x=0,z=0}}
  j.legStart={x=0,z=-4}
  controller.navigate=function() return {},true end
  local p=candidate.gains[1]
  s.t=2
  local commands,message=controller:calibrate(s)
  assert(commands and message:match("Retuning") and candidate.gains[1]<p and not candidate.validated)
  j.retry,j.leg,j.stable,j.overshoot=4,5,2,1
  j.verifyWindow={{t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}}
  s.t=4
  commands,message=controller:calibrate(s)
  assert(not commands and message:match("overshoots") and controller.profile==old,"Excessive overshoot must never replace the working profile")
  print("PASS: overshoot triggers automatic retuning and rejects failed validation")
end

for i,profile in ipairs(profiles) do
  local p=plant(table.unpack(scenarios[i]))
  p.pressure=.3; p.heading=2.9; p.yaw=.08
  local controller=A.new(names,profile)
  controller:reset(p.heading)
  local goal={x=7,z=-5}
  local arrived=false
  for _=1,1500 do
    local commands
    commands,arrived=controller:navigate(p:sample(),goal)
    assert(commands)
    p:advance(commands)
    if arrived then break end
  end
  assert(arrived,"Navigation failed after changing heading and air pressure")
  assert(math.abs(p.yaw)<.035,"Yaw was not controlled")
  for _=1,100 do p:advance(controller:navigate(p:sample(),goal)) end
  assert(math.sqrt((p.x-goal.x)^2+(p.z-goal.z)^2)<.4,"Destination station keeping drifted")
  print("PASS: reduced pressure, rotated ship, yaw correction, arrival and station keeping")
end

for i,profile in ipairs(profiles) do
  for _,pressure in ipairs({.85,.3}) do
    for _,length in ipairs({100,1000}) do
      local p=plant(table.unpack(scenarios[i])); p.pressure=pressure; p.heading=2.9
      local controller=A.new(names,profile); controller:reset(p.heading)
      local sign=pressure==.85 and 1 or -1
      local goal=length==100 and {x=sign*length,z=0} or {x=0,z=sign*length}
      local peak,overshoot=0,0
      local arrived=false
      for _=1,10000 do
        local commands,done=controller:navigate(p:sample(),goal)
        assert(commands)
        for _,out in pairs(commands) do assert(out.power<=profile.maxThrottle+.000001,"Mixer exceeded tested power") end
        p:advance(commands)
        peak=math.max(peak,math.sqrt(p.vx^2+p.vz^2))
        overshoot=math.max(overshoot,(p.x*goal.x+p.z*goal.z)/length-length)
        if done then arrived=true; break end
      end
      assert(arrived,"Fast cruise never arrived")
      assert(p.t<(length==100 and 140 or 600),"Fast cruise regressed to crawl speed")
      assert(overshoot<.5,"Fast cruise failed the braking tolerance")
      local travelTime=p.t
      for _=1,100 do p:advance(controller:navigate(p:sample(),goal)) end
      assert(math.sqrt((p.x-goal.x)^2+(p.z-goal.z)^2)<.4,"Fast arrival failed to hold its destination")
      print(("PASS: fast %dm, mass %.1f, pressure %.2f, %.1fs, peak %.2fm/s, overshoot %.3fm"):format(length,scenarios[i][1],pressure,travelTime,peak,overshoot))
    end
  end
end

do
  local controller=A.new(names,profiles[1])
  local s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,tilt=0}
  local level=controller:navigate(s,{x=10,z=0})
  local tilted=A.new(names,profiles[1])
  s.t,s.tilt=.2,.5
  local commands=tilted:navigate(s,{x=10,z=0})
  assert(commands and level,"Autopilot must not require tilt readings")
  for _,name in ipairs(names) do
    assert(math.abs(commands[name].power-level[name].power)<1e-9,"Pitch/roll must not reduce commanded thrust")
  end
  s.t=.4; s.tilt=nil
  assert(tilted:navigate(s,{x=1,z=0}),"Autopilot must continue without tilt readings")
  local invalid={}; for key,value in pairs(profiles[1]) do invalid[key]=value end
  invalid.maxThrottle=2
  assert(not A.validProfile(invalid,names),"Out-of-range saved throttle must be rejected")
  local legacy={}; for key,value in pairs(profiles[1]) do legacy[key]=value end
  legacy.maxThrottle,legacy.tiltReference=nil,nil
  assert(A.validProfile(legacy,names),"Old saved calibrations must remain readable")
  local oldCommands=controller:allocate(legacy,{10,10,.1})
  for _,out in pairs(oldCommands) do assert(out.power<=.350001,"Old profiles must retain their original throttle limit") end
  print("PASS: tilt does not limit flight thrust or require a reading; invalid power is rejected")
end

do
  local files={["airship-autopilot.cfg"]="old profile"}
  local fail=false
  local env=setmetatable({Autopilot=A,navNames=names}, {__index=_G})
  env.textutils={serialize=function() return "new profile" end}
  env.fs={
    exists=function(path) return files[path]~=nil end,
    delete=function(path) files[path]=nil end,
    open=function(path) return {write=function(value) files[path]=value end,close=function() end} end,
    move=function(from,to)
      if fail and from:match("%.tmp$") then error("disk full") end
      assert(files[from] and not files[to]); files[to],files[from]=files[from],nil
    end,
  }
  local saving=assert(source:match("(local function saveAutopilotProfile%b()%s*.-)\nif fs.exists%(\"airship%-autopilot.cfg\"%)"))
  local save=assert(load(saving.."\nreturn saveAutopilotProfile","@profile-save","t",env))()
  save(profiles[1])
  assert(files['airship-autopilot.cfg']=="new profile" and files['airship-autopilot.cfg.bak']=="old profile")
  fail=true
  assert(not pcall(save,profiles[1]))
  assert(files['airship-autopilot.cfg']=="new profile","Failed installation must restore the existing calibration")
  local corrupt={version=1,validated=true}
  assert(not A.validProfile(corrupt,names),"Incomplete profiles must be rejected")
  print("PASS: atomic profile save, backup restoration and corrupt profile rejection")
end

do
  local p=plant(1,.2)
  local old=profiles[1]
  local controller=A.new(names,old)
  assert(controller:begin(p:sample()))
  for _=1,2500 do
    p.t=p.t+.2
    local commands,message=controller:calibrate(p:sample())
    if not commands then
      assert(message:match("response too small"))
      break
    end
  end
  assert(controller.job==nil and controller.profile==old,"Failed calibration replaced the previous profile")
  assert(controller:begin(p:sample()))
  local invalid=p:sample(); invalid.heading=0/0
  local commands=controller:calibrate(invalid)
  assert(commands==nil and controller.profile==old,"Invalid sensors did not abort calibration")
  assert(controller:begin(p:sample()))
  controller:cancel()
  assert(controller.job==nil and controller.profile==old,"Cancel must preserve existing calibration")
  local changed={table.unpack(names)}; changed[1]="remapped"
  assert(A.new(changed,old).profile==nil,"Changed thrusters must require recalibration")
  print("PASS: missing response, invalid sensors, cancellation and changed thrusters retain or reject profiles correctly")
end

do
  local profile=profiles[1]
  local env=setmetatable({Autopilot=A}, {__index=_G})
  env.colors={red=1,orange=2,lime=3}
  env.position,env.altitude,env.mode={x=0,y=73,z=0},73,"HOLDING ALTITUDE"
  env.burner,env.burnerCommand,env.sampleDt,env.burnerSlewPerSecond=6,6,.2,6
  env.status={}; env.shipHeading=.4
  env.navController=A.new(names,profile)
  local sample={t=0,x=0,z=0,vx=0,vz=0,heading=.4,yaw=0,pressure=.85,tilt=0}
  env.navigationSample=function() return sample end
  env.clamp=function(v,a,b) return math.max(a,math.min(b,v)) end
  env.hoverBias=function() error("Full calibration must keep the burner fixed") end
  env.saveAutopilotProfile=function(p) assert(p.validated); env.saved=p end
  env.applyGainValues=function(g) env.applied=g end
  local start=assert(source:match("(local function startFullCalibration%(%)%s*.-)\nlocal function testVector"))
  local update=assert(source:match("(local function updateFlight%b()%s*.-)\nlocal function savedHoverPoints"))
  local startFn,updateFn=assert(load(start.."\n"..update.."\nreturn startFullCalibration,updateFlight","@integration","t",env))()
  startFn()
  assert(env.fullCalibration and env.calibrationRunning and env.manualBurner==6 and env.navController.job)
  sample.t=.2
  updateFn(true)
  assert(env.burner==6 and env.burnerCommand==6,"Calibration changed burner output")
  env.navController.calibrate=function() return {},nil,profile end
  updateFn(true)
  assert(env.saved==profile and env.navController.profile==profile and env.applied==profile.gains)
  assert(not env.fullCalibration and not env.calibrationRunning and env.burner==6,"Completion must leave height control alone")
  startFn()
  env.saveAutopilotProfile=function() error("disk full") end
  local previous=env.navController.profile
  local failedCandidate={gains={.01,0,.3},validated=true}
  env.navController.calibrate=function() return {},nil,failedCandidate end
  updateFn(true)
  assert(env.navController.profile==previous and env.autopilotNotice:match("could not save"),"Save failure replaced the working profile")
  env.mode,env.target="AUTOPILOT",{x=10,z=0}; sample.tilt=nil
  updateFn(true)
  assert(env.mode=="AUTOPILOT","Gyro-handled tilt must not stop navigation")
  sample.yaw=nil
  updateFn(true)
  assert(env.mode=="NAV LOST - RE-ENGAGE" and next(env.autopilotCommands)==nil and env.autopilotNotice:match("yaw"),"Missing yaw rate must stop navigation visibly")
  print("PASS: controller button, fixed burner output, calibration completion and failed-save rollback")
end
