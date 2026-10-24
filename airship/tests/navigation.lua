-- Run from the repository root with Lua 5.2 or newer.
local file=assert(io.open("airship/airship.lua","r"))
local source=file:read("*a"):gsub("\r\n","\n"); file:close()
local core=assert(source:match("%-%- BEGIN AUTOPILOT CORE\n(.-)\n%-%- END AUTOPILOT CORE"))
local A=assert(load(core:gsub('  function A.validSample','  A.testWindow=window\n  function A.validSample',1).."\nreturn Autopilot","@autopilot-core"))()
local names={"t0","t1","t2","t3","t4","t5","t6","t7"}
do
  local env=setmetatable({previousPosition={x=0,y=20,z=0},previousSample=0,velocity={x=0,y=0,z=0},intervalVelocity={}},{__index=_G})
  env.os={clock=function() return 3 end}
  env.FreshGPS={locate=function() return 6,20,-3 end,accept=function() return true end}
  env.navController={}; env.gpsFilter={}
  env.colors={orange=1,red=2}
  local locating=assert(source:match('(local function locate%(%)%s*.-)\nlocal function updateFlight'))
  local locate=assert(load(locating..'\nreturn locate',"@slow-gps-velocity","t",env))()
  assert(locate() and env.sampleDt==3 and env.velocity.x==1 and env.velocity.z==-.5,
    "Slow peripheral cycles must use actual GPS elapsed time instead of inflating velocity")
  assert(env.intervalVelocity.x==2 and env.intervalVelocity.z==-1,"Braking must retain GPS interval velocity without the additional EMA delay")
  env.os.clock=function() return 10 end
  env.FreshGPS.locate=function() return nil end; env.FreshGPS.error="GPS host ranges disagree"
  env.burner,env.burnerCommand,env.mode=6,6,"AUTOPILOT"
  env.navController.cancel=function() end
  assert(not locate() and env.position==nil and env.power==0 and env.burner==6 and env.manualBurner==6,
    "Invalid GPS must disengage navigation without shutting down the holding burners")
  print("PASS: three-second peripheral cycle preserves correct GPS velocity units")
end
do
  local samples={{t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=1}}
  local vx,vz=0,0
  for i=1,300 do
    local heading=(i-.5)*.02
    vx=vx+(math.cos(heading)*.7-math.sin(heading)*.3)*.02
    vz=vz+(-math.sin(heading)*.7-math.cos(heading)*.3)*.02
    samples[#samples+1]={t=i*.02,x=0,z=0,vx=vx,vz=vz,heading=i*.02,yaw=1}
  end
  local measured=A.testWindow(samples)
  assert(math.abs(measured.a[1]-.7)<1e-6 and math.abs(measured.a[2]+.3)<1e-6,
    "Survey must recover body thrust while heading turns through almost a full circle")
  print("PASS: rotating survey retains correct body acceleration instead of canceling world-space forces")
end
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
    return {t=self.t,x=self:gpsX(),y=self.y or 227,z=self:gpsZ(),vx=self.filteredX,vz=self.filteredZ,rawVx=self.rawX or 0,rawVz=self.rawZ or 0,
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
        local alpha=1-math.exp(-dt/(self.actuatorDelay or .25))
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
    self.rawX,self.rawZ=(self:gpsX()-oldX)/self.period,(self:gpsZ()-oldZ)/self.period
    self.filteredX=.5*self.filteredX+.5*self.rawX
    self.filteredZ=.5*self.filteredZ+.5*self.rawZ
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
  for _=1,10000 do
    p:advance(commands)
    local message
    commands,message,completed=controller:calibrate(p:sample())
    assert(commands,message)
    if completed then break end
  end
  assert(completed,"Automatic calibration never completed")
  assert(A.validProfile(completed,names),"Calibration saved an invalid profile")
  assert(type(completed.verification.overshoot)=="number" and completed.verification.overshoot>=0,"Settled overshoot must remain available in the calibration report")
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
  local candidate={}; for k,v in pairs(profiles[1]) do candidate[k]=v end
  candidate.validated=nil; candidate.cruiseVersion=nil
  candidate.maxSpeed=17; candidate.gains={.09,.001,.66}
  local resumed=A.new(names)
  local s={t=10,x=5,z=-3,vx=0,vz=0,heading=.4,yaw=.05,pressure=.85}
  assert(resumed:resumeVerification(s,candidate) and resumed.job.phase=="VERIFY","Saved thruster model must resume at auto tests")
  local recovered=A.cruiseSpeed(candidate)
  assert(candidate.maxSpeed==recovered and candidate.gains[1]==.0324,"Resume must use measured speed while bounding gains")
  local legacy={}; for k,v in pairs(profiles[1]) do legacy[k]=v end
  legacy.maxSpeed=2; legacy.cruiseVersion=nil
  local legacyResume=A.new(names)
  assert(legacyResume:resumeVerification(s,legacy) and legacy.maxSpeed==recovered,"Resume must recover speed from the old 8 b/s cap using measured acceleration and drag")
  legacy.maxSpeed=2
  assert(legacyResume:resumeVerification(s,legacy) and legacy.maxSpeed==2,"New controller must preserve its own failed-braking speed reduction")
  local ratcheted={}; for k,v in pairs(profiles[1]) do ratcheted[k]=v end
  ratcheted.validationVersion=nil; ratcheted.maxSpeed=2; ratcheted.responseScale=2
  assert(A.new(names):resumeVerification(s,ratcheted) and ratcheted.maxSpeed==A.cruiseSpeed(ratcheted) and ratcheted.responseScale==1,
    "Upgrade must reuse the survey and discard speed cuts caused by the old retry loop")
  legacy.gpsVersion=nil
  assert(not legacyResume:resumeVerification(s,legacy),"Unchecked-GPS measurements must not resume as a validated new survey")
  s.t=10.2; s.yaw=1
  local safeCommands,message=resumed:calibrate(s)
  assert(safeCommands and not next(safeCommands) and message:match("paused") and resumed.job.phase=="VERIFY",
    "Runaway yaw must cut thrusters and retain the validation checkpoint")
  local yawController=A.new(names,profiles[1]); yawController:reset(0)
  local yawSample={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=.05,pressure=.85}
  local highDrag={}; for k,v in pairs(profiles[1]) do highDrag[k]=v end
  highDrag.drag={profiles[1].drag[1],profiles[1].drag[2],3}
  assert(A.validProfile(highDrag,names))
  yawController:navigate(yawSample,{x=0,z=0},highDrag)
  assert(yawController.applied[3]<0,"Yaw drag compensation must not amplify an existing fast spin")
  print("PASS: saved 96-test model resumes auto tests only; high yaw drag retains negative damping")
end

do
  local controller=A.new(names)
  local s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  assert(controller:begin(s))
  local commands,message=controller:calibrate(nil)
  assert(commands and controller.job and message:match("Waiting for fresh"),"A missing live sample must pause, not cancel, calibration")
  s.t=2.5
  commands,message=controller:calibrate(s)
  assert(commands and controller.job and controller.job.phase=="BASELINE" and message:match("BASELINE"),
    "Normal slow hub polling must continue the active baseline")
  s.t=11
  commands,message=controller:calibrate(s)
  assert(commands and controller.job and controller.job.phase=="BASELINE" and controller.job.index==1,
    "A long sensor gap must restart only the active thruster baseline")
  assert(message:match("repeating thruster 1/32"))
  controller.job.phase="PULSE"; controller.job.interrupted=true; s.t=11.2
  commands,message=controller:calibrate(s)
  assert(commands and controller.job.phase=="BASELINE","A pulse interrupted by bad GPS must restart rather than contaminate the survey")
  print("PASS: navigation sample loss pauses calibration and a long gap restarts the active baseline")
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
  local oldNames,oldMounts=names,mounts
  names,mounts={},{}
  local profile={}; for k,v in pairs(profiles[1]) do profile[k]=v end
  profile.names,profile.channels={},{}
  for i=1,24 do
    local name=("t%02d"):format(i)
    names[i],profile.names[i]=name,name
    local sourceIndex=math.floor((i-1)/3)+1
    mounts[i]=oldMounts[sourceIndex]
    for d=1,4 do
      local ch=profiles[1].channels[(sourceIndex-1)*4+d]
      profile.channels[#profile.channels+1]={name=name,x=ch.x,y=ch.y,b={ch.b[1]/3,ch.b[2]/3,ch.b[3]/3}}
    end
  end
  profile.maxSpeed=2
  assert(A.validProfile(profile,names))
  local p=plant(3,.3,.01)
  p.heading,p.vx,p.vz=2.5,-2,2
  local controller=A.new(names,profile); controller:reset(p.heading)
  local goal={x=400,z=300}
  local arrived,peak,cross= false,0,0
  for _=1,4000 do
    if p.t>20 then p.pressure=.3 end
    local commands,done=controller:navigate(p:sample(),goal)
    p:advance(commands)
    peak=math.max(peak,math.sqrt(p.vx^2+p.vz^2))
    cross=math.max(cross,math.abs(-.6*p.x+.8*p.z))
    if done then arrived=true; break end
  end
  assert(arrived,"24-thruster diagonal route failed to arrive after a pressure change")
  assert(peak>13,"A stored cruise estimate must not cap available flight speed")
  assert(cross<6,"Sideways initial velocity caused a curved or divergent route")
  print(("PASS: 24 thrusters, diagonal, initial sideways drift, changing pressure; %.2fm/s peak, %.2fm cross-track, %.1fs"):format(peak,cross,p.t))
  assert(controller:beginBraking(p:sample()))
  local commands,measured={},nil
  for _=1,5000 do
    p:advance(commands)
    local message
    commands,message,measured=controller:calibrate(p:sample())
    assert(commands,message)
    if measured then break end
  end
  assert(measured and #measured.channels==96 and A.validBraking(measured.braking),'24-thruster braking must reuse the complete measured channel layout')
  local flight=plant(3,.65,.01); flight.pressure=.85; flight.heading=1.3
  local pilot=A.new(names,measured); pilot:reset(flight.heading)
  local goal={x=800,z=600}; local done,over=false,0
  for _=1,3000 do
    if .8*flight.x+.6*flight.z>400 then flight.pressure=.3; flight.y=300 end
    local out,reached=pilot:navigate(flight:sample(),goal); flight:advance(out)
    over=math.max(over,.8*flight.x+.6*flight.z-1000)
    if reached then done=true; break end
  end
  print(('BRAKING SIM 24 thrusters, 0.65s polls, diagonal, pressure/height change: arrived %s overshoot %.2f time %.1fs'):format(tostring(done),over,flight.t))
  assert(done and over<.5,'24-thruster measured braking must handle diagonal travel, slower polling and live pressure changes')
  names,mounts=oldNames,oldMounts
end

do
  local profile={}; for k,v in pairs(profiles[1]) do profile[k]=v end
  profile.maxAccel,profile.maxSpeed,profile.delay,profile.responseScale=6,13,2,1
  local controller=A.new(names,profile); controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=13,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=60,z=0})
  local oldTarget=math.sqrt((6*.6*2)^2+2*6*.6*(60-4))-6*.6*2
  assert(controller.trace.targetAlong<13 and controller.trace.targetAlong>oldTarget*.75,
    "Braking must start earlier without replacing fast arrival with a crawl")
  local previousDelay=2+.85*1.3
  local previousTarget=math.sqrt((3.3*previousDelay)^2+2*3.3*(60-3))-3.3*previousDelay
  assert(controller.trace.targetAlong>previousTarget,"Braking onset must move slightly later than v1.6.11")
  controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=14,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=60,z=0})
  local correction=controller.trace.desired[1]-profile.drag[1]*14
  assert(math.abs(correction-(controller.trace.targetAlong-14)/(1.3*.5))<1e-6,
    "Unsaturated high-speed braking must respond more strongly to the velocity error")
  controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=13,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=10000,z=0})
  assert(controller.trace.targetAlong>13 and controller.trace.desired[1]>=profile.maxAccel,
    "Long routes must still request full acceleration beyond the stored cruise estimate")
  controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=20,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=60,z=0})
  assert(controller.trace.desired[1]-profile.drag[1]*20<=-5.9,
    "Fast closing speed near the braking boundary must request strong net deceleration")
  profile.responseScale=2
  controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=6,z=0})
  assert(controller.trace.targetAlong>.65 and controller.trace.desired[1]>.25,
    "A cautious profile six blocks out must retain useful approach speed instead of blending into weak position hold")
  controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=13,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=10000,z=0})
  assert(controller.trace.targetAlong>13,"Braking caution must not impose a fixed forward cruise cap")
  profile.cruiseVersion=2
  controller=A.new(names,profile); controller:reset(0)
  controller:navigate({t=0,x=0,z=0,vx=13,vz=0,heading=0,yaw=0,pressure=profile.pressure},{x=60,z=0})
  assert(controller.trace.targetAlong<=13,"Working legacy profile must retain its speed and braking limits until new validation passes")
  print("PASS: modestly earlier braking retains full long-route acceleration without a fixed cruise cap")
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
  local commands,message,saved=controller:calibrate(s)
  assert(commands and message==nil and saved==candidate and candidate.gains[1]==p and candidate.responseScale==1 and candidate.validated,
    "A settled test must pass despite overshoot without retuning cruise response")
  assert(not controller.job and candidate.verification.overshoot==1,
    "Calibration must retain overshoot as a measurement and save the settled profile")
  print("PASS: settled overshoot is recorded without repeating or slowing calibration")
end

do
  local old=profiles[1]
  local function fixture()
    local candidate={}; for k,v in pairs(old) do candidate[k]=v end
    candidate.gains={table.unpack(old.gains)}; candidate.validated=nil
    local controller=A.new(names,old)
    local s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
    assert(controller:resumeVerification(s,candidate))
    local j=controller.job
    j.goals={{x=10,z=0},{x=20,z=0},{x=30,z=0},{x=40,z=0},{x=0,z=0}}
    controller.navigate=function(_,sample,goal)
      j.lastGoal=goal
      return {},false
    end
    return controller,j,s,candidate
  end
  local function settle(controller,j,s,x)
    s.x=x; s.vx=0; s.t=s.t+2
    j.at=s.t; j.stable=s.t-2
    j.verifyWindow={{t=s.t-4,x=x,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}}
    return controller:calibrate(s)
  end
  local c,j,s,candidate=fixture()
  j.leg=3; j.legStart={x=20,z=0}; j.overshoot=.8
  local _,reason=settle(c,j,s,30)
  assert(j.leg==4 and not j.returning and j.retry==0 and j.maxOvershoot==.8,'A settled overshooting leg must advance without a repeat')
  settle(c,j,s,40)
  local _,_,saved=settle(c,j,s,0)
  assert(saved==candidate and saved.verification.tests==5 and saved.verification.retries==0 and not c.job,'Passing test 5 must finish once without repeating overshoot legs')

  c,j,s,candidate=fixture(); j.leg=3; j.legStart={x=20,z=0}
  local speed,response,p=candidate.maxSpeed,candidate.responseScale,candidate.gains[1]
  s.t=200; s.x=22; s.vx=.19; j.previous.t=199.8
  j.verifyWindow={{t=196,x=21.24,z=0,vx=.19,vz=0,heading=0,yaw=0,pressure=.85}}
  local _,message=c:calibrate(s)
  assert(j.leg==3 and j.returning and j.retry==1 and message:match('timeout'),'Slow arrival must show a timeout and retry the current test')
  assert(candidate.maxSpeed==speed and candidate.responseScale==response and candidate.gains[1]>p,'Slow arrival must strengthen hold without cutting cruise speed')
  s.t=200.2
  _,message=c:calibrate(s)
  assert(message:match('RETURN 3/5') and message:match('timeout'),'The retry reason must remain visible during the return flight')

  c,j,s=fixture()
  s.t=.2; s.x=10.6; c:calibrate(s)
  assert(j.overshoot<=.5,'One small GPS wobble must not fail the whole calibration')
  s.t=.4; s.x=10; c:calibrate(s)
  s.t=.6; s.x=10.6; c:calibrate(s)
  s.t=1; c:calibrate(s)
  assert(j.overshoot>.5,'Sustained overshoot must still be measured for reporting')
  c,j,s=fixture(); s.t=.2; s.x=12.1; c:calibrate(s)
  assert(j.overshoot>2,'A large overshoot must be retained even from one sample')
  c,j,s=fixture(); s.t=.2; s.x=10.6; c:calibrate(s)
  s.t=9; c:calibrate(s)
  assert(j.overAt==nil and j.legStart.x==0,'A navigation gap must preserve run-up geometry but reset the overshoot duration evidence')
  s.t=9.2; c:calibrate(s)
  assert(j.overshoot<=.5,'Time without navigation samples must not count as sustained overshoot')
  c,j,s=fixture(); s.t=.2; s.x=11.9; c:calibrate(s)
  s.t=.6; s.x=10.6; c:calibrate(s)
  assert(math.abs(j.overshoot-1.9)<1e-8,'Confirming sustained overshoot must retain its initial peak, not only the later smaller sample')
  for _,case in ipairs({{0,0,10,0},{0,0,-10,0},{0,0,0,-10},{1000000,-2000000,1000030,-1999960}}) do
    c,j,s=fixture()
    j.legStart={x=case[1],z=case[2]}; j.goals[1]={x=case[3],z=case[4]}
    local dx,dz=case[3]-case[1],case[4]-case[2]; local len=math.sqrt(dx*dx+dz*dz); dx,dz=dx/len,dz/len
    s.x,s.z=case[3]+dx*1.2-dz*9,case[4]+dz*1.2+dx*9
    s.t=.2; c:calibrate(s); s.t=.6; c:calibrate(s)
    assert(math.abs(j.overshoot-1.2)<1e-7 and math.abs(j.cross-9)<1e-7 and math.abs(j.remaining+1.2)<1e-7,'Overshoot must be signed along the leg, independent of sideways distance, heading and world origin')
    c,j,s=fixture(); j.legStart={x=case[1],z=case[2]}; j.goals[1]={x=case[3],z=case[4]}
    s.x,s.z=case[3]-dx*2-dz*9,case[4]-dz*2+dx*9
    s.t=.2; c:calibrate(s); s.t=.6; c:calibrate(s)
    assert(j.overshoot==0 and math.abs(j.remaining-2)<1e-7,'Being off the route before the destination must not count as overshoot')
  end
  print('PASS: timeouts retain completed legs, while settled overshoot is measured without repeating the leg')
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

do
  local function turn(fast,start,target)
    local p=plant(1,.2); p.heading=math.rad(start)
    local controller=A.new(names,profiles[1]); controller:reset(p.heading)
    if fast then controller:setHeadingTarget(math.rad(target)) else controller.headingTarget=math.rad(target) end
    local excursion=0
    for _=1,1000 do
      local commands,arrived=controller:navigate(p:sample(),{x=0,z=0})
      if arrived then
        local duration=p.t
        for _=1,50 do
          p:advance(controller:navigate(p:sample(),{x=0,z=0}))
          excursion=math.max(excursion,math.sqrt(p.x^2+p.z^2))
        end
        assert(not fast or math.abs((p.heading-math.rad(target)+math.pi)%(2*math.pi)-math.pi)<.03,
          "Final alignment must remain settled after reaching its heading")
        controller:reset(p.heading)
        assert(not controller.finalTurn,"Fast final-turn control must reset before the next flight")
        return duration,excursion
      end
      p:advance(commands); excursion=math.max(excursion,math.sqrt(p.x^2+p.z^2))
    end
    error('Final cardinal turn did not settle')
  end
  for _,angles in ipairs({{44,0},{46,90},{350,0}}) do
    local slow=turn(false,table.unpack(angles))
    local fast,excursion=turn(true,table.unpack(angles))
    assert(fast<slow*.65,"Final turn must settle substantially faster than travel heading control")
    assert(excursion<.75,"Faster final turn must keep the destination in place")
    print(("PASS: final turn %d to %d degrees, %.1fs vs %.1fs; %.3fm drift"):format(angles[1],angles[2],fast,slow,excursion))
  end
end

local brakingProfiles={}
for i=1,3 do
  local p=plant(table.unpack(scenarios[i])); p.y=227; p.pressure=i==3 and .3 or .85
  local candidate
  if i==3 then
    candidate={}; for k,v in pairs(profiles[i]) do candidate[k]=v end; candidate.validated=nil
  end
  local controller=A.new(names,profiles[i]); assert(controller:beginBraking(p:sample(),candidate))
  local commands,completed,lastMessage={},nil,nil
  for _=1,5000 do
    p:advance(commands)
    local message
    commands,message,completed=controller:calibrate(p:sample()); lastMessage=message
    assert(commands,message)
    if completed then break end
  end
  assert(completed and A.validBraking(completed.braking),('Eight automatic stopping tests must produce a valid measured model: index %s phase %s time %.1f: %s'):format(tostring(controller.job and controller.job.index),tostring(controller.job and controller.job.phase),p.t,tostring(lastMessage)))
  assert(completed.braking.height==227 and completed.braking.pressure==p.pressure,'Brake calibration must retain live height and pressure')
  assert(controller.profile==profiles[i] and profiles[i].braking==nil,'Measuring brakes must not overwrite the working profile before persistence succeeds')
  for _,d in ipairs(completed.braking.directions) do assert(#d.runs==2 and d.force>0 and d.delay>=.05) end
  print(('PASS: measured 8 stops mass %.1f pressure %.2f, %.1fs; force %.2f drag %.3f delay %.2f'):format(scenarios[i][1],p.pressure,p.t,completed.braking.directions[1].force,completed.braking.directions[1].drag,completed.braking.directions[1].delay))
  if candidate then
    assert(not completed.validated and A.validCandidate(completed,names) and A.new(names,completed).profile==nil,'Brake calibration must accept the pending survey without enabling normal AUTO')
    assert(candidate.braking==nil and #completed.channels==#candidate.channels,'Braking measurement must preserve the original survey until saved')
    local braking=completed.braking
    assert(controller:resumeVerification(p:sample(),completed) and controller.job.phase=='VERIFY')
    completed=nil; commands={}
    local travelled,direct,truthOvershoot,previousLeg=0,0,0,nil
    for _=1,10000 do
      local j=controller.job; local goal=j.goals[j.leg]
      local dx,dz=goal.x-j.legStart.x,goal.z-j.legStart.z; local length=math.sqrt(dx*dx+dz*dz)
      if previousLeg~=j.leg then direct=direct+length; previousLeg=j.leg end
      local x,z=p.x,p.z
      p:advance(commands)
      travelled=travelled+math.sqrt((p.x-x)^2+(p.z-z)^2)
      if length>.01 then truthOvershoot=math.max(truthOvershoot,((p.x-goal.x)*dx+(p.z-goal.z)*dz)/length) end
      local message
      commands,message,completed=controller:calibrate(p:sample())
      assert(commands,message)
      if completed then break end
    end
    assert(completed and A.validProfile(completed,names) and completed.verification.tests==5 and completed.braking==braking,'All five flight tests must use the pending brake model and validate without repeating the survey')
    assert(math.abs(completed.verification.overshoot-truthOvershoot)<.05,'Flight validation overshoot must agree with the simulated physical position')
    assert(travelled<direct*1.02,'Five settled flight legs must follow their direct routes without significant detours')
    print(('PASS: five-leg travelled/direct %.4f, scored/physical overshoot %.3f/%.3f'):format(travelled/direct,completed.verification.overshoot,truthOvershoot))
    print(('PASS: pending survey -> eight stops -> five flight tests, overshoot %.3f, retries %d'):format(completed.verification.overshoot,completed.verification.retries))
  end
  brakingProfiles[i]=completed
end

for _,settings in ipairs({{.3,.25},{.65,.8},{1,1.2}}) do
  local p=plant(1.4,settings[1],.01); p.actuatorDelay=settings[2]
  local c=A.new(names,profiles[3]); assert(c:beginBraking(p:sample()))
  local commands,completed={},nil
  local recorded,rebound,low,interrupted=0,0,false,false
  for _=1,4000 do
    p:advance(commands)
    local s=p:sample(); local speed=math.sqrt(s.rawVx^2+s.rawVz^2)
    local j=c.job; local index=j.index
    if j.phase=='TAIL' or j.phase=='SETTLE' then
      if speed<.2 then low=true end
      if low then rebound=math.max(rebound,speed) end
    else low=false end
    local message; commands,message,completed=c:calibrate(s)
    assert(commands,message)
    if c.job and c.job.index>index then
      assert(speed<=.2,'Brake calibration must not record a stop while sliding backward or sideways')
      recorded=recorded+1
    end
    if settings[1]==.65 and c.job and c.job.index==2 and not interrupted then
      interrupted=true
      local first=c.job.runs[1]
      for _=1,16 do p:advance({}) end
      c.job.interrupted=true
      assert(c.job.runs[1]==first and c.job.index==2,'A navigation pause must retain the first actual recorded stop')
    end
    if completed then break end
  end
  assert(completed and recorded==8 and rebound<.25,'All eight stops must settle without a low-speed rebound under slow polling and actuator response')
  print(('PASS: eight smooth stops, poll %.2fs actuator %.2fs, rebound %.3f b/s, %.1fs'):format(settings[1],settings[2],rebound,p.t))
end

do
  local p=brakingProfiles[1]
  local settle=A.new(names,p)
  local s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  assert(settle:beginBraking(s)); s.t=.2; s.x=20
  local commands=settle:calibrate(s)
  for _,out in pairs(commands) do assert(out.power<1e-9,'A stopped calibration must not accelerate back to its earlier position') end
  local j=settle.job; j.phase='BRAKE'; j.dx,j.dz=1,0
  j.run={at=0,x=0,z=0,speed=10,previousV=.1,previousPressure=.85,distance=0,samples={},coast={}}
  s.t=1.2; s.vx=-2
  settle:calibrate(s)
  assert(j.index==1 and j.phase=='TAIL' and settle.trace.desired[1]>0,'Reverse velocity is motion to damp, not a completed stop')
  local early=A.new(names,p); s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  assert(early:beginBraking(s)); local earlyJob=early.job
  earlyJob.phase='BRAKE'; earlyJob.dx,earlyJob.dz=1,0
  earlyJob.run={at=0,x=0,z=0,speed=10,previousV=10,previousPressure=.85,distance=0,samples={},coast={}}
  s={t=.2,x=1.9,y=227,z=0,vx=9,vz=0,heading=0,yaw=0,pressure=.85}
  early:calibrate(s)
  assert(#earlyJob.run.samples==1,'A valid early reverse-deceleration sample must be recorded before the old 0.8-second gate')
  local sparse=A.new(names,p); s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  assert(sparse:beginBraking(s)); local sparseJob=sparse.job
  sparseJob.index,sparseJob.phase,sparseJob.at,sparseJob.stable=9,'SETTLE',0,0
  sparseJob.runs={}
  for i=1,8 do
    local speed=i<=4 and 8 or 16; local samples={{x=5,y=3,dt=.2}}
    if i>1 then samples[2]={x=5,y=3,dt=.2} end
    sparseJob.runs[i]={speed=speed,distance=i<=4 and 15 or 50,pressure=.85,height=227,samples=samples,
      coast={{x=5,y=.1}}}
  end
  s={t=2.1,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  local _,_,sparseFit=sparse:calibrate(s)
  assert(sparseFit and A.validBraking(sparseFit.braking),'A direction with one measured deceleration sample plus valid coast data must fit')
  local steering=A.new(names,p); steering:reset(0)
  local sample={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85,rawVx=0,rawVz=0}
  steering:navigate(sample,{x=100,z=0})
  sample.t,sample.x,sample.z,sample.vx,sample.rawVx=.2,94,5,8,8
  steering:navigate(sample,{x=100,z=0})
  assert(steering.trace.braking=='REVERSE' and steering.trace.desired[2]<0 and steering.trace.targetCross<0,'Measured reverse braking must still steer back toward the straight route')
  local dense=A.brakeModel(p,1,0,.85,.2)
  local thin=A.brakeModel(p,1,0,.3,.2)
  local slow=A.brakeModel(p,1,0,.85,1)
  assert(A.stoppingDistance(thin,15)>A.stoppingDistance(dense,15),'Lower pressure must increase learned stopping distance when drag is positive')
  assert(A.stoppingDistance(slow,15)>A.stoppingDistance(dense,15),'Slower navigation polling must increase the stopping allowance')
  for _,v in ipairs({1,5,15,40}) do
    local distance=A.stoppingDistance(dense,v)
    assert(math.abs(A.stoppingSpeed(dense,distance)-v)<.001,'Stopping speed must invert the learned physical stopping distance')
  end
  assert(math.abs(A.stoppingDistance({force=4,drag=0,delay=.5},10)-17.5)<1e-8,'Zero-drag stopping must use v*delay + v^2/(2*a)')
  for direction,d in ipairs(p.braking.directions) do
    local axes={{1,0},{-1,0},{0,1},{0,-1}}
    for _,r in ipairs(d.runs) do
      local m=A.brakeModel(p,axes[direction][1],axes[direction][2],r.pressure,d.sampleDt)
      assert(A.stoppingDistance(m,r.speed)>=r.distance,'The saved predictor must cover each actual recorded stop')
    end
  end
  local c=A.new(names,p); local s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  local _,_,partial=c:reverseCommands(s,1,0,p,.3)
  local _,_,full=c:reverseCommands(s,1,0,p)
  assert(math.abs(partial[1]/full[1]-.3)<1e-9,'Reverse thrust must support proportional braking rather than just zero or full power')
  assert(c:beginBraking(s)); local j=c.job
  j.index=2; j.runs[1]={saved=true}; j.phase='BRAKE'; j.interrupted=true; s.t=.2
  assert(c:calibrate(s) and j.phase=='SETTLE' and j.index==2 and j.runs[1].saved,'GPS interruption must restart only the current stopping test')
  local c2=A.new(names,profiles[1]); assert(c2:beginBraking(s)); local j2=c2.job
  j2.index=2; j2.runs[1]={saved=true}; j2.phase='BRAKE'; j2.run={partial=true}
  local bad={}; for k,v in pairs(s) do bad[k]=v end; bad.t=.4; bad.yaw=1
  local paused=c2:calibrate(bad)
  assert(paused and c2.job==j2 and j2.index==2 and j2.runs[1].saved and j2.interrupted,'Invalid yaw during a GPS jump must pause the current stop and keep prior records')
  local recovered={}; for k,v in pairs(s) do recovered[k]=v end; recovered.t=.6
  assert(c2:calibrate(recovered) and c2.job==j2 and j2.phase=='SETTLE' and j2.index==2 and j2.runs[1].saved,'Navigation recovery must resume the interrupted brake test without resetting its count')
  local expected={{1,0},{-1,0},{0,1},{0,-1},{1,0},{-1,0},{0,1},{0,-1}}
  for i,d in ipairs(expected) do
    local ci=A.new(names,profiles[1]); assert(ci:beginBraking(s)); ci.job.index=i; ci.job.stable=0; ci.job.at=0
    local sample={}; for k,v in pairs(s) do sample[k]=v end; sample.t=2.1
    assert(ci:calibrate(sample) and ci.job.phase=='ACCEL','Each brake pass must advance into its requested direction')
    assert(math.abs(ci.job.dx-d[1])<1e-9 and math.abs(ci.job.dz-d[2])<1e-9,'Brake passes must alternate direction and repeat the same order at the second speed')
    assert(math.abs(ci.job.targetSpeed-math.max(2,A.cruiseSpeed(ci.job.candidate)*(i<=4 and .5 or 1)))<1e-9,
      'Brake calibration should use a shorter half-cruise first pass and stop the second pass at cruise speed')
  end
  c:cancel(); assert(c.profile==p and not c.job,'Canceling brake calibration must retain the previous saved braking profile')
  assert(A.new(names,p).profile.braking==p.braking,'Saved braking data must be accepted when reloading the autopilot profile')
  assert(not c:beginBraking(s,{validated=false}),'An incomplete survey must not bypass braking calibration validation')
  print('PASS: pressure, polling latency, stopping-distance inversion, observed-stop coverage, interrupted measurement, cancellation and reload')
end

do
  local now=7
  local s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  local c=A.new(names,profiles[1]); assert(c:beginBraking(s))
  local job=c.job; job.index=2; job.runs[1]={saved=true}; job.phase='BRAKE'; job.run={partial=true}
  local env=setmetatable({Autopilot=A,navController=c,fullCalibration=true,calibrationRunning=true,
    position={x=0,y=227,z=0},previousPosition={x=0,y=227,z=0},previousSample=0,lastGpsAt=0,
    altitude=227,shipHeading=0,velocity={x=0,y=0,z=0},intervalVelocity={x=0,z=0},gpsFilter={},
    mode='BRAKE CAL',burner=6,burnerCommand=6,manualBurner=6,sampleDt=.2,burnerSlewPerSecond=6,status={},
    pid={horizontalX={},horizontalZ={}}},{__index=_G})
  env.os={clock=function() return now end}; env.colors={orange=1,red=2,lime=3}
  env.FreshGPS={error='GPS jump rejected',locate=function() return nil end,accept=function() return true end}
  env.fs={open=function() return nil end}; env.recordFlight=function() end
  env.clearBrakeCheckpoint=function() end
  env.clamp=function(v,a,b) return math.max(a,math.min(b,v)) end
  env.navigationSample=function()
    return {t=now,x=env.position.x,y=env.position.y,z=env.position.z,vx=0,vz=0,rawVx=0,rawVz=0,heading=0,yaw=0,pressure=.85}
  end
  local locating=assert(source:match('(local function locate%(%)%s*.-)\nlocal function updateFlight'))
  local update=assert(source:match('(local function updateFlight%b()%s*.-)\nlocal function savedHoverPoints'))
  local mode=assert(source:match('(local function setFlightMode%b()%s*.-)\nlocal gpsReportAt'))
  local hold=assert(source:match('(local function holdHere%b()%s*.-)\nlocal function engageAutopilot'))
  local locate,tick,cancel=assert(load(mode..'\n'..locating..'\n'..update..'\n'..hold..'\nreturn locate,updateFlight,holdHere','@brake-gps-pause','t',env))()
  assert(not locate()); tick(false)
  assert(not env.position and env.fullCalibration and env.calibrationRunning and c.job==job and job.index==2 and job.runs[1].saved and job.interrupted,'GPS expiry must pause brake calibration without deleting completed stops')
  assert(env.burner==6 and env.manualBurner==6 and env.power==0 and next(env.autopilotCommands)==nil,'GPS pause must cut horizontal commands and retain the burner')
  now=8; assert(not locate() and env.autopilotNotice:match('1/8'),'Waiting for GPS must keep the recorded count visible')
  now=9; env.FreshGPS.locate=function() return 10,227,20 end
  assert(locate()); tick(true)
  assert(c.job==job and job.index==2 and job.runs[1].saved and job.phase=='SETTLE' and job.run==nil and env.mode=='BRAKE CAL','GPS recovery must resume the same brake job and discard only its partial stop')
  env.position=nil; cancel()
  assert(not c.job and not env.fullCalibration and not env.calibrationRunning and env.manualBurner==6 and env.burner==6,'Cancel must work with no GPS and retain lift without starting another calibration')
  now=10; assert(locate()); tick(true)
  assert(not c.job and not env.fullCalibration,'GPS recovery after an explicit cancel must not restart calibration')
  print('PASS: long GPS loss keeps completed brake stops, resumes the same job, cuts horizontal output, and permits cancellation without GPS')
end

do
  local helpers=assert(source:match('(local function clearBrakeCheckpoint%b()%s*.-)\nlocal pendingAutopilot'))
  local files,objects={},{}
  local env=setmetatable({brakeCheckpointPath='airship-brake-checkpoint.cfg',navNames=names,Autopilot=A,fs={},textutils={}}, {__index=_G})
  env.textutils.serialize=function(value) local key=tostring(#objects+1); objects[key]=value; return key end
  env.textutils.unserialize=function(key) return objects[key] end
  env.fs.exists=function(path) return files[path]~=nil end
  env.fs.open=function(path,mode)
    if mode=='w' then return {write=function(value) files[path]=value end,close=function() end} end
    return {readAll=function() return files[path] end,close=function() end}
  end
  env.fs.delete=function(path) files[path]=nil end
  env.fs.move=function(from,to) assert(files[from]~=nil); files[to]=files[from]; files[from]=nil end
  local clear,save,load=assert(load(helpers..'\nreturn clearBrakeCheckpoint,saveBrakeCheckpoint,loadBrakeCheckpoint','@brake-checkpoint','t',env))()
  local run={speed=8,distance=12,pressure=.85,height=227,samples={{x=3,y=.5,dt=.2}},coast={{x=4,y=.1}}}
  local job={candidate=profiles[1],pending=false,index=2,runs={[1]=run},pressure=.85,height=227,heightMin=226,heightMax=228,heading=.4}
  save(job)
  local restored=load(profiles[1],false)
  assert(restored and restored.index==2 and restored.runs[1].distance==12 and #restored.runs[1].samples==1,
    'A settled brake stop with one valid braking sample must survive serialization and restore with its count')
  objects[files['airship-brake-checkpoint.cfg']].version=1
  assert(not load(profiles[1],false),'Old brake checkpoints must be discarded after changing sample collection')
  save(job)
  assert(load(profiles[1],false),'The updated brake checkpoint format must still resume completed stops')
  local changed={}; for k,v in pairs(profiles[1]) do changed[k]=v end
  changed.channels={}; for i,ch in ipairs(profiles[1].channels) do local c={}; for k,v in pairs(ch) do c[k]=v end; c.b={ch.b[1],ch.b[2],ch.b[3]}; changed.channels[i]=c end
  changed.channels[1].b[1]=changed.channels[1].b[1]+.01
  assert(not load(changed,false),'Checkpoint from a different thruster model must not be reused')
  clear(); assert(not env.fs.exists('airship-brake-checkpoint.cfg'),'Explicit cancellation must remove the brake checkpoint')
  print('PASS: brake stop checkpoint saves, restores only for a matching thruster model, and clears on cancel')
end

do
  local old=brakingProfiles[1]
  local candidate={}; for k,v in pairs(old) do candidate[k]=v end
  candidate.validated=nil; candidate.braking={}
  for k,v in pairs(old.braking) do candidate.braking[k]=v end
  candidate.braking.directions={}
  for i,d in ipairs(old.braking.directions) do
    local copy={}; for k,v in pairs(d) do copy[k]=v end
    candidate.braking.directions[i]=copy
  end
  local c=A.new(names,old)
  local s={t=0,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  assert(c:resumeVerification(s,candidate))
  local j=c.job
  j.goals={{x=0,z=0}}; j.legStart={x=-4,z=0}; j.overshoot=1; j.stable=0
  j.verifyWindow={{t=-2,x=0,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}}
  c.navigate=function() return {},true end
  local before=A.stoppingDistance(A.brakeModel(candidate,1,0,.85,.2),15)
  s.t=2
  local commands,message,saved=c:calibrate(s)
  local after=A.stoppingDistance(A.brakeModel(candidate,1,0,.85,.2),15)
  assert(commands and message==nil and saved==candidate and after==before and candidate.maxSpeed==old.maxSpeed,'A settled overshoot must pass without retuning measured braking or reducing cruise speed')
  assert(old.braking.directions[1].delay==candidate.braking.directions[1].delay,'Passing overshoot must preserve the working brake calibration')
  print('PASS: measured overshoot is recorded without retrying or reducing cruise speed')
end

do
  local s={t=0,x=0,y=227,z=0,vx=0,vz=0,heading=0,yaw=0,pressure=.85}
  local old=profiles[1]
  local env=setmetatable({navController=A.new(names,old),status={},mode='HOLDING ALTITUDE',burner=6,burnerCommand=6,position={x=0,y=227,z=0},
    sampleDt=.2,burnerSlewPerSecond=6,Autopilot=A,shipHeading=0}, {__index=_G})
  env.colors={red=1,orange=2,lime=3}
  env.navigationSample=function() return s end
  env.fs={exists=function() return false end}
  env.brakeCheckpointPath='airship-brake-checkpoint.cfg'
  env.loadBrakeCheckpoint=function() return nil end
  env.clearBrakeCheckpoint=function() end
  env.clamp=function(v,a,b) return math.max(a,math.min(b,v)) end
  env.recordFlight=function() end
  env.clearPendingAutopilot=function() error('Brake calibration must not remove a pending 96-test survey') end
  env.applyGainValues=function() end
  env.saveAutopilotProfile=function(p) assert(p==brakingProfiles[1]); env.saved=p end
  local start=assert(source:match('(local function startBrakingCalibration%b()%s*.-)\nlocal function testVector'))
  local update=assert(source:match('(local function updateFlight%b()%s*.-)\nlocal function savedHoverPoints'))
  local begin,tick=assert(load(start..'\n'..update..'\nreturn startBrakingCalibration,updateFlight','@brake-ui','t',env))()
  begin()
  assert(env.fullCalibration and env.navController.job.kind=='BRAKES' and env.manualBurner==6 and env.altitude==227,'Brake button must begin eight-stop measurement with the current burner held even without a prior height selection')
  env.navController.calibrate=function() env.navController.job=nil; return {},nil,brakingProfiles[1] end
  s.t=.2; tick(true)
  assert(env.saved and env.navController.profile==brakingProfiles[1] and env.burner==6 and env.autopilotNotice:match('BRAKE CAL saved'),'Completed brake calibration must save its profile and preserve lift')
  local checkpoint={candidate=old,pending=false,index=3,runs={[1]={saved=true},[2]={saved=true}},pressure=.85,height=227,heightMin=226,heightMax=228,heading=.2}
  env.loadBrakeCheckpoint=function() return checkpoint end
  s.t=.3; begin()
  assert(env.navController.job.index==3 and env.navController.job.runs[1].saved and env.navController.job.heading==.2 and env.navController.job.interrupted,'Brake start must resume completed checkpoint runs and restart only the unfinished stop')
  env.saveBrakeCheckpoint=function(job) env.checkpointSaved=job.index end
  local resumedJob=env.navController.job
  env.navController.calibrate=function() resumedJob.runs[3]={saved=true}; resumedJob.index=4; return {},'recorded' end
  s.t=.4; tick(true)
  assert(env.checkpointSaved==4,'Every newly completed settled stop must be checkpointed immediately')
  env.calibrationRunning,env.fullCalibration=false,false; env.navController.job=nil
  env.loadBrakeCheckpoint=function() return nil end
  env.navController.calibrate=function() env.navController.job=nil; return {},nil,brakingProfiles[1] end
  begin(); env.saveAutopilotProfile=function() error('disk full') end
  s.t=.4; tick(true)
  assert(env.navController.profile==brakingProfiles[1] and env.burner==6 and env.autopilotNotice:match('could not save'),'A failed brake-profile write must keep the previous working profile and burner: '..tostring(env.autopilotNotice))
  local pending={}; for k,v in pairs(brakingProfiles[3]) do pending[k]=v end; pending.validated=nil
  env.pendingAutopilot=pending; env.navController=A.new(names)
  env.savePendingAutopilot=function(p) assert(p==pending and A.validCandidate(p,names) and not p.validated); env.savedPending=p end
  begin()
  assert(env.navController.job.pending and not env.navController.job.candidate.validated,'Brake button must prefer the saved survey when the five tests have not passed')
  env.navController.calibrate=function() env.navController.job=nil; return {},nil,pending end
  s.t=.6; tick(true)
  assert(env.savedPending==pending and env.pendingAutopilot==pending and env.navController.profile==nil and env.autopilotNotice:match('5 flight tests'),'Pending brakes must save to the survey checkpoint and require flight validation')
  begin(); env.savePendingAutopilot=function() error('disk full') end
  s.t=.8; tick(true)
  assert(env.pendingAutopilot==pending and not env.navController.profile and env.burner==6 and env.autopilotNotice:match('could not save'),'Failed pending-brake saves must retain the checkpoint and keep normal AUTO unavailable')
  local job={phase='VERIFY',candidate=pending,leg=1,retry=0,persistRevision=0,legStart={x=0,z=0},goals={{x=20,z=0},{x=-20,z=0}}}
  env.navController={job=job,trace={validation={remaining=-1.2,cross=-9,overshoot=1.9}}}
  env.navController.calibrate=function() job.leg=2; return {},'next test' end
  env.fullCalibration,env.pendingRetry=true,0
  env.recordFlight=function(_,goal,_,_,stage) env.recordedGoal,env.recordedStage=goal,stage end
  s.t=1; tick(true)
  assert(env.recordedGoal==job.goals[1] and env.recordedStage=='CAL_1_TRY_1','A boundary flight sample must retain its actual test destination and stage')
  local readout=assert(source:match('(local function calibrationReadout%b()%s*.-)\nlocal function clamp'))
  local details=assert(load(readout..'\nreturn calibrationReadout','@calibration-readout','t',env))()
  assert(details()=='LEFT -1.2m  SIDE 9.0m  OVER 1.90m','Test display must distinguish signed distance, sideways error and confirmed overshoot')
  env.fullCalibration=false; assert(details()==nil,'Finished calibration must not leave stale scoring in the flight display')
  print('PASS: brake button, held burner, working/pending saves, validation gate, checkpoint preservation and disk-full rollback')
end

for i,profile in ipairs(brakingProfiles) do
  for _,pressure in ipairs({.85,.3}) do
    local p=plant(table.unpack(scenarios[i])); p.pressure=pressure; p.heading=1.1
    local controller=A.new(names,profile); controller:reset(p.heading)
    local goal={x=1000,z=0}
    local peak,over,atFifteen,atThree=0,0,nil,nil
    local arrived=false
    for _=1,5000 do
      local commands,done=controller:navigate(p:sample(),goal)
      p:advance(commands)
      peak=math.max(peak,math.sqrt(p.vx^2+p.vz^2)); over=math.max(over,p.x-goal.x)
      if not atFifteen and p.x>=985 then atFifteen=p.t end
      if not atThree and p.x>=997 then atThree=p.t end
      if done then arrived=true; break end
    end
    print(('BRAKING SIM mass %.1f pressure %.2f: arrived %s, peak %.2f, overshoot %.2f, final 15-to-3 %.1fs'):format(scenarios[i][1],pressure,tostring(arrived),peak,over,atThree and atThree-atFifteen or -1))
    assert(arrived,'Measured braking must reach the destination')
    assert(over<.5,'Measured braking must stay within half a block of overshoot')
    assert(atFifteen and atThree and atThree-atFifteen<15,'Measured braking must not crawl from fifteen to three blocks')
    controller:setHeadingTarget(math.floor(p.heading/(math.pi/2)+.5)*(math.pi/2))
    for _=1,300 do p:advance(controller:navigate(p:sample(),goal)) end
    assert(math.sqrt((p.x-goal.x)^2+p.z^2)<.75,'Measured braking must continue holding position through final cardinal alignment')
  end
end

for i,goal in ipairs({{x=1000,z=0},{x=-1000,z=0},{x=0,z=1000},{x=0,z=-1000}}) do
  local p=plant(1.4,.65,.05); p.pressure=.3; p.heading=1.1
  local controller=A.new(names,brakingProfiles[3]); controller:reset(p.heading)
  local over,arrived=0,false
  for _=1,5000 do
    local commands,done=controller:navigate(p:sample(),goal); p:advance(commands)
    over=math.max(over,(p.x*goal.x+p.z*goal.z)/1000-1000)
    if done then arrived=true; break end
  end
  assert(arrived and over<.5,'Measured brakes must handle each world direction with noisier, slower GPS')
  print(('PASS: measured brakes direction %d, 0.65s noisy GPS, overshoot %.3f, time %.1fs'):format(i,over,p.t))
end

for i,profile in ipairs(profiles) do
  for _,pressure in ipairs({.85,.3}) do
    for _,length in ipairs({100,1000}) do
      local p=plant(table.unpack(scenarios[i])); p.pressure=pressure; p.heading=2.9
      local controller=A.new(names,profile); controller:reset(p.heading)
      local sign=pressure==.85 and 1 or -1
      local goal=length==100 and {x=sign*length,z=0} or {x=0,z=sign*length}
      local peak,overshoot,atSix,atOne=0,0,nil,nil
      local arrived=false
      for _=1,10000 do
        local commands,done=controller:navigate(p:sample(),goal)
        assert(commands)
        for _,out in pairs(commands) do assert(out.power<=profile.maxThrottle+.000001,"Mixer exceeded tested power") end
        p:advance(commands)
        peak=math.max(peak,math.sqrt(p.vx^2+p.vz^2))
        overshoot=math.max(overshoot,(p.x*goal.x+p.z*goal.z)/length-length)
        local actualDistance=math.sqrt((p.x-goal.x)^2+(p.z-goal.z)^2)
        if not atSix and actualDistance<=6 then atSix=p.t end
        if not atOne and actualDistance<=1 then atOne=p.t end
        if done then arrived=true; break end
      end
      assert(arrived,"Fast cruise never arrived")
      assert(p.t<(length==100 and 140 or 600),"Fast cruise regressed to crawl speed")
      assert(overshoot<.5,"Fast cruise failed the braking tolerance")
      if i<=3 then assert(atSix and atOne and atOne-atSix<20,"Final approach must not crawl between six and one blocks remaining") end
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
  local function clone(value)
    if type(value)~="table" then return value end
    local result={}; for k,v in pairs(value) do result[k]=clone(v) end; return result
  end
  local legacy=clone(profiles[1]); legacy.cruiseVersion=2; legacy.maxSpeed=13
  local env=setmetatable({Autopilot=A,navNames=names,navController=A.new(names)}, {__index=_G})
  env.fs={exists=function() return true end,open=function() return {readAll=function() return "saved" end,close=function() end} end}
  env.textutils={serialize=function() return "saved" end,unserialize=function() return clone(legacy) end}
  local loading=assert(source:match('(if fs.exists%("airship%-autopilot.cfg"%) then.-)\nlocal sweepConfirmAt'))
  assert(load(loading,"@legacy-profile-load","t",env))()
  assert(env.navController.profile.validated and env.pendingAutopilot.validated==nil,"Migration must retain working profile while preparing auto tests")
  assert(#env.pendingAutopilot.channels==#names*4 and env.pendingAutopilot~=env.navController.profile,"Migration must reuse the survey without modifying the working profile")
  legacy.gpsVersion=nil; env.pendingAutopilot=nil; env.navController=A.new(names)
  assert(load(loading,"@unsafe-gps-profile-load","t",env))()
  assert(not env.navController.profile and not env.pendingAutopilot and env.autopilotNotice:match("GPS"),"Old unchecked-GPS model must require a fresh survey")
  print("PASS: completed legacy profile reuses its full thruster survey for new cruise validation")
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
  local commands,message=controller:calibrate(invalid)
  assert(commands and controller.job and controller.profile==old and message:match("Waiting for fresh"),"Invalid samples must pause calibration without replacing its profile")
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
  env.savePendingAutopilot=function(p) assert(A.validCandidate(p,names)) end
  env.clearPendingAutopilot=function() end
  env.recordFlight=function() end
  env.pendingAutopilot,env.pendingRetry=nil,nil
  env.applyGainValues=function(g) env.applied=g end
  local start=assert(source:match("(local function startFullCalibration%(%)%s*.-)\nlocal function testVector"))
  local update=assert(source:match("(local function updateFlight%b()%s*.-)\nlocal function savedHoverPoints"))
  local startFn,updateFn=assert(load(start.."\n"..update.."\nreturn startFullCalibration,updateFlight","@integration","t",env))()
  startFn()
  assert(env.fullCalibration and env.calibrationRunning and env.manualBurner==6 and not env.navController.job)
  sample.t=.2
  updateFn(true)
  assert(env.navController.job,"Calibration should start from the next fresh control-loop sample")
  env.navController.calibrate=function() return {},nil,profile end
  sample.t=.4
  updateFn(true)
  assert(env.burner==6 and env.burnerCommand==6,"Calibration changed burner output")
  assert(env.saved==profile and env.navController.profile==profile and env.applied==profile.gains)
  assert(not env.fullCalibration and not env.calibrationRunning and env.burner==6,"Completion must leave height control alone")
  startFn()
  env.saveAutopilotProfile=function() error("disk full") end
  local previous=env.navController.profile
  local failedCandidate={gains={.01,0,.3},validated=true}
  env.navController.calibrate=function() return {},nil,failedCandidate end
  sample.t=.6
  updateFn(true)
  sample.t=.8
  updateFn(true)
  assert(env.navController.profile==previous and env.autopilotNotice:match("could not save"),"Save failure replaced the working profile")
  env.mode,env.target="AUTOPILOT",{x=10,z=0}; sample.tilt=nil
  updateFn(true)
  assert(env.mode=="AUTOPILOT","Gyro-handled tilt must not stop navigation")
  local destination=env.target
  env.navController.navigate=function() return {},true end
  local headingCode=assert(source:match('(local function nearestCardinalHeading%b()%s*.-\nend)'))
  env.nearestCardinalHeading=assert(load(headingCode..'\nreturn nearestCardinalHeading',"@nearest-heading"))()
  for _,case in ipairs({{1,0},{44,0},{46,90},{179,180},{269,270},{359,0},{-1,0},{-91,270},{721,0}}) do
    assert(math.abs(env.nearestCardinalHeading(math.rad(case[1]))-math.rad(case[2]))<1e-9,"Final heading must round to the nearest quarter turn across 0/360")
  end
  env.arrivalHeading=math.pi; sample.heading=math.rad(83)
  updateFn(true)
  assert(env.mode=="FINAL TURN" and env.target==destination,"Arrival turn must preserve the requested destination")
  assert(math.abs(env.navController.headingTarget-math.pi/2)<1e-9,"Final turn must choose 90 degrees from actual heading 83, rather than a saved route heading")
  sample.yaw=nil
  updateFn(true)
  assert(env.mode=="NAV LOST - RE-ENGAGE" and next(env.autopilotCommands)==nil and env.autopilotNotice:match("yaw"),"Missing yaw rate must stop navigation visibly")
  env.position=nil; env.burner,env.burnerCommand=6,6
  updateFn(false)
  assert(env.burner==6 and env.burnerCommand==6 and env.power==0,"Control pass after GPS loss must preserve holding burner output")
  print("PASS: controller button, fixed burner output, calibration completion and failed-save rollback")
end

do
  local lines,closed={},0
  local created
  local env=setmetatable({Autopilot=A,navNames=names,navController={trace={},applied={}},AIRSHIP_BUILD="test"},{__index=_G})
  env.textutils={serialize=function(profile) assert(profile==profiles[1]); return "fixture profile" end}
  env.os={date=function(format) assert(format=="%Y-%m-%d_%H-%M-%S" or format=="%Y-%m-%d %H:%M:%S"); return format:find("_") and "2026-10-23_12-34-56" or "2026-10-23 12:34:56" end}
  env.fs={open=function(path,mode)
    assert(path=="flight logs/flight-2026-10-23_12-34-56.csv")
    if mode=="w" then lines={} end
    return {writeLine=function(line) lines[#lines+1]=line end,close=function() closed=closed+1 end}
  end,makeDir=function(path) assert(path=="flight logs"); created=path end,exists=function() return false end}
  local recording=assert(source:match('(local flightRecording.-)\nlocal function saveAutopilotProfile'))
  local record,getRecording=assert(load(recording..'\nreturn recordFlight,function() return flightRecording end',"@flight-recorder","t",env))()
  local s={t=0,x=123456,z=-654321,vx=0,vz=0,heading=0,yaw=0,pressure=.8}
  local goal={x=s.x+20,z=s.z-10}
  for i=0,1200 do s.t=i*.5; record(s,goal,profiles[1],{},"AUTOPILOT") end
  assert(created=="flight logs" and closed==1200 and #lines==1203,"Recorder must create a per-flight directory and retain ten minutes at 2 Hz")
  local report=table.concat(lines,'\n')
  assert(not report:find('123456',1,true) and not report:find('654321',1,true),"Flight report exposed absolute GPS coordinates")
  assert(lines[1]:find("started 2026%-10%-23 12:34:56") and lines[4]:find('0,AUTOPILOT,0,0,20,-10'),"Flight report must include a timestamp and relative target/movement")
  assert(recording:find("maxFlightSamples=1200",1,true),"Flight report must be bounded at ten minutes")
  record(s,goal,profiles[1],{},"AUTOPILOT")
  assert(closed==1200 and getRecording().truncated,"Longer flights must stop recording at the stated cap without growing disk forever")
  print("PASS: per-flight timestamped report retains ten minutes at 2 Hz without absolute GPS coordinates")
end
