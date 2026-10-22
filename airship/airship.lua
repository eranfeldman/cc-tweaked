-- Basalt 2.5 flight panel for the east-facing eight-thruster ship.
local AIRSHIP_BUILD = "1.4.1"
print("AIRSHIP v" .. AIRSHIP_BUILD .. ": loading UI")
local basalt = require("basalt")

local yieldSequence = 0
local function yieldComputer()
  -- A queued event yields without the tick delay of sleep(0).
  yieldSequence = yieldSequence + 1
  local event = "airship_yield_" .. yieldSequence
  os.queueEvent(event)
  os.pullEvent(event)
end
yieldComputer()
-- BEGIN AUTOPILOT CORE
local Autopilot = (function()
  local A = {}
  A.__index = A
  local directions = {{.35,0},{-.35,0},{0,.35},{0,-.35}}
  local throttleLimit, testThrottle = 1, .2
  local function finite(v) return type(v)=="number" and v==v and math.abs(v)<math.huge end
  local function clamp(v,a,b) return math.max(a,math.min(b,v)) end
  local function wrap(v) return (v+math.pi)%(2*math.pi)-math.pi end
  local function body(x,z,h) local c,s=math.cos(h),math.sin(h); return c*x-s*z,s*x+c*z end
  local function copy(s)
    return {t=s.t,x=s.x,z=s.z,vx=s.vx,vz=s.vz,heading=s.heading,yaw=s.yaw,pressure=s.pressure,tilt=s.tilt}
  end
  local function observation(a,b)
    local dt=b.t-a.t
    local h=a.heading+wrap(b.heading-a.heading)/2
    local ax,az=body((b.vx-a.vx)/dt,(b.vz-a.vz)/dt,h)
    local vx,vz=body((a.vx+b.vx)/2,(a.vz+b.vz)/2,h)
    return {ax,az,(b.yaw-a.yaw)/dt},{vx,vz,(a.yaw+b.yaw)/2}
  end
  local function window(samples)
    local n,t,tt,h=0,0,0,0
    local v,tv={0,0,0},{0,0,0}
    local x,z,tx,tz=0,0,0,0
    local first=samples[1]
    for _,s in ipairs(samples) do
      local elapsed=s.t-first.t
      n=n+1; t=t+elapsed; tt=tt+elapsed^2; h=h+wrap(s.heading-first.heading)
      x=x+s.x; z=z+s.z; tx=tx+elapsed*s.x; tz=tz+elapsed*s.z
      for i,value in ipairs({s.vx,s.vz,s.yaw}) do v[i]=v[i]+value; tv[i]=tv[i]+elapsed*value end
    end
    local a={}
    for i=1,3 do a[i]=(tv[i]-t*v[i]/n)/math.max(.0001,tt-t*t/n); v[i]=v[i]/n end
    local heading=first.heading+h/n
    a[1],a[2]=body(a[1],a[2],heading); v[1],v[2]=body(v[1],v[2],heading)
    local variance=math.max(.0001,tt-t*t/n)
    return {a=a,v=v,x=x/n,z=z/n,vx=(tx-t*x/n)/variance,vz=(tz-t*z/n)/variance,yaw=v[3],span=samples[n].t-first.t}
  end
  function A.validSample(s)
    if type(s)~="table" then return false end
    for _,key in ipairs({"t","x","z","vx","vz","heading","yaw","pressure"}) do if not finite(s[key]) then return false end end
    return s.pressure>=0 and (s.tilt==nil or finite(s.tilt) and s.tilt>=0 and s.tilt<=math.pi)
  end
  function A.headingRate(rates,gravity)
    if type(rates)~="table" or not finite(rates[2]) then return nil end
    if type(gravity)~="table" or not finite(rates[1]) then return rates[2] end
    for i=1,3 do if not finite(gravity[i]) then return rates[2] end end
    local length=math.sqrt(gravity[1]^2+gravity[2]^2+gravity[3]^2)
    if length<.001 then return rates[2] end
    local ux,uy,uz=-gravity[1]/length,-gravity[2]/length,-gravity[3]/length
    -- Heading is the projected local +Z direction; body Y rate alone mixes tilt into yaw.
    if 1-uz*uz<.1 then return nil end
    return (rates[1]*ux+rates[2]*uy)/(1-uz*uz),math.acos(clamp(uy,-1,1))
  end
  function A.validProfile(p,names)
    if type(p)~="table" or p.version~=1 or p.validated~=true or type(p.names)~="table" or #p.names~=#names or type(p.channels)~="table" or #p.channels~=#names*4 then return false end
    for i,name in ipairs(names) do if p.names[i]~=name then return false end end
    for _,key in ipairs({"pressure","yawWeight","maxAccel","maxSpeed","delay"}) do if not finite(p[key]) or p[key]<=0 then return false end end
    if p.pressure>2 or p.yawWeight>30 or p.maxAccel>4 or p.maxSpeed>20 or p.delay>10 or type(p.drag)~="table" or type(p.gains)~="table" then return false end
    for i=1,3 do
      if not finite(p.drag[i]) or p.drag[i]<0 or p.drag[i]>3 or not finite(p.gains[i]) or p.gains[i]<0 then return false end
    end
    if p.gains[1]<.0001 or p.gains[1]>1 or p.gains[2]>.05 or p.gains[3]>4 then return false end
    if p.tiltReference~=nil and (not finite(p.tiltReference) or p.tiltReference<0 or p.tiltReference>math.pi) then return false end
    if p.maxThrottle~=nil and (not finite(p.maxThrottle) or p.maxThrottle<.1 or p.maxThrottle>1) then return false end
    local gram={{0,0,0},{0,0,0},{0,0,0}}
    for i,ch in ipairs(p.channels) do
      if type(ch)~="table" or ch.name~=names[math.floor((i-1)/4)+1] or type(ch.b)~="table" then return false end
      local d=directions[(i-1)%4+1]
      if ch.x~=d[1] or ch.y~=d[2] then return false end
      for j=1,3 do if not finite(ch.b[j]) or math.abs(ch.b[j])>10000 then return false end end
      for r=1,3 do for c=1,3 do gram[r][c]=gram[r][c]+ch.b[r]*ch.b[c] end end
    end
    local a,b,c=gram[1],gram[2],gram[3]
    local determinant=a[1]*(b[2]*c[3]-b[3]*c[2])-a[2]*(b[1]*c[3]-b[3]*c[1])+a[3]*(b[1]*c[2]-b[2]*c[1])
    return a[1]>1e-7 and b[2]>1e-7 and c[3]>1e-7 and determinant/(a[1]*b[2]*c[3])>.0001
  end
  function A.new(names,profile)
    local sorted={}; for _,name in ipairs(names) do sorted[#sorted+1]=name end; table.sort(sorted)
    return setmetatable({names=sorted,profile=A.validProfile(profile,sorted) and profile or nil,integral={0,0}},A)
  end
  function A:reset(heading)
    self.integral={0,0}; self.headingTarget=heading; self.previous=nil; self.mix=nil; self.goalKey=nil
    self.trim={0,0,0}; self.motion=nil; self.applied=nil; self.track={}
  end
  function A:setHeadingTarget(heading) self.headingTarget=heading end
  function A:cancel()
    self.job=nil; self:reset(nil)
  end
  function A:allocate(p,desired,throttleCap)
    local u,group={},{}
    local residual={desired[1],desired[2],desired[3]*p.yawWeight}
    local columns={}
    local scale=0
    for i,ch in ipairs(p.channels) do
      local b={ch.b[1],ch.b[2],ch.b[3]*p.yawWeight}
      columns[i]=b; u[i]=0; group[ch.name]=0
      scale=scale+b[1]^2+b[2]^2+b[3]^2
    end
    local ridge=math.max(1e-8,scale/#columns*.005)
    for _=1,16 do
      for i,ch in ipairs(p.channels) do
        local b=columns[i]
        local norm=b[1]^2+b[2]^2+b[3]^2+ridge
        local gradient=b[1]*residual[1]+b[2]*residual[2]+b[3]*residual[3]-ridge*u[i]
        local value=clamp(u[i]+gradient/norm,0,math.min(p.maxThrottle or .35,throttleCap or p.maxThrottle or .35)-group[ch.name]+u[i])
        local change=value-u[i]
        u[i]=value; group[ch.name]=group[ch.name]+change
        for j=1,3 do residual[j]=residual[j]-b[j]*change end
      end
    end
    local commands={}
    for _,name in ipairs(self.names) do commands[name]={x=0,y=0,power=0} end
    for i,ch in ipairs(p.channels) do
      local out=commands[ch.name]
      out.x=out.x+ch.x*u[i]; out.y=out.y+ch.y*u[i]; out.power=out.power+u[i]
    end
    for _,out in pairs(commands) do if out.power>1e-6 then out.x=out.x/out.power; out.y=out.y/out.power else out.x,out.y,out.power=0,0,0 end end
    return commands,residual
  end
  function A:navigate(s,goal,p)
    p=p or self.profile
    if not p or not A.validSample(s) or p.tiltReference~=nil and s.tilt==nil then return nil,false end
    local key=goal.x..":"..goal.z
    if self.goalKey~=key then self.integral={0,0}; self.goalKey=key end
    self.headingTarget=self.headingTarget or s.heading
    local dt=self.previous and clamp(s.t-self.previous.t,.01,2) or .2
    local ratio=clamp(s.pressure/math.max(.001,p.pressure),0,2)
    -- Thin air supplies less natural damping, so ease delayed GPS feedback without capping cruise speed.
    local bandwidth=math.sqrt(clamp(ratio,.16,1))
    local proportional,derivative=p.gains[1]*bandwidth^2,p.gains[3]*bandwidth
    -- Learn persistent drift/torque instead of asking a tiny position integral to balance the ship.
    self.trim=self.trim or {0,0,0}
    if self.previous and self.applied then
      local a,v=observation(self.previous,s)
      local alpha=1-math.exp(-dt/8)
      for i=1,3 do
        local limit=i==3 and .05 or p.maxAccel*.6
        self.trim[i]=clamp(self.trim[i]+alpha*(a[i]+p.drag[i]*ratio*v[i]-self.applied[i]-self.trim[i]),-limit,limit)
      end
    end
    local f=self.motion or copy(s)
    local alpha=1-math.exp(-dt/.6)
    for _,k in ipairs({"x","z","vx","vz","yaw"}) do f[k]=f[k]+alpha*(s[k]-f[k]) end
    self.track=self.track or {}
    self.track[#self.track+1]=copy(s)
    while #self.track>2 and s.t-self.track[1].t>2.4 do table.remove(self.track,1) end
    if #self.track>2 then
      local trend=window(self.track)
      f.vx,f.vz=trend.vx,trend.vz
    end
    self.motion=f
    local dx,dz=goal.x-f.x,goal.z-f.z
    local distance,speed=math.sqrt(dx*dx+dz*dz),math.sqrt(f.vx^2+f.vz^2)
    local ex,ez=body(dx,dz,s.heading)
    local vx,vz=body(f.vx,f.vz,s.heading)
    local balance=1
    if s.tilt and p.tiltReference then balance=clamp(1-(s.tilt-p.tiltReference-.1)/.3,.15,1) end
    local maxAccel=p.maxAccel*balance
    local braking=math.sqrt(2*maxAccel*math.max(0,distance-.2-speed*p.delay))
    local speedLimit=math.min(p.maxSpeed*math.sqrt(balance),braking)
    local drive=proportional*distance
    if derivative>0 then drive=math.min(drive,derivative*speedLimit) end
    local acceleration={}
    for i,values in ipairs({{ex,vx},{ez,vz}}) do
      if distance<2 and speed<.25 then self.integral[i]=clamp(self.integral[i]+values[1]*dt,-2,2) else self.integral[i]=0 end
      local along=distance>.001 and values[1]/distance or 0
      local a=drive*along-derivative*values[2]+p.gains[2]*bandwidth^2*self.integral[i]
      acceleration[i]=clamp(a,-maxAccel,maxAccel)+p.drag[i]*ratio*values[2]
    end
    local yaw=clamp(.22*wrap(self.headingTarget-s.heading)-1.8*s.yaw,-.12,.12)+p.drag[3]*ratio*s.yaw
    local desired={acceleration[1]-self.trim[1],acceleration[2]-self.trim[2],yaw-self.trim[3]}
    local commands,residual=self:allocate(p,desired,(p.maxThrottle or .35)*balance)
    self.applied={desired[1]-(residual and residual[1] or 0),desired[2]-(residual and residual[2] or 0),desired[3]-(residual and residual[3]/p.yawWeight or 0)}
    self.previous=copy(s)
    return commands,distance<.35 and speed<.08 and math.abs(s.yaw)<.03 and math.abs(wrap(self.headingTarget-s.heading))<.03
  end
  function A:begin(s)
    if not A.validSample(s) or s.pressure<=.001 then return false,"Fresh GPS, heading, yaw rate and air pressure are required." end
    self:reset(s.heading)
    self.job={phase="BASELINE",at=s.t,previous=copy(s),origin={x=s.x,z=s.z},heading=s.heading,tiltReference=s.tilt,
      index=1,channels={},measurements={},window={copy(s)},coasts={},pressure=0,samples=0,dt=0,retry=0}
    return true
  end
  local function drag(job)
    local result={}
    for i=1,3 do
      local v,a,vv,av,n=0,0,0,0,#job.coasts
      for _,c in ipairs(job.coasts) do v=v+c.v[i]; a=a+c.a[i]; vv=vv+c.v[i]^2; av=av+c.v[i]*c.a[i] end
      local variance=vv-v*v/math.max(1,n)
      result[i]=variance>.001 and clamp(-(av-a*v/n)/variance,0,3) or 0
    end
    return result
  end
  function A:fit(job)
    local p={version=1,names=self.names,channels=job.channels,pressure=job.pressure/job.samples,drag=drag(job),delay=job.dt/job.samples+1.8,tiltReference=job.tiltReference,maxThrottle=job.tiltReference and throttleLimit or .35}
    for index,m in ipairs(job.measurements) do
      local response={}
      for i=1,3 do
        local before=m.before.a[i]+p.drag[i]*m.before.v[i]
        local after=m.after.a[i]+p.drag[i]*m.after.v[i]
        response[i]=(m.pulse.a[i]+p.drag[i]*m.pulse.v[i]-(before+after)/2)/testThrottle
      end
      if response[1]^2+response[2]^2+response[3]^2<1e-6 then return nil,"Thruster response too small; check fuel and obstruction, then retry." end
      local d=directions[(index-1)%4+1]
      p.channels[index]={name=self.names[math.floor((index-1)/4)+1],x=d[1],y=d[2],b=response}
    end
    local force,yaw=0,0
    local extent={0,0,0,0}
    for _,name in ipairs(self.names) do
      local best={0,0,0,0}
      for _,ch in ipairs(p.channels) do if ch.name==name then
        force=force+ch.b[1]^2+ch.b[2]^2; yaw=yaw+ch.b[3]^2
        best[1]=math.max(best[1],ch.b[1]); best[2]=math.max(best[2],-ch.b[1])
        best[3]=math.max(best[3],ch.b[2]); best[4]=math.max(best[4],-ch.b[2])
      end end
      for i=1,4 do extent[i]=extent[i]+best[i]*p.maxThrottle end
    end
    if yaw<1e-7 or force<1e-5 then return nil,"No usable translation or yaw response; check fuel, nozzles and gimbal." end
    p.yawWeight=clamp(math.sqrt(force/yaw)*.7,1,30)
    p.maxAccel=math.min(3,math.min(table.unpack(extent))*.55)
    if p.maxAccel<.015 then return nil,"Insufficient measured thrust in all four directions." end
    p.maxSpeed=math.min(12,math.sqrt(48*p.maxAccel),p.maxAccel*.75/math.max(.02,p.drag[1],p.drag[2]))
    local frequency=math.min(.3,1/(2*p.delay),math.sqrt(p.maxAccel/3))
    p.gains={frequency^2,frequency^2*.02,2.2*frequency}
    for _,goal in ipairs({{p.maxAccel,0,0},{-p.maxAccel,0,0},{0,p.maxAccel,0},{0,-p.maxAccel,0},{0,0,.03},{0,0,-.03}}) do
      local _,residual=self:allocate(p,goal)
      local error=residual[1]^2+residual[2]^2+residual[3]^2
      local magnitude=goal[1]^2+goal[2]^2+(goal[3]*p.yawWeight)^2
      if error>magnitude*.2 then return nil,"Thruster layout cannot independently steer and counter yaw." end
    end
    return p
  end
  local function travelGoals(job)
    local p=job.candidate
    local span=math.max(24,2*p.maxSpeed*p.gains[3]/p.gains[1],p.maxSpeed^2/p.maxAccel+p.maxSpeed*p.delay+4)
    return {{x=job.origin.x+span,z=job.origin.z},{x=job.origin.x-span,z=job.origin.z},
      {x=job.origin.x,z=job.origin.z+span},{x=job.origin.x,z=job.origin.z-span},{x=job.origin.x,z=job.origin.z}}
  end
  function A:calibrate(s)
    local j=self.job
    if not j then return nil,"Calibration is not running." end
    if not A.validSample(s) or s.t<=j.previous.t or s.t-j.previous.t>2 then self.job=nil; return nil,"Calibration stopped: stale or invalid navigation readings." end
    if j.tiltReference~=nil and s.tilt==nil then self.job=nil; return nil,"Calibration stopped: gimbal gravity reading lost." end
    local dt=s.t-j.previous.t
    j.dt=j.dt+dt; j.samples=j.samples+1; j.pressure=j.pressure+s.pressure
    j.previous=copy(s)
    if j.phase=="BASELINE" or j.phase=="PULSE" or (j.phase=="COAST" and s.t-j.at>=1) then j.window[#j.window+1]=copy(s) end
    if j.phase=="BASELINE" and s.t-j.at>=4 then
      j.before=window(j.window); j.coasts[#j.coasts+1]=j.before; j.phase="WARMUP"; j.at=s.t
    end
    if j.phase=="WARMUP" and s.t-j.at>=1 then j.phase="PULSE"; j.at=s.t; j.window={copy(s)} end
    if j.phase=="PULSE" and s.t-j.at>=6 then
      j.pulse=window(j.window); j.window={}; j.phase="COAST"; j.at=s.t
    end
    if j.phase=="COAST" and s.t-j.at>=5 then
      local after=window(j.window)
      j.coasts[#j.coasts+1]=after
      j.measurements[j.index]={pulse=j.pulse,before=j.before,after=after}; j.before=after
      j.index=j.index+1
      if j.index>#self.names*4 then
        local p,err=self:fit(j)
        if not p then self.job=nil; return nil,err end
        j.candidate=p; j.phase="VERIFY"; j.leg=1; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.legStart={x=s.x,z=s.z}; j.overshoot=0
        j.goals=travelGoals(j)
        self:reset(j.heading)
      else j.phase="WARMUP"; j.at=s.t end
    end
    if j.phase=="RECOVER" then
      if s.tilt<=j.candidate.tiltReference+.1 then
        j.phase="VERIFY"; j.at=s.t; j.legStart={x=s.x,z=s.z}
      elseif s.t-j.at>30 then self.job=nil; return nil,"Rocking did not settle with thrusters off; previous calibration kept."
      else return {},"Letting rocking settle before testing reduced thrust." end
    end
    if j.phase=="VERIFY" then
      if s.tilt and j.candidate.tiltReference and s.tilt>j.candidate.tiltReference+.35 then
        if j.retry>=4 then self.job=nil; return nil,"Too much rocking at tested power; previous calibration kept." end
        j.retry=j.retry+1
        j.candidate.maxThrottle=math.max(.1,j.candidate.maxThrottle*.65)
        j.candidate.maxAccel=j.candidate.maxAccel*.65
        j.candidate.maxSpeed=j.candidate.maxSpeed*.8
        j.goals=travelGoals(j)
        j.phase="RECOVER"; j.leg=1; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.legStart={x=s.x,z=s.z}; j.overshoot=0
        self:reset(j.heading)
        return {},"Reducing thrust for rocking; repeating fast travel tests."
      end
      local goal=j.goals[j.leg]
      local gx,gz=goal.x-j.legStart.x,goal.z-j.legStart.z
      local length=math.sqrt(gx*gx+gz*gz)
      if length>.01 then j.overshoot=math.max(j.overshoot,((s.x-goal.x)*gx+(s.z-goal.z)*gz)/length) end
      local commands=self:navigate(s,goal,j.candidate)
      -- Measure net drift over several rocking cycles; bound each position so real excursions cannot pass.
      j.verifyWindow=j.verifyWindow or {}
      j.verifyWindow[#j.verifyWindow+1]=copy(s)
      while #j.verifyWindow>2 and s.t-j.verifyWindow[1].t>4 do table.remove(j.verifyWindow,1) end
      local f=window(j.verifyWindow)
      local dx,dz=goal.x-f.x,goal.z-f.z
      local settled=math.sqrt(dx*dx+dz*dz)<.35 and math.sqrt(f.vx*f.vx+f.vz*f.vz)<.08 and math.abs(f.yaw)<.03
      local rawDistance=math.sqrt((goal.x-s.x)^2+(goal.z-s.z)^2)
      local bounded=rawDistance<.7 and math.abs(s.yaw)<.1
      j.stable=settled and bounded and (j.stable or s.t) or nil
      if j.stable and s.t-j.stable>=1.5 and f.span>=3 then
        j.leg=j.leg+1; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.legStart={x=s.x,z=s.z}
        if j.leg>#j.goals then
          if j.overshoot>.5 then
            if j.retry>=4 then self.job=nil; return nil,"Braking validation still overshoots; previous calibration kept." end
            j.retry=j.retry+1; j.candidate.gains[1]=j.candidate.gains[1]*.7
            j.candidate.gains[2]=j.candidate.gains[1]*.02
            j.candidate.gains[3]=math.min(4,j.candidate.gains[3]*1.2)
            j.candidate.maxSpeed=j.candidate.maxSpeed*.8
            j.leg=1; j.overshoot=0; self:reset(j.heading)
            return {},"Retuning braking; repeating all four direction tests."
          end
          j.candidate.validated=true
          j.candidate.verification={overshoot=j.overshoot,passes=j.retry+1}
          self.job=nil
          return {},nil,j.candidate
        end
      elseif s.t-j.at>math.max(60,6/math.sqrt(j.candidate.gains[1]))+length/math.max(.1,j.candidate.maxSpeed)*2 then
        if j.retry>=4 then
          self.job=nil
          return nil,("Auto test %d stalled: error %.2fm, drift %.2fm/s, yaw %.3f. Previous calibration kept."):format(j.leg,math.sqrt(dx*dx+dz*dz),math.sqrt(f.vx^2+f.vz^2),math.abs(f.yaw))
        end
        j.retry=j.retry+1
        local speed=math.sqrt(f.vx*f.vx+f.vz*f.vz)
        j.candidate.gains[1]=clamp(j.candidate.gains[1]*(speed>.12 and .7 or 1.25),.0001,.25)
        j.candidate.gains[2]=j.candidate.gains[1]*.02
        j.candidate.gains[3]=math.min(4,2.8*math.sqrt(j.candidate.gains[1]))
        j.candidate.maxSpeed=j.candidate.maxSpeed*.85
        j.leg=1; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.legStart={x=s.x,z=s.z}; j.overshoot=0; self:reset(j.heading)
      end
      return commands,("AUTO TEST %d/5 | %.2fm %.2fm/s | pass %d/5"):format(math.min(j.leg,5),math.sqrt(dx*dx+dz*dz),math.sqrt(f.vx^2+f.vz^2),j.retry+1)
    end
    local commands={}
    if j.phase=="WARMUP" or j.phase=="PULSE" then
      local direction=directions[(j.index-1)%4+1]
      commands[self.names[math.floor((j.index-1)/4)+1]]={x=direction[1],y=direction[2],power=testThrottle}
    end
    return commands,("AUTO CAL %d/%d  %s"):format(math.min(j.index,#self.names*4),#self.names*4,j.phase)
  end
  return A
end)()
-- END AUTOPILOT CORE

print("Discovering peripherals...")
local monitor = peripheral.find("monitor")
local directGpu, directDisplay, directWidth, directHeight, directGpuError
local directGpuEnabled = true
local findAvionics
local function setupDirectGpu()
  local gpuFound = peripheral.wrap("directgpu_5") or peripheral.find("directgpu")
  yieldComputer()
  if not gpuFound then directGpuError = "DirectGPU peripheral not found"; return false end
  local listOk, displays = pcall(gpuFound.listDisplays)
  yieldComputer()
  local display = listOk and type(displays) == "table" and displays[1]
  local function getInfo(id)
    if id == nil then return nil, "no display ID returned" end
    local ok, info = pcall(gpuFound.getDisplayInfo, id)
    yieldComputer()
    if not ok then return nil, tostring(info) end
    if not info or type(info.pixelWidth) ~= "number" or type(info.pixelHeight) ~= "number" then
      return nil, "display info has no pixel dimensions"
    end
    return info
  end
  local info, infoError = getInfo(display)
  local setupError
  if not info then
    local created, newDisplay = pcall(gpuFound.autoDetectAndCreateDisplayWithResolution, 1)
    yieldComputer()
    if created then display = newDisplay; info, infoError = getInfo(display)
    else setupError = "create display: " .. tostring(newDisplay) end
  end
  if info then
    local mouseOk, mouseError = pcall(gpuFound.enableMouseEvents, display, true, false)
    yieldComputer()
    if not mouseOk then
      directGpuError = "Mouse setup failed: " .. tostring(mouseError)
      directGpu = nil
      return false
    end
    directGpu, directDisplay = gpuFound, display
    directWidth, directHeight = info.pixelWidth, info.pixelHeight
    directGpuError = nil
    return true
  else
    local scanOk, monitorInfo = pcall(gpuFound.autoDetectMonitor)
    yieldComputer()
    if not scanOk then
      setupError = setupError or ("monitor scan: " .. tostring(monitorInfo))
      directGpuError = "Setup failed: " .. setupError
    elseif monitorInfo ~= nil then
      directGpuError = "Display setup: " .. tostring(setupError or infoError)
    else
      directGpuError = "No monitor near DirectGPU block; move it beside monitor"
    end
    directGpu = nil
    return false
  end
end
-- Resolve devices behind the Malahit hub nested behind the wired peripheral hub.
findAvionics = function(kind)
  local direct = peripheral.find(kind)
  yieldComputer()
  if direct then return direct end
  for _, rootName in ipairs(peripheral.getNames()) do
    local root = peripheral.wrap(rootName)
    yieldComputer()
    if root and type(root.getNamesRemote) == "function" then
      local ok, names = pcall(root.getNamesRemote)
      yieldComputer()
      if ok and type(names) == "table" then
        for _, remoteName in ipairs(names) do
          local typeOk, remoteType = pcall(root.getTypeRemote, remoteName)
          yieldComputer()
          local types = type(remoteType) == "table" and remoteType or {remoteType}
          for _, remoteKind in ipairs(types) do
            if remoteKind == kind then
              return setmetatable({}, {__index=function(_, method)
                return function(...) return root.callRemote(remoteName, method, ...) end
              end})
            elseif remoteKind == "malahit_hub" or remoteKind == "peripheral_hub" then
              local childOk, childNames = pcall(root.callRemote, remoteName, "getNamesRemote")
              yieldComputer()
              if childOk and type(childNames) == "table" then
                for _, childName in ipairs(childNames) do
                  local childTypeOk, childType = pcall(root.callRemote, remoteName, "getTypeRemote", childName)
                  yieldComputer()
                  local childTypes = type(childType) == "table" and childType or {childType}
                  for _, childKind in ipairs(childTypes) do
                    if childKind == kind then
                      return setmetatable({}, {__index=function(_, method)
                        return function(...) return root.callRemote(remoteName, "callRemote", childName, method, ...) end
                      end})
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
monitor = monitor or findAvionics("monitor")
local monitorName
if monitor then
  local ok, name = pcall(peripheral.getName, monitor)
  if ok then monitorName = name end
end
local function findThrusters()
  local found = {}
  local function add(name, device)
    local id = name:match("([^/]+)$") or name
    if device and not found[id] then found[id] = device end
  end
  for _, name in ipairs(peripheral.getNames()) do
    local kinds = {peripheral.getType(name)}
    for _, kind in ipairs(kinds) do
      if kind == "liquid_vector_thruster" then
        add(name, peripheral.wrap(name))
      end
    end
    yieldComputer()
  end
  local count = 0
  for _ in pairs(found) do count = count + 1 end
  if count >= 8 then return found end
  for _, name in ipairs(peripheral.getNames()) do
    local hub = peripheral.wrap(name)
    yieldComputer()
    if hub and type(hub.getNamesRemote) == "function" then
      local function scan(remoteHub, path, depth)
        if depth > 2 then return end
        local ok, names = pcall(remoteHub.getNamesRemote)
        yieldComputer()
        if not ok or type(names) ~= "table" then return end
        for _, childName in ipairs(names) do
          local typeOk, remoteType = pcall(remoteHub.getTypeRemote, childName)
          yieldComputer()
          local types = type(remoteType) == "table" and remoteType or {remoteType}
          local child = setmetatable({}, {__index=function(_, method)
            return function(...) return remoteHub.callRemote(childName, method, ...) end
          end})
          for _, kind in ipairs(types) do
            if kind == "liquid_vector_thruster" then
              add(path .. "/" .. childName, child)
            elseif (kind == "peripheral_hub" or kind == "malahit_hub") and depth < 2 then
              scan(child, path .. "/" .. childName, depth + 1)
            end
          end
        end
      end
      scan(hub, name, 1)
    end
  end
  return found
end
print("Discovering thrusters...")
local thrusters = findThrusters()
local thrusterCount = 0
for _ in pairs(thrusters) do thrusterCount = thrusterCount + 1 end
if thrusterCount ~= 8 then
  error("Expected 8 liquid_vector_thruster peripherals, found " .. thrusterCount .. "; check the wired hub")
end
print("Discovering avionics...")
local altitudeSensor = findAvionics("altitude_sensor")
local navigationTable = findAvionics("navigation_table")
local gimbalSensor = findAvionics("gimbal_sensor")
print("Setting up display...")
if directGpuEnabled then setupDirectGpu() else directGpuError = "DirectGPU paused for stability; using CC monitor" end
local frame = basalt.getMainFrame()
local monitorFrame
frame.background = colors.black
if monitorFrame then monitorFrame.background = colors.black end
local screenWidth, screenHeight
screenWidth, screenHeight = term.getSize()
local compact = screenWidth < 49 or screenHeight < 19
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local burner = redstone.getAnalogOutput("front") or 0
local burnerCommand, vectorX, vectorY, power = burner, 0, 0, 0
local burnerSlewPerSecond = 6
local calibrated = fs.exists("airship-calibrated")
local running, manualBurner, burnerSweep, calibrationRunning = true, nil, nil, false
local navNames={}; for name in pairs(thrusters) do navNames[#navNames+1]=name end; table.sort(navNames)
local navController=Autopilot.new(navNames)
local autopilotCommands, fullCalibration, autopilotNotice
local autopilotNoticeColor=colors.orange
local function saveAutopilotProfile(profile)
  assert(Autopilot.validProfile(profile,navNames),"Invalid autopilot calibration")
  local path="airship-autopilot.cfg"
  local encoded=textutils.serialize(profile)
  local file=assert(fs.open(path..".tmp","w"),"Cannot write autopilot calibration")
  file.write(encoded); file.close()
  if fs.exists(path..".bak") then fs.delete(path..".bak") end
  local previous=fs.exists(path)
  if previous then fs.move(path,path..".bak") end
  local ok,err=pcall(fs.move,path..".tmp",path)
  if not ok then if previous then fs.move(path..".bak",path) end; error(err) end
end
if fs.exists("airship-autopilot.cfg") then
  local ok,profile=pcall(function()
    local file=assert(fs.open("airship-autopilot.cfg","r")); local value=textutils.unserialize(file.readAll()); file.close(); return value
  end)
  if ok and Autopilot.validProfile(profile,navNames) then navController.profile=profile
  else autopilotNotice="Saved autopilot profile is invalid or thrusters changed; run FULL AUTO CAL." end
end
local sweepConfirmAt
local sweepSummary
local sweepSummaryUntil
local gpuButtons = {}
local gpuTab, gpuHeightPage = "FLIGHT", 1
local gpuRenderRevision = 0
local learnedHover = {}
local altitudeSensorPressure
local shipHeading
local yawRate
local shipTilt
local measuredThrust = 0
local status, pidSummary, posText, altText, sysText, navText, xInput, heightInput, pidInput, calibrateButton
-- Conservative starting gains; GPS velocity feedback compensates for variable sample timing.
local pid = {
  horizontalX = {p=.035, i=.0005, d=.18, limit=.25, integral=0},
  horizontalZ = {p=.035, i=.0005, d=.18, limit=.25, integral=0}
}
local gainLimits = {{.0001,1}, {0,.05}, {0,4}}
local function applyGainValues(values)
  for i, bounds in ipairs(gainLimits) do
    if type(values[i]) ~= "number" or values[i] ~= values[i] or values[i] < bounds[1] or values[i] > bounds[2] then return false end
  end
  pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d = values[1], values[2], values[3]
  pid.horizontalZ.p, pid.horizontalZ.i, pid.horizontalZ.d = values[1], values[2], values[3]
  pid.horizontalX.integral, pid.horizontalZ.integral = 0, 0
  if navController.profile then
    navController.profile.gains={values[1],values[2],values[3]}
    navController:reset(shipHeading)
  end
  return true
end
local function readGainFile()
  if not fs.exists("airship-pid.cfg") then return end
  local file = fs.open("airship-pid.cfg", "r")
  local values = {}
  for token in file.readAll():gmatch("%S+") do
    local value = tonumber(token)
    if not value then file.close(); return end
    values[#values + 1] = value
  end
  file.close()
  if #values >= 3 then applyGainValues(values) end
end
local function gainValues()
  return {pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d}
end
local function saveGainFile(values)
  if navController.profile then saveAutopilotProfile(navController.profile) end
  local file=assert(fs.open("airship-pid.cfg","w"),"Cannot write PID settings")
  file.write(table.concat(values," ")); file.close()
end
if fs.exists("airship-hover.cfg") then
  local file = fs.open("airship-hover.cfg", "r")
  for line in file.readAll():gmatch("[^\r\n]+") do
    local pressure, level, height = line:match("^([%-%.%d]+)%s+([%-%.%d]+)%s*([%-%.%d]*)$")
    pressure, level, height = tonumber(pressure), tonumber(level), tonumber(height)
    if pressure and level and level >= 0 and level <= 15 then
      local key = ("%.3f"):format(height or pressure)
      learnedHover[key] = {pressure=pressure, level=level, height=height}
    end
  end
  file.close()
end
local function hoverBias(height)
  local points = {}
  for _, sample in pairs(learnedHover) do
    if type(sample.height) == "number" then
      points[#points + 1] = {sample.height, sample.level}
    end
  end
  table.sort(points, function(a, b) return a[1] < b[1] end)
  if #points == 0 then return nil end
  if height <= points[1][1] then return points[1][2] end
  if height >= points[#points][1] then return points[#points][2] end
  for i = 1, #points - 1 do
    local low, high = points[i], points[i + 1]
    if height >= low[1] and height <= high[1] then
      if high[1] == low[1] then return high[2] end
      local fraction = (height - low[1]) / (high[1] - low[1])
      return low[2] + (high[2] - low[2]) * fraction
    end
  end
  return nil
end
local function saveHoverMap()
  return pcall(function()
    local file = assert(fs.open("airship-hover.cfg.tmp", "w"))
    for _, sample in pairs(learnedHover) do file.writeLine(("%.4f %.3f %.2f"):format(sample.pressure, sample.level, sample.height or 0)) end
    file.close()
    local hadPrevious = fs.exists("airship-hover.cfg")
    if fs.exists("airship-hover.cfg.bak") then fs.delete("airship-hover.cfg.bak") end
    if hadPrevious then fs.move("airship-hover.cfg", "airship-hover.cfg.bak") end
    local moved, err = pcall(fs.move, "airship-hover.cfg.tmp", "airship-hover.cfg")
    if not moved then
      if hadPrevious then fs.move("airship-hover.cfg.bak", "airship-hover.cfg") end
      error(err)
    end
    if fs.exists("airship-hover.cfg.bak") then fs.delete("airship-hover.cfg.bak") end
  end)
end
local clearHoverConfirmAt
local function clearHoverMap()
  if not clearHoverConfirmAt or os.clock() - clearHoverConfirmAt >= 8 then
    clearHoverConfirmAt = os.clock()
    status.text = "Tap CLEAR again within 8 seconds to erase saved heights."
    status.foreground = colors.orange
    return
  end
  clearHoverConfirmAt = nil
  local previous = learnedHover
  learnedHover = {}
  local ok, err = saveHoverMap()
  if ok then
    gpuHeightPage = 1
    status.text = "All saved calibration heights cleared."
    status.foreground = colors.lime
  else
    learnedHover = previous
    status.text = "Could not clear saved heights: " .. tostring(err)
    status.foreground = colors.red
  end
end
if not navController.profile then readGainFile() end
if navController.profile then
  applyGainValues(navController.profile.gains)
  autopilotNotice="Full autopilot calibration loaded."
  autopilotNoticeColor=colors.lime
end
local velocity = {x=0, y=0, z=0}
local previousPosition, previousSample
local sampleDt = .5
local altitudeSensorHeight
local altitudeSensorReady = false
local lastGpsAt
local pressureAt, headingAt, yawAt
local function navigationSample()
  local now=os.clock()
  if not position or not lastGpsAt or now-lastGpsAt>2 or not pressureAt or now-pressureAt>2 or not headingAt or now-headingAt>2 or not yawAt or now-yawAt>2 then return nil end
  return {t=now,x=position.x,z=position.z,vx=velocity.x,vz=velocity.z,heading=shipHeading,yaw=yawRate,pressure=altitudeSensorPressure,tilt=shipTilt}
end
local readoutSamples,readoutSampleAt,readoutGoalKey={},nil,nil
local function navigationReadout()
  local goalKey=target and (target.x..":"..target.z) or nil
  if goalKey~=readoutGoalKey then readoutSamples,readoutSampleAt,readoutGoalKey={},nil,goalKey end
  if position and lastGpsAt and lastGpsAt~=readoutSampleAt then
    local distance=target and math.sqrt((target.x-position.x)^2+(target.z-position.z)^2) or nil
    readoutSamples[#readoutSamples+1]={t=lastGpsAt,vx=velocity.x,vz=velocity.z,distance=distance}
    readoutSampleAt=lastGpsAt
    while #readoutSamples>2 and lastGpsAt-readoutSamples[1].t>4 do table.remove(readoutSamples,1) end
  end
  local avgX,avgZ,n=0,0,#readoutSamples
  for _,sample in ipairs(readoutSamples) do avgX,avgZ=avgX+sample.vx,avgZ+sample.vz end
  if n>0 then avgX,avgZ=avgX/n,avgZ/n else avgX,avgZ=velocity.x,velocity.z end
  local speed=math.sqrt(avgX^2+avgZ^2)
  if not position then speed=0 end
  local distance=position and target and math.sqrt((target.x-position.x)^2+(target.z-position.z)^2) or nil
  local first,last=readoutSamples[1],readoutSamples[n]
  local span=first and last and last.t-first.t or 0
  local closing=span>0 and first.distance and last.distance and (first.distance-last.distance)/span or 0
  local eta=distance and (mode=="DESTINATION REACHED" and 0 or (mode=="AUTOPILOT" and span>=2 and closing>.1 and distance/closing or nil)) or nil
  local etaText=eta and (math.floor(eta/60)..":"..("%02d"):format(math.floor(eta%60))) or "--"
  return speed,distance,etaText
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function outputs(cooperative)
  for name, thruster in pairs(thrusters) do
    local command=autopilotCommands and autopilotCommands[name]
    local x,y,thrust=vectorX,vectorY,power
    if autopilotCommands then x,y,thrust=command and command.x or 0,command and command.y or 0,command and command.power or 0 end
    thruster.setVector(x, y)
    if cooperative then yieldComputer() end
    command=autopilotCommands and autopilotCommands[name]
    thrust=autopilotCommands and (command and command.power or 0) or power
    thruster.setThrustNormalized(thrust)
    if cooperative then yieldComputer() end
  end
  redstone.setAnalogOutput("front", burner)
end
local function stop()
  calibrationRunning = false
  autopilotNotice=nil
  navController:cancel(); autopilotCommands,fullCalibration=nil,nil
  if burnerSweep then
    sweepSummary = ("Sweep canceled. Saved %d/11; skipped %d."):format(burnerSweep.saved, burnerSweep.skipped)
    sweepSummaryUntil = os.clock() + 15
  end
  manualBurner = nil
  burnerSweep = nil
  vectorX, vectorY, power, burner, burnerCommand, mode = 0, 0, 0, 0, 0, "STOPPED"
  pcall(outputs)
end
local updateRequested = false
local function requestProgramUpdate()
  if not fs.exists("/airship-updater.lua") then
    status.text = "Updater missing; install airship-updater.lua once."
    status.foreground = colors.red
    return
  end
  updateRequested = true
  stop()
  basalt.stop()
end
local function setFlightMode(nextMode)
  calibrationRunning = false
  autopilotNotice=nil
  navController:cancel(); autopilotCommands,fullCalibration=nil,nil
  manualBurner = nil
  burnerSweep = nil
  vectorX, vectorY, power, mode = 0, 0, 0, nextMode
  pid.horizontalX.integral, pid.horizontalZ.integral = 0, 0
end
local function locate()
  local now = os.clock()
  if altitudeSensor then
    local ok, height, verticalSpeed, pressure = pcall(function()
      local height = altitudeSensor.getHeight()
      yieldComputer()
      local verticalSpeed = altitudeSensor.getVerticalSpeed()
      yieldComputer()
      local pressure = altitudeSensor.getAirPressure()
      yieldComputer()
      return height, verticalSpeed, pressure
    end)
    if ok and type(height) == "number" and height == height and math.abs(height) < math.huge then
      altitudeSensorHeight, altitudeSensorReady = height, true
      if type(pressure) == "number" and pressure == pressure and pressure >= 0 and math.abs(pressure) < math.huge then altitudeSensorPressure = pressure; pressureAt=os.clock() end
      if position then position.y = height end
      if type(verticalSpeed) == "number" and verticalSpeed == verticalSpeed and math.abs(verticalSpeed) < math.huge then velocity.y = verticalSpeed end
    end
  end
  if navigationTable then
    local ok, heading = pcall(navigationTable.getHeadingRad)
    yieldComputer()
    if ok and type(heading) == "number" and heading == heading and math.abs(heading) < math.huge then shipHeading = heading; headingAt=os.clock() end
  end
  if gimbalSensor then
    local ok, rates = pcall(gimbalSensor.getAngularRatesRad)
    yieldComputer()
    local gravityOk,gravity=pcall(gimbalSensor.getGravity)
    yieldComputer()
    local headingRate
    if ok then headingRate,shipTilt=Autopilot.headingRate(rates,gravityOk and gravity or nil) end
    if headingRate then yawRate=headingRate; yawAt=os.clock() end
  end
  local x, y, z = gps.locate(.5, false)
  now=os.clock()
  if x then
    local centerX,centerZ=x,z
    if shipHeading then centerX,centerZ=x+2*math.sin(shipHeading),z+2*math.cos(shipHeading) end
    if previousPosition and previousSample then
      sampleDt = clamp(now - previousSample, .01, 1.5)
      velocity.x = velocity.x * .5 + ((centerX - previousPosition.x) / sampleDt) * .5
      if not altitudeSensorReady then velocity.y = velocity.y * .5 + ((y - previousPosition.y) / sampleDt) * .5 end
      velocity.z = velocity.z * .5 + ((centerZ - previousPosition.z) / sampleDt) * .5
    end
    position = {x=centerX, y=altitudeSensorReady and altitudeSensorHeight or y, z=centerZ}
    previousPosition, previousSample = position, now
    lastGpsAt = now
    if not altitude then altitude = math.floor(position.y + .5) end
    if mode == "WAITING FOR GPS" then mode = "HOLDING ALTITUDE" end
    return true
  elseif lastGpsAt and os.clock() - lastGpsAt > 2 then
    position = nil
    previousPosition, previousSample = nil, nil
    velocity = {x=0, y=0, z=0}
    lastGpsAt = nil
    if mode ~= "STOPPED" then
      vectorX, vectorY, power, burner, burnerCommand = 0, 0, 0, 0, 0
      if burnerSweep then
        sweepSummary = ("Sweep aborted: GPS lost. Saved %d/11; skipped %d."):format(burnerSweep.saved, burnerSweep.skipped)
        sweepSummaryUntil = os.clock() + 15
      end
      burnerSweep, manualBurner = nil, nil
      if fullCalibration then autopilotNotice="GPS lost; full calibration stopped. Previous profile kept."; autopilotNoticeColor=colors.red
      else autopilotNotice=nil end
      navController:cancel(); autopilotCommands,fullCalibration=nil,nil; calibrationRunning=false
      mode = mode == "AUTOPILOT" and "GPS LOST - RE-ENGAGE" or "WAITING FOR GPS"
    end
  end
  return false
end
local function updateFlight(gpsFresh)
  if not position then
    vectorX, vectorY, power, burner, burnerCommand = 0, 0, 0, 0, 0
    if mode ~= "STOPPED" and mode ~= "GPS LOST - RE-ENGAGE" then mode = "WAITING FOR GPS" end
    return
  end
  if mode == "STOPPED" or mode == "GPS LOST - RE-ENGAGE" then return end
  if not altitude then mode = "SET ALTITUDE"; return end
  if burnerSweep then
    burnerCommand = burnerCommand + clamp(burnerSweep.level - burnerCommand, -burnerSlewPerSecond * sampleDt, burnerSlewPerSecond * sampleDt)
    burner = math.floor(burnerCommand + .5)
  elseif manualBurner ~= nil then
    burnerCommand, burner = manualBurner, manualBurner
  else
    local mappedBurner = hoverBias(altitude)
    if mappedBurner then
      burnerCommand = burnerCommand + clamp(mappedBurner - burnerCommand, -burnerSlewPerSecond * sampleDt, burnerSlewPerSecond * sampleDt)
      burner = math.floor(burnerCommand + .5)
    end
  end
  if fullCalibration then
    vectorX,vectorY,power=0,0,0
    if not gpsFresh then autopilotCommands={}; return end
    local commands,message,completed=navController:calibrate(navigationSample())
    autopilotCommands=commands or {}
    if completed then
      local saved,err=pcall(saveAutopilotProfile,completed)
      if saved then navController.profile=completed; applyGainValues(completed.gains); autopilotNotice=("FULL AUTO CAL saved: %.1f b/s, %.0f%% thrust."):format(completed.maxSpeed,(completed.maxThrottle or .35)*100); autopilotNoticeColor=colors.lime
      else autopilotNotice="Calibration passed but could not save: "..tostring(err); autopilotNoticeColor=colors.red end
    elseif not commands then autopilotNotice=message; autopilotNoticeColor=colors.red
    else autopilotNotice=message; autopilotNoticeColor=colors.orange end
    if completed or not commands then
      fullCalibration,calibrationRunning=false,false
      mode="HOLDING ALTITUDE"; autopilotCommands={}; navController:reset(shipHeading)
    end
    return
  end
  if (mode == "AUTOPILOT" or mode == "FINAL TURN" or mode == "DESTINATION REACHED") and target then
    vectorX,vectorY,power=0,0,0
    if not gpsFresh then autopilotCommands={}; return end
    local sample=navigationSample()
    if not Autopilot.validSample(sample) or navController.profile and navController.profile.tiltReference~=nil and sample.tilt==nil then
      autopilotCommands={}; mode="NAV LOST - RE-ENGAGE"; autopilotNotice="Fresh heading, yaw, gravity and pressure required; re-engage AUTO."
      autopilotNoticeColor=colors.red
      return
    end
    local wasReached=mode=="DESTINATION REACHED"
    local wasFinalTurn=mode=="FINAL TURN"
    local commands,arrived=navController:navigate(sample,target)
    autopilotCommands=commands or {}
    if arrived and mode=="AUTOPILOT" then
      local dx,dz=target.x-sample.x,target.z-sample.z
      if dx*dx+dz*dz>.25 then navController:setHeadingTarget(math.atan(dx,dz)) end
      target={x=sample.x,z=sample.z}
      mode="FINAL TURN"
    elseif wasFinalTurn then mode=arrived and "DESTINATION REACHED" or "FINAL TURN"
    elseif wasReached then mode="DESTINATION REACHED"
    else mode="AUTOPILOT" end
  end
end

local function savedHoverPoints()
  local points = {}
  for _, sample in pairs(learnedHover) do if type(sample.height) == "number" then points[#points + 1] = sample end end
  table.sort(points, function(a,b) return a.height < b.height end)
  return points
end
local function savedAltitudes()
  local heights = {}
  for _, sample in ipairs(savedHoverPoints()) do heights[#heights + 1] = sample.height end
  return heights
end
local function selectAltitudePreset()
  if not position then status.text = "Waiting for GPS before selecting a height."; return end
  local heights = savedAltitudes()
  if #heights == 0 then
    status.text = "No saved calibration altitudes. Run a sweep or lock a burner level."
    status.foreground = colors.orange
    return
  end
  local selected = 1
  for i, height in ipairs(heights) do
    if altitude and height > altitude + .001 then selected = i; break end
  end
  altitude = heights[selected]
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Saved altitude %d/%d: target Y %.1f"):format(selected, #heights, altitude)
  status.foreground = colors.lightBlue
end
local function selectSavedAltitude(index)
  local height = savedAltitudes()[index]
  if height then
    altitude = height
    setFlightMode("HOLDING ALTITUDE")
    status.text = ("Saved target selected: Y %.1f"):format(height)
    status.foreground = colors.lightBlue
  end
end
local function lockHoverLevel()
  if manualBurner == nil then
    status.text = "Press M to set the manual burner level before locking."; status.foreground = colors.orange; return
  end
  if not altitudeSensorReady or not altitudeSensorPressure then
    status.text = "Lock needs Avionics altitude and pressure readings."; status.foreground = colors.red; return
  end
  if not position or math.abs(velocity.y) >= .12 then
    status.text = "Wait until vertical speed is below 0.12, then lock."; status.foreground = colors.orange; return
  end
  local level = redstone.getAnalogOutput("front")
  local pressure = tonumber(("%.4f"):format(altitudeSensorPressure))
  local key = ("%.3f"):format(altitudeSensorHeight)
  learnedHover[key] = {pressure=pressure, level=level, height=altitudeSensorHeight}
  if saveHoverMap() then
    status.text = ("Locked signal %d at Y %.1f / pressure %.3f"):format(level, altitudeSensorHeight, altitudeSensorPressure)
    status.foreground = colors.lime
  else
    status.text = "Level captured in memory, but could not save airship-hover.cfg."; status.foreground = colors.red
  end
end
local function finishBurnerSweep(message)
  local sweep = burnerSweep
  burnerSweep, manualBurner = nil, nil
  if position then altitude = math.floor(position.y + .5) end
  setFlightMode("HOLDING ALTITUDE")
  sweepSummary = (message and (message .. " ") or "Sweep finished. ") .. ("Saved %d/11; skipped %d."):format(sweep.saved, sweep.skipped)
  sweepSummaryUntil = os.clock() + 15
  status.text = sweepSummary
  status.foreground = message and colors.red or colors.lime
end
local function startBurnerSweep()
  if calibrationRunning then status.text="Finish or cancel autopilot calibration first."; return end
  autopilotNotice=nil
  if type(burnerSweep) == "table" then status.text = "Burner sweep is already running."; return end
  if mode == "STOPPED" then status.text = "Resume flight control before starting the sweep."; status.foreground = colors.red; return end
  if not position or not altitudeSensorReady or not altitudeSensorPressure then
    status.text = "Sweep needs GPS and Avionics altitude/pressure."; status.foreground = colors.red; return
  end
  if sweepConfirmAt and os.clock() - sweepConfirmAt < 8 then
    sweepSummary = nil
    burnerSweep = {level=15, changedAt=os.clock(), stableSince=nil, saved=0, skipped=0}
    manualBurner, vectorX, vectorY, power, target = nil, 0, 0, 0, nil
    mode = "BURNER SWEEP"
    status.text = "Sweep started: levels 15 down to 5"; status.foreground = colors.orange
    sweepConfirmAt = nil
  else
    sweepConfirmAt = os.clock()
    status.text = "Sweep armed. Confirm again within 8 seconds."; status.foreground = colors.orange
  end
end
local function updateBurnerSweep()
  if type(burnerSweep) ~= "table" then return end
  local now = os.clock()
  if not position or not altitudeSensorPressure then finishBurnerSweep("Sweep aborted: GPS or pressure reading lost."); return end
  local atLevel = math.abs(burnerCommand - burnerSweep.level) < .05
  if atLevel and math.abs(velocity.y) < .12 then burnerSweep.stableSince = burnerSweep.stableSince or now
  else burnerSweep.stableSince = nil end
  if burnerSweep.stableSince and now - burnerSweep.stableSince >= 4 then
    local pressure = tonumber(("%.4f"):format(altitudeSensorPressure))
    local key = ("%.3f"):format(altitudeSensorHeight)
    learnedHover[key] = {pressure=pressure, level=burnerSweep.level, height=altitudeSensorHeight}
    if not saveHoverMap() then finishBurnerSweep("Sweep stopped: calibration file could not be saved."); return end
    burnerSweep.saved = burnerSweep.saved + 1
    if burnerSweep.level == 5 then finishBurnerSweep(); return end
    burnerSweep.level = burnerSweep.level - 1
    burnerSweep.changedAt, burnerSweep.stableSince = now, nil
  elseif now - burnerSweep.changedAt >= 45 then
    burnerSweep.skipped = burnerSweep.skipped + 1
    if burnerSweep.level == 5 then finishBurnerSweep(); return end
    burnerSweep.level = burnerSweep.level - 1
    burnerSweep.changedAt, burnerSweep.stableSince = now, nil
  end
  status.text = ("SWEEP %d/15 | saved %d, skipped %d"):format(burnerSweep.level, burnerSweep.saved, burnerSweep.skipped)
  status.foreground = colors.orange
end
local function toggleManualBurner()
  if type(burnerSweep) == "table" then status.text = "Finish or stop the burner sweep first."; return end
  if manualBurner ~= nil then
    manualBurner = nil
    if position then altitude = math.floor(position.y + .5) end
    setFlightMode("HOLDING ALTITUDE")
    status.text = "Auto altitude hold restored at current height."; status.foreground = colors.lime
  else
    manualBurner = redstone.getAnalogOutput("front")
    vectorX, vectorY, power, target = 0, 0, 0, nil
    mode = "MANUAL BURNER"
    status.text = ("Manual burner: %d/15; Up/Down adjusts, K locks"):format(manualBurner)
    status.foreground = colors.orange
  end
end
local function adjustManualBurner(delta)
  if manualBurner == nil then return false end
  manualBurner = clamp(manualBurner + delta, 0, 15)
  status.text = ("Manual burner: %d/15; settle, then K to lock"):format(manualBurner)
  status.foreground = colors.orange
  return true
end
local function readThrusterPower()
  local total, count = 0, 0
  for _, thruster in pairs(thrusters) do
    local ok, value = pcall(thruster.getPower)
    yieldComputer()
    if ok and type(value) == "number" and value == value then total, count = total + value, count + 1 end
  end
  measuredThrust = count > 0 and total / count * 15 or 0
end

local function gpuLayout()
  local w, h = directWidth, directHeight
  local margin = math.max(12, math.floor(w * .02))
  local gap = math.max(8, math.floor(w * .012))
  local titleH = math.max(40, math.floor(h * .08))
  local tabY, tabH = titleH, math.max(38, math.floor(h * .07))
  local contentY = tabY + tabH + math.max(12, math.floor(h * .025))
  local buttonH = math.max(34, math.floor(h * .07))
  local buttonY = h - margin - buttonH
  return {w=w,h=h,margin=margin,gap=gap,titleH=titleH,tabY=tabY,tabH=tabH,contentY=contentY,buttonH=buttonH,buttonY=buttonY}
end
local function addGpuButton(x, y, w, h, label, color, action, align)
  gpuButtons[#gpuButtons + 1] = {x=x,y=y,w=w,h=h,label=label,color=color,action=action,align=align}
end
local function renderDirectMonitor()
  if not directGpu then return end
  local gpu, display = directGpu, directDisplay
  local revision = gpuRenderRevision
  local function yieldFrame()
    yieldComputer()
    if revision ~= gpuRenderRevision then error("DirectGPU frame superseded", 0) end
  end
  local l = gpuLayout()
  local w, h = l.w, l.h
  local function rect(x,y,rw,rh,color)
    gpu.fillRect(display,x,y,rw,rh,color[1],color[2],color[3])
    yieldFrame()
  end
  local function txt(value,x,y,size,color,style)
    gpu.drawText(display,tostring(value),x,y,color[1],color[2],color[3],"Arial",size,style or "plain")
    yieldFrame()
  end
  local bg,panel,ink,muted = {10,17,29},{20,32,48},{229,240,250},{143,164,184}
  gpu.clear(display,bg[1],bg[2],bg[3])
  yieldFrame()
  rect(0,0,w,l.titleH,{15,29,47})
  local titleSize = math.max(18,math.floor(h*.032))
  txt("AIRSHIP v" .. AIRSHIP_BUILD,l.margin,math.floor((l.titleH-titleSize)/2),titleSize,ink,"bold")
  txt(mode == "BURNER SWEEP" and "CALIBRATING" or mode,math.floor(w*.28),math.floor((l.titleH-titleSize)/2),math.max(14,math.floor(titleSize*.7)),mode == "BURNER SWEEP" and {255,190,90} or {88,220,190},"bold")
  local contentY = l.contentY
  if gpuTab == "FLIGHT" then
    local gap = l.gap
    local cardW = math.floor((w-2*l.margin-2*gap)/3)
    local cardY, cardH = contentY, math.floor(h*.29)
    local xs = {l.margin,l.margin+cardW+gap,l.margin+2*(cardW+gap)}
    for _,x in ipairs(xs) do rect(x,cardY,cardW,cardH,panel) end
    local small,big = math.max(14,math.floor(h*.022)),math.max(28,math.floor(h*.055))
    txt("ALTITUDE",xs[1]+14,cardY+12,small,muted,"bold")
    txt(position and ("%.1f m"):format(position.y) or "--",xs[1]+14,cardY+math.floor(cardH*.34),big,ink,"bold")
    local mappedLevel
    for _,sample in pairs(learnedHover) do if sample.height and altitude and math.abs(sample.height-altitude)<.01 then mappedLevel=sample.level; break end end
    txt(("TARGET %s  |  BURNER %s"):format(altitude and ("%.0f"):format(altitude) or "--",mappedLevel and tostring(mappedLevel) or "--"),xs[1]+14,cardY+math.floor(cardH*.70),small,muted)
    txt("VERTICAL SPEED",xs[2]+14,cardY+12,small,muted,"bold")
    txt(("%.2f"):format(velocity.y),xs[2]+14,cardY+math.floor(cardH*.34),big,ink,"bold")
    txt("blocks / sec",xs[2]+14,cardY+math.floor(cardH*.70),small,muted)
    local speedX,speedY,speedW=xs[2]+14,cardY+math.floor(cardH*.84),cardW-28
    rect(speedX,speedY,speedW,math.max(6,math.floor(h*.01)),{57,73,91})
    local ratio=math.max(-1,math.min(1,velocity.y/1.5)); local center=speedX+math.floor(speedW/2)
    rect(center,speedY,2,math.max(6,math.floor(h*.01)),{143,164,184})
    if ratio~=0 then local bar=math.max(2,math.floor(math.abs(ratio)*speedW/2)); rect(ratio>0 and center or center-bar,speedY,bar,math.max(6,math.floor(h*.01)),ratio>0 and {55,205,160} or {255,190,90}) end
    txt("BURNER",xs[3]+14,cardY+12,small,muted,"bold")
    txt(("%d / 15"):format(burner),xs[3]+14,cardY+math.floor(cardH*.34),big,{255,190,90},"bold")
    txt(("THRUST %.1f / 15"):format(measuredThrust),xs[3]+14,cardY+math.floor(cardH*.70),small,muted)
    local detailY=cardY+cardH+math.max(14,math.floor(h*.09)); local detailH=l.buttonY-detailY-l.margin
    rect(l.margin,detailY,w-2*l.margin,detailH,panel)
    local detailFont=math.max(11,math.min(small,math.floor(detailH*.19)))
    txt("FLIGHT STATUS",l.margin+14,detailY+8,detailFont,muted,"bold")
    txt(position and ("GPS  X %.1f     Y %.1f     Z %.1f"):format(position.x,position.y,position.z) or "GPS SIGNAL LOST",l.margin+14,detailY+math.floor(detailH*.34),detailFont,position and ink or {255,110,110})
    local speed,distance,eta=navigationReadout()
    txt(("MODE  %s     AV %s     HDG %s°     YAW %s rad/s"):format(mode,altitudeSensorReady and "READY" or "MISSING",shipHeading and ("%.0f"):format(math.deg(shipHeading)) or "--",yawRate and ("%.2f"):format(yawRate) or "--"),l.margin+14,detailY+math.floor(detailH*.52),detailFont,ink)
    txt(("SPEED  %.1f b/s     DISTANCE  %s     ETA  %s"):format(speed,distance and ("%.1f"):format(distance) or "--",eta),l.margin+14,detailY+math.floor(detailH*.70),detailFont,ink)
    txt(target and ("DESTINATION  X %.0f   Z %.0f"):format(target.x,target.z) or "NO DESTINATION SET",l.margin+14,detailY+math.floor(detailH*.87),detailFont,muted)
  elseif gpuTab == "CALIBRATION" then
    local titleSize=math.max(18,math.floor(h*.032))
    txt("AUTOPILOT / BURNER CALIBRATION",l.margin,contentY,titleSize,ink,"bold")
    if not fullCalibration then txt("15  →  5",l.margin,contentY+titleSize+10,math.max(14,math.floor(h*.023)),muted) end
    local barY=contentY+titleSize+math.max(34,math.floor(h*.07))
    local barGap=math.max(6,math.floor(w*.008)); local barW=math.floor((w-2*l.margin-10*barGap)/11)
    for i=0,10 do
      local level=15-i; local x=l.margin+i*(barW+barGap); local marked=false
      for _,sample in pairs(learnedHover) do if sample.level==level then marked=true; break end end
      local active=burnerSweep and burnerSweep.level==level
      rect(x,barY,barW,math.max(18,math.floor(h*.045)),active and {255,180,72} or marked and {55,205,160} or {57,73,91})
      txt(level,x+4,barY+math.max(22,math.floor(h*.05)),math.max(12,math.floor(h*.02)),marked and {115,235,195} or muted)
    end
    local saved=0; for _ in pairs(learnedHover) do saved=saved+1 end
    txt(("Saved points: %d     Burner: %d / 15     Vertical speed: %.2f"):format(saved,burner,velocity.y),l.margin,barY+math.floor(h*.11),math.max(15,math.floor(h*.025)),ink)
    if autopilotNotice then txt(autopilotNotice,l.margin,barY+math.floor(h*.17),math.max(12,math.floor(h*.019)),{255,190,90},"bold")
    elseif burnerSweep then txt(("SWEEP LEVEL %d   SAVED %d/11   SKIPPED %d"):format(burnerSweep.level,burnerSweep.saved,burnerSweep.skipped),l.margin,barY+math.floor(h*.17),math.max(15,math.floor(h*.025)),{255,190,90},"bold")
    elseif sweepSummary then txt(sweepSummary,l.margin,barY+math.floor(h*.17),math.max(14,math.floor(h*.022)),{255,190,90})
    else txt("SWEEP requires two taps to confirm. Each settled level saves automatically.",l.margin,barY+math.floor(h*.17),math.max(14,math.floor(h*.022)),muted) end
    local detailY=barY+math.floor(h*.25); local detailH=l.buttonY-detailY-l.margin
    rect(l.margin,detailY,w-2*l.margin,detailH,panel)
    local detailFont=math.max(11,math.min(math.floor(h*.023),math.floor(detailH*.2)))
    txt("SENSOR / SWEEP DETAIL",l.margin+14,detailY+8,detailFont,muted,"bold")
    txt(("AVIONICS ALTITUDE  %s     PRESSURE  %s"):format(altitudeSensorHeight and ("%.1f"):format(altitudeSensorHeight) or "--",altitudeSensorPressure and ("%.3f"):format(altitudeSensorPressure) or "--"),l.margin+14,detailY+math.floor(detailH*.38),detailFont,ink)
    local stable=burnerSweep and burnerSweep.stableSince and ("%.1f sec"):format(os.clock()-burnerSweep.stableSince) or "--"
    txt(("CURRENT OUTPUT  %d / 15     SETTLED  %s     GPS  %s"):format(burner,stable,position and "OK" or "LOST"),l.margin+14,detailY+math.floor(detailH*.68),detailFont,ink)
  else
    local heights=savedHoverPoints()
    local rowH=math.max(32,math.floor(h*.052)); local rowGap=math.max(4,math.floor(h*.008))
    local rows=math.max(1,math.floor((l.buttonY-contentY-math.max(32,math.floor(h*.05))-l.margin)/(rowH+rowGap)))
    local pages=math.max(1,math.ceil(#heights/rows)); gpuHeightPage=math.max(1,math.min(gpuHeightPage,pages))
    txt(("SAVED ALTITUDES   %d POINTS   PAGE %d / %d"):format(#heights,gpuHeightPage,pages),l.margin,contentY,math.max(18,math.floor(h*.032)),ink,"bold")
    if #heights==0 then txt("No saved calibration heights yet.",l.margin,contentY+math.floor(h*.09),math.max(16,math.floor(h*.026)),muted) end
  end
  for _,b in ipairs(gpuButtons) do
    rect(b.x,b.y,b.w,b.h,b.color)
    local textSize=math.max(13,math.floor(h*.022))
    local tx=b.align=="left" and b.x+12 or b.x+math.max(6,math.floor((b.w-#b.label*textSize*.55)/2))
    txt(b.label,tx,b.y+math.floor((b.h-textSize)/2),textSize,ink,"bold")
  end
  gpu.updateDisplay(display)
  yieldComputer()
end
local monitorStatus, monitorPos, monitorAlt, monitorSystem, monitorCalibrateButton, monitorGpuNotice, compactGpuNotice
local showGainValues
local lastGpuRender = 0
local gpuRedrawPending = false
local lastGpuCheck = 0
local lastThrustRead = 0
local function adjustAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  altitude = (altitude or math.floor(position.y + .5)) + delta
  setFlightMode("HOLDING ALTITUDE")
end
local function stepSavedAltitude(delta)
  if not position then status.text = "Waiting for GPS before changing altitude."; return end
  local heights = savedAltitudes()
  if #heights == 0 then adjustAltitude(delta); return end
  local current = altitude or position.y
  local index
  if delta > 0 then
    for i, height in ipairs(heights) do if height > current + .001 then index = i; break end end
    index = index or 1
  else
    for i = #heights, 1, -1 do if heights[i] < current - .001 then index = i; break end end
    index = index or #heights
  end
  selectSavedAltitude(index)
end
local function holdHere()
  if position then
    local canceledCalibration = calibrationRunning
    altitude = math.floor(position.y + .5); target = nil; setFlightMode("HOLDING ALTITUDE")
    if canceledCalibration then status.text = "Direction calibration canceled; holding altitude."; status.foreground = colors.orange end
  end
end
local function engageAutopilot()
  local function refuse(message)
    autopilotNotice,autopilotNoticeColor=message,colors.orange
    status.text,status.foreground=message,colors.orange
  end
  if calibrationRunning then refuse("Wait for calibration to finish, or cancel it first."); return end
  local sample=navigationSample()
  if not Autopilot.validSample(sample) then refuse("AUTO needs fresh GPS, heading, yaw rate and air pressure."); return end
  if not navController.profile then refuse("Run FULL AUTO CAL in the calibration menu first.")
  elseif navController.profile.tiltReference~=nil and sample.tilt==nil then refuse("AUTO needs a fresh gimbal gravity reading.")
  elseif not position or not target then refuse("GPS fix and destination are required.")
  else setFlightMode("AUTOPILOT"); navController:reset(sample.heading); autopilotNotice=nil; status.text = "Autopilot engaged"; status.foreground = colors.lime end
end
local function startFullCalibration()
  if calibrationRunning then status.text="Calibration is running; HOLD cancels it."; return end
  if burnerSweep then status.text="Finish or stop the burner sweep first."; return end
  if mode=="STOPPED" then
    autopilotNotice,autopilotNoticeColor="Set a stable burner level before FULL AUTO CAL.",colors.orange
    status.text,status.foreground=autopilotNotice,autopilotNoticeColor
    return
  end
  local sample=navigationSample()
  local ok,err=navController:begin(sample)
  if not ok then autopilotNotice,autopilotNoticeColor=err,colors.red; status.text=err; status.foreground=colors.red; return end
  manualBurner,burnerCommand=burner,burner
  vectorX,vectorY,power,target=0,0,0,nil
  autopilotCommands={}; fullCalibration,calibrationRunning=true,true
  mode="FULL AUTO CAL"; autopilotNotice="Measuring, then long cruise tests; HOLD cancels."
  autopilotNoticeColor=colors.orange
  status.text=autopilotNotice; status.foreground=colors.orange
end
local function testVector(x, y, label)
  calibrationRunning = false
  autopilotNotice=nil
  navController:cancel(); autopilotCommands,fullCalibration=nil,nil
  vectorX, vectorY, power, mode = -x, -y, .33, label
  status.text = "Low-power test: " .. label .. "  |  Press HOLD to stop"
  status.foreground = colors.orange
end
local function confirmCalibration()
  if calibrationRunning then status.text = "Direction calibration is already running."; return end
  if mode == "STOPPED" then status.text = "Resume flight control before calibrating."; status.foreground = colors.red; return end
  if not position then status.text = "Wait for GPS before calibrating."; status.foreground = colors.red; return end
  autopilotNotice=nil
  navController:cancel(); autopilotCommands=nil
  calibrationRunning = true
  local tests = {{.2,0,"X-"},{-.2,0,"X+"},{0,-.2,"Y+"},{0,.2,"Y-"}}
  basalt.schedule(function()
    for i, test in ipairs(tests) do
      if not calibrationRunning then return end
      vectorX, vectorY, power, mode = test[1], test[2], .33, "CAL " .. test[3]
      status.text = ("Direction check %d/4: %s; press HOLD to cancel."):format(i, test[3])
      status.foreground = colors.orange
      sleep(.75)
      if not calibrationRunning then return end
      vectorX, vectorY, power = 0, 0, 0
      sleep(.35)
    end
    if not calibrationRunning then return end
    local file = fs.open("airship-calibrated", "w")
    if not file then
      calibrationRunning = false
      status.text = "Tests finished, but calibration could not be saved."; status.foreground = colors.red
      return
    end
    file.write("east front, X right, Y forward")
    file.close()
    calibrationRunning, calibrated = false, true
    vectorX, vectorY, power, mode = 0, 0, 0, "HOLDING ALTITUDE"
    if calibrateButton then calibrateButton.text, calibrateButton.background = "DIR CHECK", colors.green end
    if monitorCalibrateButton then monitorCalibrateButton.text, monitorCalibrateButton.background = "DIR CHECK", colors.green end
    status.text = "Four low-power direction checks finished. Calibration saved."; status.foreground = colors.lime
  end)
end

local function createGpuButtons()
  if not directGpu then return end
  gpuButtons = {}
  local l=gpuLayout()
  local tabs={"FLIGHT","CALIBRATION","HEIGHTS"}
  local tabW=math.floor((l.w-2*l.margin-2*l.gap)/3)
  for i,tab in ipairs(tabs) do
    local selected=tab
    addGpuButton(l.margin+(i-1)*(tabW+l.gap),l.tabY,tabW,l.tabH,tab,tab==gpuTab and {43,90,132} or {28,43,60},function() gpuTab=selected end)
  end
  local function buttonRow(items,y)
    local gap=math.max(5,math.floor(l.w*.006)); local bw=math.floor((l.w-2*l.margin-(#items-1)*gap)/#items)
    for i,item in ipairs(items) do addGpuButton(l.margin+(i-1)*(bw+gap),y,bw,l.buttonH,item[1],item[3],item[4]) end
  end
  if gpuTab=="FLIGHT" then
    buttonRow({
      {"ALT -",nil,{35,62,91},function() stepSavedAltitude(-1) end},
      {"ALT +",nil,{35,62,91},function() stepSavedAltitude(1) end},
      {"SAVED",nil,{43,90,132},selectAltitudePreset},
      {"HOLD",nil,{35,62,91},holdHere},
      {"AUTO",nil,{42,112,83},engageAutopilot},
      {"STOP",nil,{125,48,56},stop},
    },l.buttonY)
  elseif gpuTab=="CALIBRATION" then
    addGpuButton(math.floor(l.w*.5),l.contentY+math.max(18,math.floor(l.h*.032))+4,
      math.floor(l.w*.5)-l.margin,math.max(24,math.floor(l.h*.045)),"FULL AUTO CAL",{46,107,130},startFullCalibration)
    if fullCalibration then addGpuButton(l.margin,l.contentY+math.max(18,math.floor(l.h*.032))+4,
      math.floor(l.w*.45)-l.margin,math.max(24,math.floor(l.h*.045)),"CANCEL AUTO CAL",{100,75,35},holdHere) end
    buttonRow({
      {"FWD",nil,{70,82,96},function() testVector(0,.2,"TEST Y+") end},
      {"BACK",nil,{70,82,96},function() testVector(0,-.2,"TEST Y-") end},
      {"CHECK",nil,calibrated and {55,130,90} or {130,78,31},confirmCalibration},
      {"MANUAL",nil,{100,75,35},toggleManualBurner},
      {"LOCK",nil,{100,75,35},lockHoverLevel},
      {"SWEEP 15-5",nil,{130,78,31},startBurnerSweep},
      {"UPD",nil,{46,107,130},requestProgramUpdate},
      {"STOP",nil,{125,48,56},stop},
      {clearHoverConfirmAt and os.clock()-clearHoverConfirmAt<8 and "CLEAR?" or "CLEAR",nil,{125,48,56},clearHoverMap},
    },l.buttonY)
  else
    local heights=savedHoverPoints()
    local rowH=math.max(32,math.floor(l.h*.052)); local rowGap=math.max(4,math.floor(l.h*.008))
    local topGap=math.max(32,math.floor(l.h*.05))
    local rows=math.max(1,math.floor((l.buttonY-l.contentY-topGap-l.margin)/(rowH+rowGap)))
    local pages=math.max(1,math.ceil(#heights/rows)); gpuHeightPage=math.max(1,math.min(gpuHeightPage,pages))
    local first=(gpuHeightPage-1)*rows+1
    for i=first,math.min(#heights,first+rows-1) do
      local sample=heights[i]; local y=l.contentY+topGap+(i-first)*(rowH+rowGap); local index=i
      addGpuButton(l.margin,y,l.w-2*l.margin,rowH,("Y %.1f     BURNER %d / 15     PRESSURE %.3f"):format(sample.height,sample.level,sample.pressure or 0),{20,32,48},function() selectSavedAltitude(index); gpuTab="FLIGHT" end,"left")
    end
    local navW=math.floor((l.w-2*l.margin-l.gap)/2)
    addGpuButton(l.margin,l.buttonY,navW,l.buttonH,"PREVIOUS",gpuHeightPage>1 and {35,62,91} or {40,48,57},function() gpuHeightPage=math.max(1,gpuHeightPage-1) end)
    addGpuButton(l.margin+navW+l.gap,l.buttonY,navW,l.buttonH,"NEXT",gpuHeightPage<pages and {35,62,91} or {40,48,57},function() gpuHeightPage=math.min(pages,gpuHeightPage+1) end)
  end
end
createGpuButtons()

local function createMonitorUi()
  if monitorFrame or not monitor then return end
  monitorFrame = basalt.createFrame(monitor)
  monitorFrame.background = colors.black
  local width = monitor.getSize()
  monitorFrame:addLabel({x=2, y=1, width=width-2, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
  monitorStatus = monitorFrame:addLabel({x=2, y=2, width=width-2, height=1, text="Starting...", foreground=colors.yellow})
  monitorPos = monitorFrame:addLabel({x=2, y=3, width=width-2, height=1, text="GPS --", foreground=colors.white})
  monitorAlt = monitorFrame:addLabel({x=2, y=4, width=width-2, height=1, text="ALT -- / TARGET --", foreground=colors.lightBlue})
  monitorSystem = monitorFrame:addLabel({x=2, y=5, width=width-2, height=1, text="BURNER 0/15  THRUST 0/15", foreground=colors.lightGray})
  monitorFrame:addButton({x=2, y=6, width=8, height=1, text="ALT -", background=colors.orange}):onClick(function() stepSavedAltitude(-1) end)
  monitorFrame:addButton({x=11, y=6, width=8, height=1, text="ALT +", background=colors.orange}):onClick(function() stepSavedAltitude(1) end)
  monitorFrame:addButton({x=2, y=7, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  monitorFrame:addButton({x=13, y=7, width=11, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  monitorFrame:addButton({x=25, y=7, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  monitorFrame:addButton({x=2, y=8, width=12, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  monitorFrame:addButton({x=15, y=8, width=12, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  monitorCalibrateButton = monitorFrame:addButton({x=2, y=9, width=14, height=1, text="DIR CHECK", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  monitorFrame:addButton({x=18, y=9, width=20, height=1, text="FULL AUTO CAL", background=colors.blue}):onClick(startFullCalibration)
  monitorFrame:addButton({x=2, y=10, width=14, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  monitorFrame:addButton({x=18, y=10, width=12, height=1, text="SAVED ALT", background=colors.blue}):onClick(selectAltitudePreset)
  monitorFrame:addButton({x=32, y=10, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
  monitorFrame:addButton({x=2, y=11, width=16, height=1, text="SWEEP 15-5", background=colors.orange}):onClick(startBurnerSweep)
  monitorFrame:addButton({x=20, y=11, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
  monitorFrame:addButton({x=32, y=11, width=12, height=1, text="CLEAR MAP", background=colors.red}):onClick(clearHoverMap)
  monitorGpuNotice = monitorFrame:addLabel({x=2, y=12, width=width-2, height=1, text="GPU: " .. (directGpuError or "CC monitor fallback"), foreground=colors.orange})
end
if not directGpu then createMonitorUi() end

local flightPage, calibrationPage
local savedAltitudeButtons = {}
if compact then
  frame:addLabel({x=2, y=1, width=screenWidth-2, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
  status = frame:addLabel({x=2, y=2, width=screenWidth-2, height=1, text="Starting...", foreground=colors.yellow})
  posText = frame:addLabel({x=2, y=3, width=screenWidth-2, height=1, text="GPS --", foreground=colors.white})
  altText = frame:addLabel({x=2, y=4, width=screenWidth-2, height=1, text="ALT -- / TARGET --", foreground=colors.lightBlue})
  navText = frame:addLabel({x=2, y=14, width=screenWidth-3, height=1, text="SPD --  DIST --  ETA --", foreground=colors.lightBlue})
  frame:addLabel({x=2, y=5, width=12, height=1, text="DESTINATION X Z", foreground=colors.lightBlue})
  xInput = frame:addInput({x=2, y=6, width=screenWidth-19, height=1, placeholder="X Z", background=colors.white, foreground=colors.black})
  local function setDestination()
    local xText, zText = xInput.text:match("^%s*(%S+)%s+(%S+)%s*$")
    local x, z = tonumber(xText), tonumber(zText)
    if not x or not z then status.text = "Enter X and Z coordinates"; status.foreground = colors.red; return end
    target = {x=x, z=z}; setFlightMode("HOLDING ALTITUDE"); frame:setFocused(nil)
    status.text = ("Destination %.0f, %.0f set"):format(x, z)
  end
  frame:addButton({x=screenWidth-16, y=6, width=14, height=1, text="SET DEST", background=colors.blue}):onClick(setDestination)
  xInput:onEnter(setDestination)
  frame:addLabel({x=2, y=7, width=12, height=1, text="ALTITUDE TARGET", foreground=colors.lightBlue})
  heightInput = frame:addInput({x=2, y=8, width=8, height=1, placeholder="Y", background=colors.white, foreground=colors.black})
  local function setHeight()
    local value = tonumber(heightInput.text)
    if not value then status.text = "Enter a numeric height"; status.foreground = colors.red; return end
    altitude = value; setFlightMode("HOLDING ALTITUDE"); frame:setFocused(nil)
    status.text = hoverBias(value) and ("Height target Y %.1f uses calibrated burner output."):format(value) or "No saved burner calibration; output remains at its current amount."
    status.foreground = hoverBias(value) and colors.lime or colors.orange
  end
  heightInput:onEnter(setHeight)
  frame:addButton({x=11, y=8, width=10, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
  frame:addButton({x=22, y=8, width=6, height=1, text="ALT -", background=colors.orange}):onClick(function() stepSavedAltitude(-1) end)
  frame:addButton({x=29, y=8, width=6, height=1, text="ALT +", background=colors.orange}):onClick(function() stepSavedAltitude(1) end)
  frame:addButton({x=2, y=9, width=10, height=1, text="HOLD HERE", background=colors.blue}):onClick(holdHere)
  frame:addButton({x=13, y=9, width=10, height=1, text="AUTOPILOT", background=colors.lime}):onClick(engageAutopilot)
  frame:addButton({x=24, y=9, width=8, height=1, text="STOP", background=colors.red}):onClick(stop)
  frame:addButton({x=2, y=10, width=12, height=1, text="TEST FWD", background=colors.gray}):onClick(function() testVector(0, .2, "TEST Y+") end)
  frame:addButton({x=15, y=10, width=12, height=1, text="TEST BACK", background=colors.gray}):onClick(function() testVector(0, -.2, "TEST Y-") end)
  calibrateButton = frame:addButton({x=2, y=11, width=13, height=1, text="DIR CHECK", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
  frame:addButton({x=16, y=11, width=10, height=1, text="SWEEP V", background=colors.orange}):onClick(startBurnerSweep)
  frame:addButton({x=27, y=11, width=12, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
  frame:addButton({x=40, y=11, width=10, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
  pidInput = frame:addInput({x=2, y=12, width=screenWidth-13, height=1, placeholder="Horizontal P I D only", background=colors.white, foreground=colors.black})
  showGainValues = function()
    pidInput.text = ("%.3g %.1g %.2g"):format(table.unpack(gainValues()))
  end
  showGainValues()
  local function setGains()
    if calibrationRunning then status.text="Wait for calibration before changing gains."; return end
    local values = {}
    for token in pidInput.text:gmatch("%S+") do values[#values + 1] = tonumber(token) end
    if #values ~= #gainLimits or not applyGainValues(values) then status.text = "Invalid PID values"; status.foreground = colors.red; return end
    local saved,err=pcall(saveGainFile,values)
    if not saved then status.text="Could not save gains: "..tostring(err); status.foreground=colors.red; return end
    showGainValues(); frame:setFocused(nil); status.text = "PID gains saved"
  end
  pidInput:onEnter(setGains)
  frame:addButton({x=screenWidth-10, y=12, width=8, height=1, text="SET PID", background=colors.blue}):onClick(setGains)
  if directGpuError then compactGpuNotice = frame:addLabel({x=2, y=13, width=screenWidth-3, height=2, text="DirectGPU: " .. directGpuError, foreground=colors.orange}) end
  frame:addButton({x=screenWidth-12, y=15, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
  frame:addButton({x=2, y=15, width=20, height=1, text="FULL AUTO CAL  R", background=colors.blue}):onClick(startFullCalibration)
else
frame:addLabel({x=2, y=1, width=30, height=1, text="AIRSHIP v" .. AIRSHIP_BUILD, foreground=colors.white, background=colors.blue})
status = frame:addLabel({x=2, y=2, width=screenWidth-3, height=1, text="Starting controller...", foreground=colors.yellow})
frame:addButton({x=screenWidth-12, y=1, width=10, height=1, text="UPDATE", background=colors.blue}):onClick(requestProgramUpdate)
local tabs = frame:addTabControl({x=1, y=3, width=screenWidth, height=screenHeight-2})
flightPage = tabs:addTab("FLIGHT")
calibrationPage = tabs:addTab("CALIBRATION")

local function card(parent, x, width, title)
  local box = parent:addFrame({x=x, y=1, width=width, height=4, background=colors.gray})
  box:addLabel({x=2, y=1, width=width-2, height=1, text=title, foreground=colors.lightBlue, background=colors.gray})
  return box
end
local gpsCard = card(flightPage, 2, 16, "GPS POSITION")
local altitudeCard = card(flightPage, 19, 15, "ALTITUDE")
local systemCard = card(flightPage, 35, 15, "FLIGHT SYSTEM")
posText = gpsCard:addLabel({x=2, y=2, width=13, height=3, text="X --\nY --\nZ --", foreground=colors.white, background=colors.gray})
altText = altitudeCard:addLabel({x=2, y=2, width=12, height=2, text="Now --\nTarget --", foreground=colors.white, background=colors.gray})
sysText = systemCard:addLabel({x=2, y=2, width=12, height=2, text="Burner 0/15\nThrust 0/15", foreground=colors.white, background=colors.gray})
navText = flightPage:addLabel({x=2, y=17, width=47, height=1, text="SPEED --  DISTANCE --  ETA --", foreground=colors.lightBlue})

flightPage:addLabel({x=2, y=6, width=22, height=1, text="DESTINATION  (X Z)", foreground=colors.lightBlue})
xInput = flightPage:addInput({x=2, y=7, width=22, height=1, placeholder="Example: 200 -700", background=colors.white, foreground=colors.black})
local function setDestination()
  local xText, zText = xInput.text:match("^%s*(%S+)%s+(%S+)%s*$")
  local x, z = tonumber(xText), tonumber(zText)
  if not x or not z then status.text = "Enter numeric X and Z coordinates first."; status.foreground = colors.red; return end
  target = {x=x, z=z}
  setFlightMode("HOLDING ALTITUDE")
  status.text = ("Destination set: X %.0f, Z %.0f"):format(x, z)
  status.foreground = colors.lime
  frame:setFocused(nil)
end
flightPage:addButton({x=26, y=7, width=12, height=1, text="SET DEST", background=colors.blue}):onClick(setDestination)
xInput:onEnter(setDestination)

flightPage:addLabel({x=2, y=9, width=16, height=1, text="TARGET ALTITUDE Y", foreground=colors.lightBlue})
heightInput = flightPage:addInput({x=2, y=10, width=12, height=1, placeholder="Height", background=colors.white, foreground=colors.black})
local function setHeight()
  local value = tonumber(heightInput.text)
  if not value then status.text = "Enter a numeric height target."; status.foreground = colors.red; return end
  altitude = value
  setFlightMode("HOLDING ALTITUDE")
  status.text = hoverBias(value) and ("Height Y %.0f uses calibrated burner output."):format(value) or "No saved burner calibration; output stays at its current amount."
  status.foreground = hoverBias(value) and colors.lime or colors.orange
  frame:setFocused(nil)
end
flightPage:addButton({x=16, y=10, width=12, height=1, text="SET HEIGHT", background=colors.blue}):onClick(setHeight)
heightInput:onEnter(setHeight)
flightPage:addButton({x=30, y=10, width=13, height=1, text="SAVED ALT", background=colors.blue}):onClick(selectAltitudePreset)
for i=1,4 do
  local index = i
  local x, y = index % 2 == 1 and 2 or 25, index <= 2 and 12 or 13
  savedAltitudeButtons[index] = flightPage:addButton({x=x, y=y, width=22, height=1, text="NO SAVED ALTITUDE", background=colors.gray}):onClick(function() selectSavedAltitude(index) end)
end
local function flightButton(x, width, label, color, action)
  return flightPage:addButton({x=x, y=14, width=width, height=2, text=label, background=color}):onClick(action)
end
flightButton(2, 7, "ALT -", colors.orange, function() stepSavedAltitude(-1) end)
flightButton(10, 7, "ALT +", colors.orange, function() stepSavedAltitude(1) end)
flightButton(18, 9, "HOLD", colors.blue, holdHere)
flightButton(28, 12, "AUTOPILOT", colors.lime, engageAutopilot)
flightButton(41, 9, "STOP", colors.red, stop)

pidSummary = calibrationPage:addLabel({x=2, y=1, width=47, height=1, text="PID  loading gains...", foreground=colors.lightGray})
calibrationPage:addLabel({x=2, y=3, width=47, height=1, text="HORIZONTAL PID: P I D   |   HEIGHT: BURNER AMOUNT", foreground=colors.lightBlue})
pidInput = calibrationPage:addInput({x=2, y=4, width=35, height=1, placeholder="0.035 0.0005 0.18", background=colors.white, foreground=colors.black})
showGainValues = function()
  pidInput.text = ("%.4f %.5f %.2f"):format(table.unpack(gainValues()))
end
showGainValues()
local function setGains()
  if calibrationRunning then status.text="Wait for calibration before changing gains."; return end
  local values = {}
  for token in pidInput.text:gmatch("%S+") do
    local value = tonumber(token)
    if not value then status.text = "Enter horizontal P I D values only."; status.foreground = colors.red; return end
    values[#values + 1] = value
  end
  if #values ~= #gainLimits or not applyGainValues(values) then
    status.text = "Invalid gains. Check ranges in the setup guide."; status.foreground = colors.red; return
  end
  local saved,err=pcall(saveGainFile,values)
  if not saved then status.text="Could not save gains: "..tostring(err); status.foreground=colors.red; return end
  showGainValues()
  frame:setFocused(nil)
  status.text = "PID gains applied and saved."; status.foreground = colors.lime
end
calibrationPage:addButton({x=39, y=4, width=10, height=1, text="SET PID", background=colors.blue}):onClick(setGains)
pidInput:onEnter(setGains)

local function calibrationButton(x, y, width, label, color, action)
  return calibrationPage:addButton({x=x, y=y, width=width, height=2, text=label, background=color}):onClick(action)
end
calibrationButton(2, 6, 22, "Y+ / FWD", colors.gray, function() testVector(0, .2, "TEST Y+") end)
calibrationButton(26, 6, 22, "Y- / BACK", colors.gray, function() testVector(0, -.2, "TEST Y-") end)
calibrateButton = calibrationPage:addButton({x=2, y=9, width=14, height=1, text="DIR CHECK", background=calibrated and colors.green or colors.orange}):onClick(confirmCalibration)
calibrationPage:addButton({x=18, y=9, width=12, height=1, text="MANUAL M", background=colors.orange}):onClick(toggleManualBurner)
calibrationPage:addButton({x=32, y=9, width=15, height=1, text="LOCK LEVEL", background=colors.orange}):onClick(lockHoverLevel)
calibrationButton(2, 12, 16, "SWEEP 15-5", colors.orange, startBurnerSweep)
calibrationPage:addLabel({x=20, y=12, width=13, height=1, text="Sweep saves stable heights", foreground=colors.lightGray})
calibrationPage:addButton({x=35, y=12, width=14, height=1, text="CLEAR HEIGHTS", background=colors.red}):onClick(clearHoverMap)
calibrationPage:addButton({x=2, y=15, width=20, height=1, text="FULL AUTO CAL  R", background=colors.blue}):onClick(startFullCalibration)
calibrationPage:addButton({x=24, y=15, width=20, height=1, text="CANCEL AUTO CAL  H", background=colors.gray}):onClick(holdHere)
end

local function isFocused(input)
  for _, parent in ipairs({frame, flightPage, calibrationPage}) do
    if parent then
      local ok, focused = pcall(function() return parent:getFocused() end)
      if ok and focused == input then return true end
    end
  end
  return false
end
local function refresh()
  if os.clock() - lastThrustRead >= 1 then
    readThrusterPower()
    lastThrustRead = os.clock()
  end
  if compact then
    posText.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    altText.text = ("ALT %.1f / TARGET %.0f"):format(position and position.y or 0, altitude or 0)
  else
    posText.text = position and ("X " .. ("%.1f"):format(position.x) .. "  Y " .. ("%.1f"):format(position.y) .. "\nZ " .. ("%.1f"):format(position.z)) or "GPS not found"
    altText.text = ("Now %s\nTarget %s"):format(position and ("%.1f"):format(position.y) or "--", altitude and ("%.0f"):format(altitude) or "--")
    sysText.text = ("Burner %d/15\nThrust %.1f/15\nHeading %s°"):format(burner, measuredThrust, shipHeading and ("%.0f"):format(math.deg(shipHeading)) or "--")
  end
  local speed,distance,eta=navigationReadout()
  if navText then navText.text=(compact and "SPD %.1f  D %s  ETA %s" or "SPEED %.1f b/s   DISTANCE %s   ETA %s"):format(speed,distance and ("%.1f"):format(distance) or "--",eta) end
  if not isFocused(heightInput) then heightInput.text = altitude and tostring(math.floor(altitude + .5)) or "" end
  if not isFocused(pidInput) then showGainValues() end
  if #savedAltitudeButtons > 0 then
    local heights = savedAltitudes()
    for i, button in ipairs(savedAltitudeButtons) do
      button.text = heights[i] and ("TARGET Y " .. ("%.1f"):format(heights[i])) or "NO SAVED ALTITUDE"
      button.background = heights[i] and colors.blue or colors.gray
    end
  end
  if pidSummary then pidSummary.text = directGpuError and ("GPU: " .. directGpuError) or ("Horizontal PID %.3f / %.4f / %.2f | Height from burner map"):format(pid.horizontalX.p, pid.horizontalX.i, pid.horizontalX.d) end
  if compactGpuNotice then compactGpuNotice.text = directGpuError and ("DirectGPU: " .. directGpuError) or "DirectGPU ready" end
  if monitorGpuNotice then monitorGpuNotice.text = directGpuError and ("GPU: " .. directGpuError) or "GPU ready" end
  if mode == "AUTOPILOT" then status.text, status.foreground = "AUTOPILOT ACTIVE  |  EAST-FACING SHIP", colors.lime
  elseif mode == "GPS LOST - RE-ENGAGE" then status.text, status.foreground = mode, colors.red
  elseif mode == "WAITING FOR GPS" then status.text, status.foreground = "Waiting for GPS signal...", colors.orange
  elseif mode == "HOLDING ALTITUDE" then status.text, status.foreground = "ALTITUDE HOLD ACTIVE", colors.lightBlue
  elseif mode == "DESTINATION REACHED" then status.text, status.foreground = "DESTINATION REACHED  |  HOLDING POSITION + ALTITUDE", colors.lime
  elseif mode == "FINAL TURN" then status.text, status.foreground = "AT DESTINATION  |  ALIGNING TO ROUTE", colors.orange
  elseif mode == "SET ALTITUDE" then status.text, status.foreground = "GPS ready  |  set an altitude target", colors.orange
  elseif mode == "CALIBRATION CONFIRM" then status.text, status.foreground = "Confirm calibration within 8 seconds", colors.orange
  elseif mode == "BURNER SWEEP" and burnerSweep then status.text, status.foreground = ("SWEEP %d/15 | saved %d, skipped %d | press S to stop"):format(burnerSweep.level, burnerSweep.saved, burnerSweep.skipped), colors.orange
  end
  if mode == "MANUAL BURNER" then status.text, status.foreground = ("MANUAL BURNER %d/15 | UP/DOWN adjust, K locks"):format(manualBurner or 0), colors.orange
  elseif mode:match("^TEST ") then status.text, status.foreground = mode .. "  |  press HOLD to stop", colors.orange
  elseif mode == "STOPPED" then status.text, status.foreground = "OUTPUTS STOPPED", colors.red end
  if autopilotNotice and mode~="STOPPED" then status.text,status.foreground=autopilotNotice,autopilotNoticeColor end
  if sweepSummary and os.clock() < sweepSummaryUntil then status.text, status.foreground = sweepSummary, colors.orange
  elseif sweepSummary then sweepSummary = nil end
  if monitorFrame then
    monitorPos.text = position and ("X %.1f Y %.1f Z %.1f"):format(position.x, position.y, position.z) or "GPS not found"
    monitorAlt.text = ("ALT %.1f / T %.0f   D %s   ETA %s"):format(position and position.y or 0, altitude or 0,distance and ("%.0f"):format(distance) or "--",eta)
    local savedCount = 0
    for _ in pairs(learnedHover) do savedCount = savedCount + 1 end
    monitorSystem.text = ("B%d/15 T%.1f SPD %.1f GPS %s"):format(burner, measuredThrust,speed,position and "OK" or "LOST")
    monitorStatus.text, monitorStatus.foreground = status.text, status.foreground
  end
  if directGpu and (gpuRedrawPending or os.clock() - lastGpuRender >= .05) then
    gpuRedrawPending = false
    createGpuButtons()
    local ok, renderError = pcall(renderDirectMonitor)
    if ok then lastGpuRender = os.clock()
    elseif renderError == "DirectGPU frame superseded" then gpuRedrawPending = true
    else
      directGpu = nil; directGpuEnabled = false
      directGpuError = "Draw failed: " .. tostring(renderError)
      createMonitorUi()
    end
  end
end

local lastGpuClick
local function clickGpuButton(x, y, source)
  if not directGpu or type(x) ~= "number" or type(y) ~= "number" then return end
  for _, button in ipairs(gpuButtons) do
    if x >= button.x and x < button.x + button.w and y >= button.y and y < button.y + button.h then
      local key = button.label .. ":" .. button.x .. ":" .. button.y
      local now = os.clock()
      if lastGpuClick and lastGpuClick.key == key and lastGpuClick.source ~= source and now - lastGpuClick.at < .15 then return end
      lastGpuClick = {key=key, source=source, at=now}
      button.action()
      if button.label:match("^SWEEP") then sweepSummary = nil end
      gpuRenderRevision = gpuRenderRevision + 1
      gpuRedrawPending = true
      return
    end
  end
end
local function handleMonitorTouch(name, x, y)
  if not directGpu or not monitor or name ~= monitorName or type(x) ~= "number" or type(y) ~= "number" then return end
  local ok, cols, rows = pcall(monitor.getSize)
  if not ok or type(cols) ~= "number" or type(rows) ~= "number" or cols <= 0 or rows <= 0 then return end
  clickGpuButton((x - .5) * directWidth / cols, (y - .5) * directHeight / rows, "monitor")
end
local function handleGpuMouse(display, kind, x, y)
  if display == directDisplay and kind == "mouse_click" then clickGpuButton(x, y, "gpu_event") end
end
local function pollGpuInput()
  local gpu, display = directGpu, directDisplay
  for _ = 1, 32 do
    if directGpu ~= gpu or directDisplay ~= display then return end
    local pollOk, event = pcall(gpu.pollEvent, display)
    yieldComputer()
    if directGpu ~= gpu or directDisplay ~= display then return end
    if not pollOk then
      directGpu = nil; directGpuEnabled = false
      directGpuError = "Input poll failed: " .. tostring(event)
      createMonitorUi()
      return
    end
    if event == nil then return end
    if type(event) == "table" and event.type == "mouse_click" then
      clickGpuButton(event.x, event.y, "gpu")
    end
  end
end

local ok, err = pcall(function()
  local _, startY = gps.locate(.5, false)
  altitude = startY and math.floor(startY + .5) or nil
  if altitude then mode = "HOLDING ALTITUDE" else mode = "WAITING FOR GPS" end
  basalt.schedule(function()
    yieldComputer()
    while running do
      local success, failure = pcall(function()
        local gpsFresh = locate()
        updateFlight(gpsFresh)
        updateBurnerSweep()
        outputs(true)
      end)
      if not success then
        local sweep = burnerSweep
        stop()
        sweepSummary = "Controller error: " .. tostring(failure) .. (sweep and (" Sweep saved %d/11; skipped %d."):format(sweep.saved, sweep.skipped) or "")
        sweepSummaryUntil = os.clock() + 15
        running = false; status.text = sweepSummary; status.foreground = colors.red; refresh(); basalt.stop(); return
      end
      sleep(.2)
    end
  end)
  basalt.schedule(function()
    yieldComputer()
    while running do
      refresh()
      sleep(gpuRedrawPending and 0 or .05)
    end
  end)
  basalt.schedule(function()
    yieldComputer()
    while running do
      if directGpu and os.clock() - lastGpuCheck >= 5 then
        lastGpuCheck = os.clock()
        local alive, info = pcall(directGpu.getDisplayInfo, directDisplay)
        yieldComputer()
        if not alive or not info then
          directGpu = nil; directGpuEnabled = false
          directGpuError = "Display check failed: " .. tostring(info or "no display info")
          createMonitorUi()
        end
      end
      if directGpuEnabled and not directGpu then
        pcall(setupDirectGpu)
        createGpuButtons()
        if not monitorGpuNotice and monitorFrame then
          monitorGpuNotice = monitorFrame:addLabel({x=2, y=12, width=monitor.getSize()-2, height=1, text="GPU: " .. (directGpuError or "ready"), foreground=colors.orange})
        end
        if not compactGpuNotice and compact then
          compactGpuNotice = frame:addLabel({x=2, y=13, width=screenWidth-3, height=2, text="DirectGPU: " .. (directGpuError or "ready"), foreground=colors.orange})
        end
      end
      if directGpuEnabled and directGpu then
        pollGpuInput()
      end
      sleep(directGpu and .05 or 2)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, name, x, y = os.pullEvent("monitor_touch")
      handleMonitorTouch(name, x, y)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, display, kind, x, y = os.pullEvent("directgpu_mouse")
      handleGpuMouse(display, kind, x, y)
    end
  end)
  basalt.schedule(function()
    while running do
      local _, key = os.pullEvent("key")
      local typing = isFocused(xInput) or isFocused(heightInput) or isFocused(pidInput)
      if typing then
        if key == keys.escape then
          frame:setFocused(nil)
        end
      else
        if key == keys.up then if not adjustManualBurner(1) then adjustAltitude(1) end
        elseif key == keys.down then if not adjustManualBurner(-1) then adjustAltitude(-1) end
        elseif key == keys.left then testVector(-.2, 0, "TEST X-")
        elseif key == keys.right then testVector(.2, 0, "TEST X+")
        elseif key == keys.f then testVector(0, .2, "TEST Y+")
        elseif key == keys.b then testVector(0, -.2, "TEST Y-")
        elseif key == keys.g then xInput:focus()
        elseif key == keys.y then heightInput:focus()
        elseif key == keys.t then pidInput:focus()
        elseif key == keys.h then holdHere()
        elseif key == keys.p then selectAltitudePreset()
        elseif key == keys.m then toggleManualBurner()
        elseif key == keys.k then lockHoverLevel()
        elseif key == keys.v then startBurnerSweep()
        elseif key == keys.u then requestProgramUpdate()
        elseif key == keys.a then engageAutopilot()
        elseif key == keys.s then stop()
        elseif key == keys.c then confirmCalibration()
        elseif key == keys.r then startFullCalibration()
        end
      end
    end
  end)
  basalt.run()
end)
running = false
stop()
if updateRequested then
  local opened = false
  if shell and type(shell.openTab) == "function" then opened = pcall(shell.openTab, "/airship-updater.lua") end
  if not opened then shell.run("/airship-updater.lua") end
end
if not ok then term.setTextColor(colors.red); print("Airship UI stopped: " .. tostring(err)) end
