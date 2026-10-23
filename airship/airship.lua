-- Basalt 2.5 flight panel for the east-facing 24-thruster ship.
local AIRSHIP_BUILD = "1.7.12"
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
    return {t=s.t,x=s.x,y=s.y,z=s.z,vx=s.vx,vz=s.vz,rawVx=s.rawVx,rawVz=s.rawVz,heading=s.heading,yaw=s.yaw,pressure=s.pressure}
  end
  local function observation(a,b)
    local dt=b.t-a.t
    local h=a.heading+wrap(b.heading-a.heading)/2
    local ax,az=body((b.vx-a.vx)/dt,(b.vz-a.vz)/dt,h)
    local vx,vz=body((a.vx+b.vx)/2,(a.vz+b.vz)/2,h)
    return {ax,az,(b.yaw-a.yaw)/dt},{vx,vz,(a.yaw+b.yaw)/2}
  end
  local function window(samples)
    local n,t,tt=0,0,0
    local a,v,duration={0,0,0},{0,0,0},0
    local sum,weighted={0,0,0},{0,0,0}
    local x,z,tx,tz=0,0,0,0
    local first=samples[1]
    for index,s in ipairs(samples) do
      local elapsed=s.t-first.t
      n=n+1; t=t+elapsed; tt=tt+elapsed^2
      x=x+s.x; z=z+s.z; tx=tx+elapsed*s.x; tz=tz+elapsed*s.z
      if index>1 then
        local previous=samples[index-1]
        local dt=s.t-previous.t
        if dt>0 then
          local acceleration,velocity=observation(previous,s)
          for i=1,3 do a[i]=a[i]+acceleration[i]*dt; v[i]=v[i]+velocity[i]*dt end
          duration=duration+dt
        end
      end
      for i=1,3 do sum[i]=sum[i]+a[i]; weighted[i]=weighted[i]+elapsed*a[i] end
    end
    -- Rotate each interval before averaging; a turning ship's world-space forces cancel out.
    local variance=math.max(.0001,tt-t*t/n)
    for i=1,3 do a[i]=(weighted[i]-t*sum[i]/n)/variance; v[i]=v[i]/math.max(.0001,duration) end
    return {a=a,v=v,x=x/n,z=z/n,vx=(tx-t*x/n)/variance,vz=(tz-t*z/n)/variance,yaw=v[3],span=samples[n].t-first.t}
  end
  function A.validSample(s)
    if type(s)~="table" then return false end
    for _,key in ipairs({"t","x","z","vx","vz","heading","yaw","pressure"}) do if not finite(s[key]) then return false end end
    return s.pressure>=0
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
    return (rates[1]*ux+rates[2]*uy)/(1-uz*uz)
  end
  function A.validProfile(p,names)
    if type(p)~="table" or p.version~=1 or p.validated~=true or type(p.names)~="table" or #p.names~=#names or type(p.channels)~="table" or #p.channels~=#names*4 then return false end
    for i,name in ipairs(names) do if p.names[i]~=name then return false end end
    for _,key in ipairs({"pressure","yawWeight","maxAccel","maxSpeed","delay"}) do if not finite(p[key]) or p[key]<=0 then return false end end
    if p.pressure>2 or p.yawWeight>30 or p.maxAccel>6 or p.maxSpeed>32 or p.delay>10 or type(p.drag)~="table" or type(p.gains)~="table" then return false end
    for i=1,3 do
      if not finite(p.drag[i]) or p.drag[i]<0 or p.drag[i]>3 or not finite(p.gains[i]) or p.gains[i]<0 then return false end
    end
    if p.gains[1]<.0001 or p.gains[1]>1 or p.gains[2]>.05 or p.gains[3]>4 then return false end
    if p.maxThrottle~=nil and (not finite(p.maxThrottle) or p.maxThrottle<.1 or p.maxThrottle>1) then return false end
    if p.responseScale~=nil and (not finite(p.responseScale) or p.responseScale<1 or p.responseScale>4) then return false end
    if p.braking~=nil and not A.validBraking(p.braking) then return false end
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
    self.trim={0,0,0}; self.motion=nil; self.applied=nil; self.track={}; self.route=nil; self.finalTurn=nil
  end
  function A:setHeadingTarget(heading) self.headingTarget=heading; self.finalTurn=true end
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
  function A:reverseCommands(s,dx,dz,p,fraction,lateral)
    local strength=p.maxAccel*2*(fraction or 1)
    local bx,bz=body(-dx*strength+(lateral and lateral.x or 0),-dz*strength+(lateral and lateral.z or 0),s.heading)
    local yaw=clamp(.12*wrap((self.headingTarget or s.heading)-s.heading)-2.8*s.yaw,-.12,.12)
    local desired={bx,bz,yaw}
    local commands,residual=self:allocate(p,desired)
    return commands,residual,desired
  end
  function A:stopCommands(s,p,dt,previous)
    local speed=math.sqrt(s.vx^2+s.vz^2)
    local response=math.max(2,dt*5,p.delay*2)
    local lead=math.max(dt,p.delay*.65)
    local vx,vz=s.vx,s.vz
    if previous then
      vx=vx+clamp((s.vx-previous.vx)/dt,-p.maxAccel*2,p.maxAccel*2)*lead
      vz=vz+clamp((s.vz-previous.vz)/dt,-p.maxAccel*2,p.maxAccel*2)*lead
    end
    local bx,bz=body(vx,vz,s.heading)
    -- Dampen motion without chasing an earlier position or heading while thrust decays.
    local desired={speed>.06 and -bx/response or 0,speed>.06 and -bz/response or 0,
      math.abs(s.yaw)>.015 and clamp(-s.yaw/response,-.12,.12) or 0}
    local commands,residual=self:allocate(p,desired)
    self.applied={desired[1]-residual[1],desired[2]-residual[2],desired[3]-residual[3]/p.yawWeight}
    self.trace={desired=desired,braking="CAL_DAMP"}
    return commands
  end
  function A:navigate(s,goal,p)
    p=p or self.profile
    if not p or not A.validSample(s) then return nil,false end
    if p.braking and finite(s.rawVx) and finite(s.rawVz) then s=copy(s); s.vx,s.vz=s.rawVx,s.rawVz end
    local key=goal.x..":"..goal.z
    if self.goalKey~=key then
      self.integral={0,0}; self.goalKey=key
      local dx,dz=goal.x-s.x,goal.z-s.z
      local length=math.sqrt(dx*dx+dz*dz)
      self.route={x=s.x,z=s.z,tx=length>.001 and dx/length or 1,tz=length>.001 and dz/length or 0}
    end
    self.headingTarget=self.headingTarget or s.heading
    local dt=self.previous and clamp(s.t-self.previous.t,.01,2) or .2
    local ratio=clamp(s.pressure/math.max(.001,p.pressure),0,2)
    -- Learn balance during travel; final alignment keeps it instead of learning turning transients.
    self.trim=self.trim or {0,0,0}
    if self.previous and self.applied and not self.finalTurn then
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
    local vx,vz=body(f.vx,f.vz,s.heading)
    local maxAccel=p.maxAccel
    local r=self.route
    local remaining=dx*r.tx+dz*r.tz
    local cross=(f.x-r.x)*(-r.tz)+(f.z-r.z)*r.tx
    local alongSpeed=f.vx*r.tx+f.vz*r.tz
    local crossSpeed=-f.vx*r.tz+f.vz*r.tx
    local response=math.max(1.2,p.delay*.65)*(p.responseScale or 1)
    local modern=p.cruiseVersion==3
    local caution=modern and clamp((p.responseScale or 1)-1,0,1) or 1
    local brakeAccel=maxAccel*(.55-.2*caution)
    local function plannedSpeed(error,limit,margin,delay)
      local gap=math.max(0,math.abs(error)-(margin or .04))
      delay=delay or (modern and p.delay+.65*response+math.max(0,response-math.max(1.2,p.delay*.65)) or p.delay+response)
      local braking=math.sqrt((brakeAccel*delay)^2+2*brakeAccel*gap)-brakeAccel*delay
      return (error<0 and -1 or 1)*math.min(limit,braking,gap*.7/response)
    end
    local cruiseLimit=modern and math.huge or p.maxSpeed
    local targetAlong=plannedSpeed(remaining,cruiseLimit,3+9*caution)
    local rx,rz=body(r.tx,r.tz,s.heading)
    local closing=s.vx*r.tx+s.vz*r.tz
    local travelSign=math.abs(closing)>.2 and (closing<0 and -1 or 1) or (remaining<0 and -1 or 1)
    local measured=A.brakeModel(p,rx*travelSign,rz*travelSign,s.pressure,self.previous and s.t-self.previous.t or dt)
    if measured then
      dx,dz=goal.x-s.x,goal.z-s.z
      distance,speed=math.sqrt(dx*dx+dz*dz),math.sqrt(s.vx^2+s.vz^2)
      vx,vz=body(s.vx,s.vz,s.heading)
      remaining=dx*r.tx+dz*r.tz; alongSpeed=closing
      cross=(s.x-r.x)*(-r.tz)+(s.z-r.z)*r.tx
      crossSpeed=-s.vx*r.tz+s.vz*r.tx
    end
    local stopping=measured and A.stoppingDistance(measured,math.abs(closing)) or 0
    local measuredApproach=measured and (distance>3 or math.abs(closing)>1.5)
    local targetCross=plannedSpeed(-cross,math.max(1,p.maxSpeed*.35))
    local az=clamp((targetCross-crossSpeed)/response,-maxAccel*.7,maxAccel*.7)
    if measuredApproach then
      remaining=(goal.x-s.x)*r.tx+(goal.z-s.z)*r.tz
      local stopLimit=A.stoppingSpeed(measured,math.max(0,math.abs(remaining)-.35))
      local linearLimit=math.abs(remaining)*.6
      targetAlong=(remaining<0 and -1 or 1)*math.min(stopLimit,linearLimit)
      local error=(closing-targetAlong)*(closing<0 and -1 or 1)
      if math.abs(closing)>.1 and error>-.15 then
        local targetSpeed=math.abs(targetAlong)
        local slope=linearLimit<stopLimit and .6 or 1/(measured.delay+targetSpeed/(measured.force+measured.drag*targetSpeed))
        local decel=math.abs(closing)*slope+error/math.max(.5,dt*2)
        local fraction=clamp((decel-measured.drag*math.abs(closing))/measured.fullForce,0,1)
        local direction=closing<0 and -1 or 1
        local commands,residual,desired=self:reverseCommands(s,r.tx*direction,r.tz*direction,p,fraction,{x=-r.tz*az,z=r.tx*az})
        self.applied={desired[1]-residual[1],desired[2]-residual[2],desired[3]-residual[3]/p.yawWeight}
        self.trace={cross=cross,remaining=remaining,targetAlong=targetAlong,targetCross=targetCross,desired=desired,stoppingDistance=stopping,braking="REVERSE"}
        self.previous=copy(s)
        return commands,false
      end
    end
    -- Keep the high-speed stopping margin from creating a dead zone before arrival.
    if modern and not measuredApproach then
      local approach=clamp((20-distance)/14,0,1)
      local approachSpeed=plannedSpeed(remaining,cruiseLimit,.15,p.delay)
      local approachGain=.2-.08*caution
      approachSpeed=clamp(approachSpeed,-math.abs(remaining)*approachGain,math.abs(remaining)*approachGain)
      targetAlong=targetAlong*(1-approach)+approachSpeed*approach
    end
    local braking=modern and alongSpeed*(targetAlong-alongSpeed)<0
    local ax=(targetAlong-alongSpeed)/(braking and distance>20 and response*.5 or response)
    -- Reserve thrust for sideways recovery rather than bending the route toward the current position.
    ax=clamp(ax,-math.sqrt(maxAccel^2-az^2),math.sqrt(maxAccel^2-az^2))
    local bx,bz=body(ax*r.tx-az*r.tz,ax*r.tz+az*r.tx,s.heading)
    local blend=modern and clamp((distance-.15)/(.35+1.5*caution),0,1) or clamp((distance-3-caution)/(6+14*caution),0,1)
    local ex,ez=body(dx,dz,s.heading)
    local bandwidth=math.sqrt(clamp(ratio,.16,1))
    local hold={}
    for i,v in ipairs({{ex,vx},{ez,vz}}) do
      if distance<2 and speed<.25 then self.integral[i]=clamp(self.integral[i]+v[1]*dt,-2,2) else self.integral[i]=0 end
      hold[i]=p.gains[1]*bandwidth^2*v[1]-p.gains[3]*bandwidth*v[2]+p.gains[2]*self.integral[i]
    end
    bx,bz=blend*bx+(1-blend)*hold[1],blend*bz+(1-blend)*hold[2]
    local acceleration={bx+p.drag[1]*ratio*vx,bz+p.drag[2]*ratio*vz}
    local yawDamping=math.max(1.5,2.8-math.min(1.3,p.drag[3]*ratio))
    local turnLimit=self.finalTurn and .24 or .12
    local yaw=clamp((self.finalTurn and .45 or .12)*wrap(self.headingTarget-s.heading)-yawDamping*s.yaw,-turnLimit,turnLimit)
    local desired={acceleration[1]-self.trim[1],acceleration[2]-self.trim[2],yaw-self.trim[3]}
    local commands,residual=self:allocate(p,desired)
    self.applied={desired[1]-(residual and residual[1] or 0),desired[2]-(residual and residual[2] or 0),desired[3]-(residual and residual[3]/p.yawWeight or 0)}
    self.trace={cross=cross,remaining=remaining,targetAlong=targetAlong,targetCross=targetCross,desired=desired,stoppingDistance=stopping,braking=measuredApproach and "TRACK" or "FALLBACK"}
    self.previous=copy(s)
    return commands,distance<.35 and speed<.08 and math.abs(s.yaw)<.03 and math.abs(wrap(self.headingTarget-s.heading))<.03
  end
  function A:begin(s)
    if not A.validSample(s) then return false,"Waiting for fresh GPS, heading, yaw rate and pressure readings." end
    if s.pressure<=.001 then return false,"Air pressure reading is zero; check the Avionics altitude sensor." end
    self:reset(s.heading)
    self.job={phase="BASELINE",at=s.t,previous=copy(s),origin={x=s.x,z=s.z},heading=s.heading,
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
  function A.cruiseSpeed(p)
    -- Estimate validation-leg length; cruise itself is limited by stopping distance and available thrust.
    return math.min(math.sqrt(96*p.maxAccel),p.maxAccel*.9/math.max(.02,p.drag[1],p.drag[2]))
  end
  function A.validBraking(b)
    if type(b)~="table" or b.version~=1 or not finite(b.pressure) or b.pressure<=.001 or b.pressure>2
      or not finite(b.height) or type(b.directions)~="table" or #b.directions~=4 then return false end
    for _,d in ipairs(b.directions) do
      if type(d)~="table" then return false end
      for _,key in ipairs({"force","drag","delay","maxSpeed","sampleDt"}) do if not finite(d[key]) then return false end end
      if d.force<.01 or d.force>200 or d.drag<0 or d.drag>10 or d.delay<.05 or d.delay>10
        or d.maxSpeed<=0 or d.maxSpeed>1000 or d.sampleDt<=0 or d.sampleDt>8 then return false end
    end
    return true
  end
  function A.brakeModel(p,dx,dz,pressure,period)
    local b=p.braking
    if not A.validBraking(b) then return nil end
    local x=b.directions[dx>=0 and 1 or 2]
    local z=b.directions[dz>=0 and 3 or 4]
    local wx,wz=math.abs(dx),math.abs(dz)
    local sum=wx+wz
    if sum<.001 then return nil end
    local delay=math.max(wx>1e-6 and x.delay or 0,wz>1e-6 and z.delay or 0)
    local sampleDt=math.min(wx>1e-6 and x.sampleDt or math.huge,wz>1e-6 and z.sampleDt or math.huge)
    return {force=.9/(wx/x.force+wz/z.force),fullForce=(wx*x.force+wz*z.force)/sum,drag=(wx*x.drag+wz*z.drag)/sum*clamp(pressure/b.pressure,0,3),
      delay=delay+math.max(0,(period or 0)-sampleDt)}
  end
  function A.stoppingDistance(m,speed)
    local u=m.drag*speed/m.force
    local distance
    if u<.001 then distance=speed^2/(2*m.force)*(1-2*u/3+u^2/2)
    else distance=m.force/m.drag^2*(u-math.log(1+u)) end
    return speed*m.delay+distance
  end
  function A.stoppingSpeed(m,distance)
    local lo,hi=0,math.max(0,distance)/m.delay
    for _=1,20 do
      local mid=(lo+hi)/2
      if A.stoppingDistance(m,mid)>distance then hi=mid else lo=mid end
    end
    return lo
  end
  function A:beginBraking(s,candidate)
    local model=candidate or self.profile
    if not A.validCandidate(model,self.names) then return false,"BRAKE CAL needs the completed 96-test thruster survey first." end
    if model.gpsVersion~=2 then return false,"BRAKE CAL needs a GPS-checked thruster survey first." end
    if not A.validSample(s) or not finite(s.y) or s.pressure<=.001 then return false,"BRAKE CAL needs fresh GPS, height, heading, yaw and pressure." end
    local p={}; for k,v in pairs(model) do p[k]=v end; p.braking=nil
    if candidate then p.validated=nil end
    self:reset(s.heading)
    self.job={kind="BRAKES",phase="SETTLE",at=s.t,previous=copy(s),origin={x=s.x,z=s.z},hold={x=s.x,z=s.z},heading=s.heading,
      candidate=p,pending=candidate~=nil,index=1,runs={},pressure=s.pressure,height=s.y,heightMin=s.y,heightMax=s.y}
    return true
  end
  local function fitBraking(j)
    local b={version=1,pressure=j.pressure,height=j.height,heightMin=j.heightMin,heightMax=j.heightMax,directions={}}
    for direction=1,4 do
      local n,x,y,period,cxx,cxy=0,0,0,0,0,0
      for _,i in ipairs({direction,direction+4}) do
        for _,s in ipairs(j.runs[i].coast) do cxx=cxx+s.x^2; cxy=cxy+s.x*s.y end
        for _,s in ipairs(j.runs[i].samples) do
          n=n+1; x=x+s.x; y=y+s.y; period=period+s.dt
        end
      end
      if n<1 then return nil,("No braking samples in direction %d; previous calibration kept."):format(direction) end
      if cxx<.001 then return nil,("Too few coast samples in direction %d; previous calibration kept."):format(direction) end
      local drag=clamp(cxy/cxx,0,10)
      local force=(y-drag*x)/n
      if not finite(force) or force<.01 or force>200 then return nil,"Measured reverse thrust is invalid; previous calibration kept." end
      local d={force=force,drag=drag,delay=.05,maxSpeed=0,sampleDt=period/n,runs={}}
      for _,i in ipairs({direction,direction+4}) do
        local r=j.runs[i]
        local physical=A.stoppingDistance({force=force,drag=drag*r.pressure/j.pressure,delay=0},r.speed)
        d.delay=math.max(d.delay,(r.distance-physical)/r.speed+d.sampleDt*.5)
        d.maxSpeed=math.max(d.maxSpeed,r.speed)
        d.runs[#d.runs+1]={speed=r.speed,distance=r.distance,pressure=r.pressure,height=r.height}
      end
      b.directions[direction]=d
    end
    if not A.validBraking(b) then return nil,"Braking model failed validation; previous calibration kept." end
    return b
  end
  function A:calibrateBraking(s)
    local j=self.job
    if not A.validSample(s) or not finite(s.y) then j.interrupted=true; return {},"BRAKE CAL waiting for fresh navigation and height." end
    if finite(s.rawVx) and finite(s.rawVz) then s=copy(s); s.vx,s.vz=s.rawVx,s.rawVz end
    if s.t<=j.previous.t then return {},"BRAKE CAL waiting for a new sample." end
    if s.t-j.previous.t>8 or j.interrupted then
      j.interrupted=nil; j.phase="SETTLE"; j.at=s.t; j.stable=nil; j.hold={x=s.x,z=s.z}; j.run=nil; self:reset(j.heading)
    end
    local dt=s.t-j.previous.t
    local previous=j.previous
    j.previous=copy(s); j.heightMin=math.min(j.heightMin,s.y); j.heightMax=math.max(j.heightMax,s.y)
    if math.abs(s.yaw)>.8 or s.pressure<=.001 then
      j.interrupted=true
      return {} ,"BRAKE CAL paused: waiting for valid yaw and air pressure."
    end
    local speed=math.sqrt(s.vx^2+s.vz^2)
    if j.phase=="SETTLE" then
      local commands=self:stopCommands(s,j.candidate,dt,previous)
      j.stable=speed<.2 and math.abs(s.yaw)<.03 and (j.stable or s.t) or nil
      if j.stable and s.t-j.stable>=2 then
        if j.index>8 then
          local b,err=fitBraking(j)
          if not b then return {},err end
          j.candidate.braking=b; self.job=nil; return {},nil,j.candidate
        end
        local directions={{1,0},{-1,0},{0,1},{0,-1}}
        local d=directions[(j.index-1)%4+1]
        local c,ss=math.cos(j.heading),math.sin(j.heading)
        j.dx,j.dz=c*d[1]+ss*d[2],-ss*d[1]+c*d[2]
        j.goal={x=s.x+j.dx*10000,z=s.z+j.dz*10000}
        j.targetSpeed=math.max(2,A.cruiseSpeed(j.candidate)*(j.index<=4 and .5 or 1))
        j.phase="ACCEL"; j.at=s.t; j.history={}; self:reset(j.heading)
      elseif s.t-j.at>60 then j.interrupted=true; return {},"BRAKE CAL paused; cannot settle yet. Completed stops kept." end
      return commands,("BRAKE CAL settling | %d/8 recorded | %.2f b/s"):format(j.index-1,speed)
    end
    local along=s.vx*j.dx+s.vz*j.dz
    if j.phase=="ACCEL" or j.phase=="REACCEL" then
      j.history[#j.history+1]={t=s.t,v=along}
      while #j.history>2 and s.t-j.history[2].t>4 do table.remove(j.history,1) end
      local old=j.history[1]
      local plateau=s.t-old.t>=3.5 and along>=old.v*.98 and along<=old.v*1.03 and along>=2
      if along>=j.targetSpeed or plateau or s.t-j.at>=30 then
        if along<2 then j.interrupted=true; return {},"BRAKE CAL paused; could not build measurable speed. Completed stops kept." end
        if j.phase=="ACCEL" then
          j.phase="COAST"; j.at=s.t; j.coast={}; j.coastV=along; j.coastPressure=s.pressure
        else
          j.run={at=s.t,x=s.x,z=s.z,speed=along,previousV=along,previousPressure=s.pressure,distance=0,pressure=s.pressure,height=s.y,samples={},coast=j.coast}
          j.phase="BRAKE"; j.at=s.t; self:reset(j.heading)
        end
      else return self:navigate(s,j.goal,j.candidate),("BRAKE CAL %d/8 accelerate | %.1f / %.1f b/s"):format(j.index,along,j.targetSpeed) end
    end
    if j.phase=="COAST" then
      if s.t-j.at>.8 and along>.5 then
        j.coast[#j.coast+1]={x=(j.coastV+along)/2*(j.coastPressure+s.pressure)/(2*j.pressure),y=(j.coastV-along)/dt}
      end
      j.coastV,j.coastPressure=along,s.pressure
      if s.t-j.at>=3 then j.phase="REACCEL"; j.at=s.t; j.history={} end
      return {},("BRAKE CAL %d/8 measuring drag | %.1f b/s"):format(j.index,along)
    end
    local r=j.run
    r.distance=math.max(r.distance,(s.x-r.x)*j.dx+(s.z-r.z)*j.dz)
    local decel=math.max(0,(r.previousV-along)/dt)
    if j.phase=="BRAKE" and along>.5 and r.previousV>along then
      r.samples[#r.samples+1]={x=(r.previousV+along)/2*(r.previousPressure+s.pressure)/(2*j.pressure),y=(r.previousV-along)/dt,dt=dt}
    end
    r.previousV,r.previousPressure=along,s.pressure
    if j.phase=="BRAKE" and along<=math.max(.5,math.max(decel,j.candidate.maxAccel)*math.max(1.5,dt*3,j.candidate.delay)) then j.phase="TAIL" end
    r.stable=math.abs(along)<=.15 and speed<=.2 and math.abs(s.yaw)<.03 and (r.stable or s.t) or nil
    if r.stable and s.t-r.stable>=2 then
      j.runs[j.index]=r; j.index=j.index+1; j.hold={x=s.x,z=s.z}; j.phase="SETTLE"; j.at=s.t; j.stable=r.stable
      self:reset(j.heading)
      return {},("BRAKE CAL %d/8 recorded | %.1f b/s stopped in %.1f blocks"):format(j.index-1,r.speed,r.distance)
    end
    if s.t-j.at>30 then j.interrupted=true; return {},"BRAKE CAL paused; stop timed out. Completed stops kept." end
    if j.phase=="TAIL" then return self:stopCommands(s,j.candidate,dt,previous),("BRAKE CAL %d/8 finishing stop | %.2f b/s | %.1f blocks"):format(j.index,speed,r.distance) end
    local commands,residual,desired=self:reverseCommands(s,j.dx,j.dz,j.candidate)
    self.applied={desired[1]-residual[1],desired[2]-residual[2],desired[3]-residual[3]/j.candidate.yawWeight}
    self.trace={desired=desired,braking="CAL_REVERSE"}
    return commands,("BRAKE CAL %d/8 full reverse | %.1f b/s | %.1f blocks"):format(j.index,along,r.distance)
  end
  function A:fit(job)
    local p={version=1,gpsVersion=2,cruiseVersion=3,validationVersion=2,responseScale=1,names=self.names,channels=job.channels,pressure=job.pressure/job.samples,drag=drag(job),delay=job.dt/job.samples+1.8,maxThrottle=throttleLimit}
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
    p.maxAccel=math.min(6,math.min(table.unpack(extent))*.55)
    if p.maxAccel<.015 then return nil,"Insufficient measured thrust in all four directions." end
    p.maxSpeed=A.cruiseSpeed(p)
    local frequency=math.min(.18,1/(2*p.delay),math.sqrt(p.maxAccel/3))
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
    local span=math.max(80,p.maxSpeed^2/(p.maxAccel*.35)+p.maxSpeed*(p.delay+math.max(1.2,p.delay*.65))+24)
    return {{x=job.origin.x+span,z=job.origin.z},{x=job.origin.x-span,z=job.origin.z},
      {x=job.origin.x,z=job.origin.z+span},{x=job.origin.x,z=job.origin.z-span},{x=job.origin.x,z=job.origin.z}}
  end
  function A.validCandidate(p,names)
    if type(p)~="table" then return false end
    local checked={}; for k,v in pairs(p) do checked[k]=v end; checked.validated=true
    return A.validProfile(checked,names)
  end
  function A:resumeVerification(s,p)
    if not A.validSample(s) or not A.validCandidate(p,self.names) then return false,"Saved thruster model or navigation sample is invalid." end
    if p.gpsVersion~=2 then return false,"Old survey used unchecked GPS; a fresh thruster survey is required." end
    if s.pressure<=.001 then return false,"Air pressure reading is zero; check the Avionics altitude sensor." end
    if p.cruiseVersion~=3 then
      p.maxSpeed=A.cruiseSpeed(p)
      p.cruiseVersion,p.responseScale=3,1
    end
    if p.validationVersion~=2 then
      p.maxSpeed=A.cruiseSpeed(p); p.responseScale=1; p.validationVersion=2
    end
    p.gains={math.min(p.gains[1],.0324),math.min(p.gains[2],.00065),math.min(1,math.max(p.gains[3],.396))}
    self:reset(s.heading)
    self.job={phase="VERIFY",at=s.t,previous=copy(s),origin={x=s.x,z=s.z},heading=s.heading,
      candidate=p,leg=1,stable=nil,verifyWindow={copy(s)},legStart={x=s.x,z=s.z},overshoot=0,retry=0,persistRevision=0,dt=0,samples=0,pressure=0}
    self.job.goals=travelGoals(self.job)
    return true
  end
  local function retryLeg(controller,j,s,reason,braking,speed)
    if j.retry>=4 then
      controller.job=nil
      return nil,("Auto test %d failed after 5 attempts: %s. Previous calibration kept."):format(j.leg,reason)
    end
    j.retry=j.retry+1; j.totalRetries=(j.totalRetries or 0)+1; j.persistRevision=(j.persistRevision or 0)+1
    if braking then
      j.candidate.responseScale=math.min(4,(j.candidate.responseScale or 1)*1.2)
      if j.candidate.braking then
        for _,d in ipairs(j.candidate.braking.directions) do d.delay=math.min(10,d.delay*1.2+.05) end
      end
    else
      local gains=j.candidate.gains
      if speed>.35 then gains[3]=math.min(4,gains[3]*1.2)
      else gains[1]=clamp(gains[1]*1.25,.0001,.25); gains[2]=gains[1]*.02 end
    end
    j.lastRetry=reason; j.returning=true; j.returnStart={x=s.x,z=s.z}; j.at=s.t; j.stable=nil; j.verifyWindow=nil
    controller:reset(j.heading)
    return {},("AUTO TEST %d/5 retry %d/5: %s; returning to start."):format(j.leg,j.retry+1,reason)
  end
  function A:calibrate(s)
    local j=self.job
    if not j then return nil,"Calibration is not running." end
    if j.kind=="BRAKES" then return self:calibrateBraking(s) end
    if not A.validSample(s) then return {},"Waiting for fresh GPS, heading, yaw rate and pressure readings." end
    if s.t<=j.previous.t then return {},"Waiting for a new navigation sample." end
    if s.t-j.previous.t>8 or j.interrupted then
      j.interrupted=nil
      j.previous=copy(s)
      if j.phase=="VERIFY" then
        j.at=s.t; j.stable=nil; j.verifyWindow={copy(s)}; j.overAt=nil; j.overPeak=nil; self:reset(j.heading); self.trace=nil
        return {},("Navigation gap; restarting auto test %d/5."):format(j.leg)
      end
      while #j.coasts>(j.index-1)*2 do table.remove(j.coasts) end
      j.phase="BASELINE"; j.at=s.t; j.window={copy(s)}; j.before=nil; j.pulse=nil
    return {},("Navigation interrupted; repeating thruster %d/%d baseline."):format(j.index,#self.names*4)
    end
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
    if j.phase=="VERIFY" then
      if math.abs(s.yaw)>.8 then
        return {},("Auto tests paused at yaw %.2f rad/s; thrusters are off, model saved. Press HOLD before resuming."):format(s.yaw)
      end
      local destination=j.goals[j.leg]
      local goal=j.returning and j.legStart or destination
      local gx,gz=destination.x-j.legStart.x,destination.z-j.legStart.z
      local length=math.sqrt(gx*gx+gz*gz)
      local start=j.returning and j.returnStart or j.legStart
      local tx,tz=goal.x-start.x,goal.z-start.z
      local span=math.sqrt(tx*tx+tz*tz)
      j.remaining=span>.01 and ((goal.x-s.x)*tx+(goal.z-s.z)*tz)/span or 0
      j.cross=span>.01 and ((s.z-start.z)*tx-(s.x-start.x)*tz)/span or 0
      if not j.returning and length>.01 then
        local over=((s.x-destination.x)*gx+(s.z-destination.z)*gz)/length
        j.overAt=over>.5 and (j.overAt or s.t) or nil
        j.overPeak=over>.5 and math.max(j.overPeak or 0,over) or nil
        -- Ignore one noisy fix, but preserve sustained or large overshoot for the report.
        if over<=.5 or over>2 or j.overAt and s.t-j.overAt>=.3 then j.overshoot=math.max(j.overshoot,j.overPeak or over) end
      end
      local commands=self:navigate(s,goal,j.candidate)
      self.trace=self.trace or {}
      self.trace.validation={leg=j.leg,remaining=j.remaining,cross=j.cross,overshoot=j.overshoot}
      -- Measure net drift over several rocking cycles; bound each position so real excursions cannot pass.
      j.verifyWindow=j.verifyWindow or {}
      j.verifyWindow[#j.verifyWindow+1]=copy(s)
      while #j.verifyWindow>2 and s.t-j.verifyWindow[1].t>4 do table.remove(j.verifyWindow,1) end
      local f=window(j.verifyWindow)
      j.peakSpeed=math.max(j.peakSpeed or 0,math.sqrt(f.vx^2+f.vz^2))
      local dx,dz=goal.x-f.x,goal.z-f.z
      local settled=math.sqrt(dx*dx+dz*dz)<.35 and math.sqrt(f.vx*f.vx+f.vz*f.vz)<.08 and math.abs(f.yaw)<.03
      local rawDistance=math.sqrt((goal.x-s.x)^2+(goal.z-s.z)^2)
      local bounded=rawDistance<.7 and math.abs(s.yaw)<.1
      j.stable=settled and bounded and (j.stable or s.t) or nil
      if j.stable and s.t-j.stable>=1.5 and f.span>=3 then
        if j.returning then
          j.returning=nil; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.overshoot=0; j.overAt=nil; j.overPeak=nil
          self:reset(j.heading)
          return {},("AUTO TEST %d/5 retry %d/5: %s; starting repeat."):format(j.leg,j.retry+1,j.lastRetry)
        end
        j.maxOvershoot=math.max(j.maxOvershoot or 0,j.overshoot)
        j.leg=j.leg+1; j.at=s.t; j.stable=nil; j.verifyWindow=nil; j.legStart={x=s.x,z=s.z}
        j.retry=0; j.overshoot=0; j.overAt=nil; j.overPeak=nil; j.lastRetry=nil
        if j.leg>#j.goals then
          j.candidate.validated=true
          j.candidate.verification={overshoot=j.maxOvershoot,retries=j.totalRetries or 0,tests=#j.goals,peakSpeed=j.peakSpeed}
          self.job=nil
          return {},nil,j.candidate
        end
      elseif s.t-j.at>math.max(60,6/math.sqrt(j.candidate.gains[1]))+length/math.max(.1,j.candidate.maxSpeed)*2 then
        local speed=math.sqrt(f.vx*f.vx+f.vz*f.vz)
        return retryLeg(self,j,s,("%stimeout %.1fm %.2fm/s yaw %.2f"):format(j.returning and "return " or "",math.sqrt(dx*dx+dz*dz),speed,math.abs(f.yaw)),false,speed)
      end
      return commands,("%s %d/5 try %d/5%s | %.2fm %.2fm/s"):format(j.returning and "RETURN" or "AUTO TEST",math.min(j.leg,5),j.retry+1,j.lastRetry and " | "..j.lastRetry or "",rawDistance,math.sqrt(f.vx^2+f.vz^2))
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

-- BEGIN GPS FIX CORE
local FreshGPS=(function()
  local G={channel=40000}
  local function finite(v) return type(v)=="number" and v==v and math.abs(v)<math.huge end
  local function solveMatrix(matrix,tolerance)
    for col=1,3 do
      local pivot=col
      for row=col+1,3 do if math.abs(matrix[row][col])>math.abs(matrix[pivot][col]) then pivot=row end end
      matrix[col],matrix[pivot]=matrix[pivot],matrix[col]
      local divisor=matrix[col][col]
      if math.abs(divisor)<tolerance then return nil end
      for j=col,4 do matrix[col][j]=matrix[col][j]/divisor end
      for row=1,3 do if row~=col then
        local factor=matrix[row][col]
        for j=col,4 do matrix[row][j]=matrix[row][j]-factor*matrix[col][j] end
      end end
    end
    return {matrix[1][4],matrix[2][4],matrix[3][4]}
  end
  local function rangeError(p,fixes)
    local cost,worst=0,0
    for _,f in ipairs(fixes) do
      local error=math.sqrt((p[1]-f.x)^2+(p[2]-f.y)^2+(p[3]-f.z)^2)-f.distance
      cost=cost+error^2; worst=math.max(worst,math.abs(error))
    end
    return cost,worst
  end
  function G.solve(fixes)
    if #fixes<4 then return nil,"Need four fresh GPS hosts" end
    local first=fixes[1]
    local matrix={{0,0,0,0},{0,0,0,0},{0,0,0,0}}
    for i=2,#fixes do
      local f=fixes[i]
      local row={f.x-first.x,f.y-first.y,f.z-first.z}
      local length=math.sqrt(row[1]^2+row[2]^2+row[3]^2)
      if length<.01 then return nil,"Duplicate GPS host positions" end
      local value=(length^2+first.distance^2-f.distance^2)/(2*length)
      for j=1,3 do row[j]=row[j]/length end
      for j=1,3 do
        for k=1,3 do matrix[j][k]=matrix[j][k]+row[j]*row[k] end
        matrix[j][4]=matrix[j][4]+row[j]*value
      end
    end
    local p=solveMatrix(matrix,1e-6)
    if not p then return nil,"GPS host geometry is flat or too narrow" end
    local localFixes={}
    for _,f in ipairs(fixes) do localFixes[#localFixes+1]={x=f.x-first.x,y=f.y-first.y,z=f.z-first.z,distance=f.distance} end
    local cost,worst=rangeError(p,localFixes)
    -- Squared-range equations amplify small range errors; refine against the measured distances.
    for _=1,8 do
      if cost<1e-12 then break end
      matrix={{0,0,0,0},{0,0,0,0},{0,0,0,0}}
      for _,f in ipairs(localFixes) do
        local row={p[1]-f.x,p[2]-f.y,p[3]-f.z}
        local distance=math.sqrt(row[1]^2+row[2]^2+row[3]^2)
        if distance>1e-9 then
          for j=1,3 do row[j]=row[j]/distance end
          for j=1,3 do
            for k=1,3 do matrix[j][k]=matrix[j][k]+row[j]*row[k] end
            matrix[j][4]=matrix[j][4]-row[j]*(distance-f.distance)
          end
        end
      end
      local step=solveMatrix(matrix,1e-12)
      if not step then break end
      local improved=false
      for attempt=0,5 do
        local scale=2^-attempt
        local nextP={p[1]+step[1]*scale,p[2]+step[2]*scale,p[3]+step[3]*scale}
        local nextCost,nextWorst=rangeError(nextP,localFixes)
        if finite(nextCost) and nextCost<cost then
          p,cost,worst=nextP,nextCost,nextWorst; improved=true; break
        end
      end
      if not improved then break end
    end
    G.residual=worst
    if not finite(cost) or worst>.75 then return nil,"GPS host ranges disagree" end
    return {x=first.x+p[1],y=first.y+p[2],z=first.z+p[3]}
  end
  function G.accept(state,fix,accel)
    if not finite(fix.x) or not finite(fix.y) or not finite(fix.z) or not finite(fix.t) then return false,"Invalid GPS coordinates" end
    local last=state.last
    if last then
      local dt=fix.t-last.t
      if dt<=0 then return false,"Repeated GPS timestamp" end
      local dx=fix.x-last.x-(state.vx or 0)*dt
      local dz=fix.z-last.z-(state.vz or 0)*dt
      -- A velocity from two fixes includes their position error and acceleration during that interval.
      local uncertainty=state.dt and (1.5/state.dt+.5*accel*state.dt) or 40
      local allowance=1.5+uncertainty*dt+.5*accel*dt^2
      G.check={dt=dt,previous_dt=state.dt,dx=dx,dz=dz,vx=state.vx,vz=state.vz,allowance=allowance}
      if math.sqrt(dx^2+dz^2)>allowance then return false,"GPS jump rejected" end
      local vx,vz=(fix.x-last.x)/dt,(fix.z-last.z)/dt
      state.vx,state.vz,state.dt=vx,vz,dt
    end
    state.last=fix
    return true
  end
  function G.locate(timeout)
    G.fixes,G.check,G.residual=nil,nil,nil
    local side
    for _,name in ipairs(rs.getSides()) do
      if peripheral.getType(name)=="modem" and peripheral.call(name,"isWireless") then side=name; break end
    end
    if not side then G.error="No attached wireless modem"; return nil end
    local modem=peripheral.wrap(side)
    local reply
    for _=1,20000 do
      G.channel=40000+(G.channel-39999)%20000
      if not modem.isOpen(G.channel) then reply=G.channel; break end
    end
    if not reply then G.error="No free GPS reply channel"; return nil end
    modem.open(reply)
    local timer=os.startTimer(timeout or 2)
    local ok,result,reason=pcall(function()
      modem.transmit(65534,reply,"PING")
      local fixes,seen={},{}
      G.fixes=fixes
      local failure="Need four fresh GPS hosts"
      while true do
        local event,from,channel,sender,message,distance=os.pullEvent()
        if event=="timer" and from==timer then return nil,failure end
        if event=="modem_message" and from==side and channel==reply and sender==65534 and finite(distance) and distance>=0
          and type(message)=="table" and finite(message[1]) and finite(message[2]) and finite(message[3]) then
          local key=message[1]..":"..message[2]..":"..message[3]
          if not seen[key] then
            seen[key]=true; fixes[#fixes+1]={x=message[1],y=message[2],z=message[3],distance=distance}
            local position,err=G.solve(fixes)
            if position then return position end
            failure=err
          end
        end
      end
    end)
    modem.close(reply); os.cancelTimer(timer)
    if not ok then G.error=tostring(result); return nil end
    G.error=reason
    if result then return result.x,result.y,result.z end
  end
  function G.report()
    local report={error=G.error,check=G.check,residual=G.residual,hosts={}}
    local first=G.fixes and G.fixes[1]
    for _,f in ipairs(G.fixes or {}) do
      report.hosts[#report.hosts+1]={x=f.x-first.x,y=f.y-first.y,z=f.z-first.z,distance=f.distance}
    end
    return report
  end
  return G
end)()
-- END GPS FIX CORE

print("Discovering peripherals...")
local monitor = peripheral.find("monitor")
local directGpu, directDisplay, directWidth, directHeight, directGpuError
local directGpuEnabled = (...) ~= "--no-gpu"
local findAvionics
local gpuSetupStage
local function gpuSetupProgress(stage)
  gpuSetupStage = stage
  print("GPU: " .. stage)
  local file = fs.open("airship-gpu.log", "a")
  if file then file.writeLine(stage); file.close() end
end
local function configureDirectGpu()
  gpuSetupProgress("find peripheral")
  local gpuFound = peripheral.wrap("directgpu_5") or peripheral.find("directgpu")
  yieldComputer()
  if not gpuFound then directGpuError = "DirectGPU peripheral not found"; return false end
  gpuSetupProgress("listDisplays")
  local listOk, displays = pcall(gpuFound.listDisplays)
  yieldComputer()
  local display = listOk and type(displays) == "table" and displays[1]
  local function getInfo(id)
    if id == nil then return nil, "no display ID returned" end
    gpuSetupProgress("getDisplayInfo " .. tostring(id))
    local ok, info = pcall(gpuFound.getDisplayInfo, id)
    yieldComputer()
    if not ok then return nil, tostring(info) end
    if type(info) ~= "table" or type(info.pixelWidth) ~= "number" or type(info.pixelHeight) ~= "number"
      or not (info.pixelWidth > 0 and info.pixelWidth < math.huge and info.pixelHeight > 0 and info.pixelHeight < math.huge) then
      return nil, "display info has no pixel dimensions"
    end
    return info
  end
  local info, infoError = getInfo(display)
  local setupError
  if not info then
    local nameOk, name = pcall(peripheral.getName, monitor)
    yieldComputer()
    if not nameOk or type(name) ~= "string" or name == "" then
      directGpuError = "Monitor name unavailable; expose the CC monitor on the wired network"
      return false
    end
    -- An unnamed create scans a 32-block radius inside Java and cannot be timed out by Lua.
    gpuSetupProgress("create display " .. name .. " at resolution 1")
    local created, newDisplay = pcall(gpuFound.autoDetectAndCreateDisplayWithResolution, name, 1)
    yieldComputer()
    if created then display = newDisplay; info, infoError = getInfo(display)
    else setupError = "create display: " .. tostring(newDisplay) end
  end
  if info then
    gpuSetupProgress("enableMouseEvents")
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
    gpuSetupProgress("ready")
    return true
  else
    directGpuError = "Display setup: " .. tostring(setupError or infoError)
    directGpu = nil
    return false
  end
end
local function setupDirectGpu()
  local ready, failure
  local file = fs.open("airship-gpu.log", "w")
  if file then file.close() end
  -- pcall catches errors; parallel also bounds peripheral calls that wait for events.
  local finished = parallel.waitForAny(function()
    local ok, result = pcall(configureDirectGpu)
    ready = ok and result
    if not ok then failure = tostring(result) end
  end, function() sleep(8) end)
  if finished ~= 1 then failure = "Timed out at " .. tostring(gpuSetupStage) end
  if not ready then
    directGpu, directDisplay = nil, nil
    directGpuEnabled = false
    directGpuError = failure or directGpuError or "Display setup failed"
    gpuSetupProgress(directGpuError .. "; using CC monitor")
  end
  return ready or false
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
if thrusterCount ~= 24 then
  error("Expected 24 liquid_vector_thruster peripherals, found " .. thrusterCount .. "; check the wired hub")
end
print("Discovering avionics...")
local altitudeSensor = findAvionics("altitude_sensor")
local navigationTable = findAvionics("navigation_table")
local gimbalSensor = findAvionics("gimbal_sensor")
print("Setting up display...")
if directGpuEnabled then setupDirectGpu() else directGpuError = "DirectGPU paused for stability; using CC monitor" end
print("Building controls...")
local frame = basalt.getMainFrame()
local monitorFrame
frame.background = colors.black
if monitorFrame then monitorFrame.background = colors.black end
local screenWidth, screenHeight
screenWidth, screenHeight = term.getSize()
local compact = screenWidth < 49 or screenHeight < 19
local position, target, altitude, mode = nil, nil, nil, "STARTING"
local function nearestCardinalHeading(heading)
  local quarterTurn = math.pi / 2
  return (math.floor(heading / quarterTurn + .5) % 4) * quarterTurn
end
local burner = redstone.getAnalogOutput("front") or 0
local burnerCommand, vectorX, vectorY, power = burner, 0, 0, 0
local burnerSlewPerSecond = 6
local calibrated = fs.exists("airship-calibrated")
local running, manualBurner, burnerSweep, calibrationRunning = true, nil, nil, false
local navNames={}; for name in pairs(thrusters) do navNames[#navNames+1]=name end; table.sort(navNames)
local navController=Autopilot.new(navNames)
local autopilotCommands, fullCalibration, autopilotNotice, brakeCalibrationActive
local autopilotNoticeColor=colors.orange
local flightRecording
local flightLogDirectory="flight logs"
local maxFlightSamples=1200
local function newFlightRecording(s)
  local r={t=s.t,at=s.t-.5,count=0}
  local ok,path=pcall(function()
    fs.makeDir(flightLogDirectory)
    local base=flightLogDirectory.."/flight-"..os.date("%Y-%m-%d_%H-%M-%S")
    local candidate=base..".csv"
    local suffix=1
    while fs.exists(candidate) do
      suffix=suffix+1
      candidate=base..("-%02d.csv"):format(suffix)
    end
    return candidate
  end)
  if ok then r.path=path else r.failed=tostring(path) end
  return r
end
local function recordFlight(s,goal,profile,commands,stage)
  if not Autopilot.validSample(s) then return end
  local r=flightRecording or newFlightRecording(s)
  flightRecording=r
  if r.failed or s.t-r.at<.5 then return end
  if r.count>=maxFlightSamples then r.truncated=true; return end
  r.x,r.z,r.stage=r.x or s.x,r.z or s.z,r.stage or stage
  local file
  local ok,err=pcall(function()
    file=assert(fs.open(r.path,r.count==0 and "w" or "a"),"Cannot open flight report")
    if r.count==0 then
      file.writeLine("# Airship v"..AIRSHIP_BUILD.."; started "..os.date("%Y-%m-%d %H:%M:%S").."; relative X/Z only; max 1200 samples at no more than 2 Hz")
      file.writeLine("# profile "..textutils.serialize(profile,{compact=true}))
      local columns="time,stage,x,z,goal_x,goal_z,vx,vz,heading,yaw,pressure,cross,remaining,target_along,target_cross,requested_ax,requested_az,requested_yaw,allocated_ax,allocated_az,allocated_yaw,stop_distance,braking,raw_vx,raw_vz,test_remaining,test_cross,overshoot"
      for _,name in ipairs(navNames) do columns=columns..","..name.."_x,"..name.."_y,"..name.."_power" end
      file.writeLine(columns)
    end
    local trace=navController.trace or {}
    local validation=trace.validation or {}
    local desired,applied=trace.desired or {},navController.applied or {}
    local values={s.t-r.t,stage,s.x-r.x,s.z-r.z,goal.x-r.x,goal.z-r.z,s.vx,s.vz,s.heading,s.yaw,s.pressure,
      trace.cross or 0,trace.remaining or 0,trace.targetAlong or 0,trace.targetCross or 0,
      desired[1] or 0,desired[2] or 0,desired[3] or 0,applied[1] or 0,applied[2] or 0,applied[3] or 0,trace.stoppingDistance or 0,trace.braking or "FALLBACK",s.rawVx or s.vx,s.rawVz or s.vz,validation.remaining or 0,validation.cross or 0,validation.overshoot or 0}
    for _,name in ipairs(navNames) do
      local command=commands and commands[name] or {}
      values[#values+1]=command.x or 0; values[#values+1]=command.y or 0; values[#values+1]=command.power or 0
    end
    for i,value in ipairs(values) do values[i]=type(value)=="number" and ("%.5g"):format(value) or tostring(value) end
    file.writeLine(table.concat(values,","))
  end)
  if file then pcall(file.close) end
  if not ok then r.failed=tostring(err) else r.at=s.t; r.count=r.count+1 end
end
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
local pendingAutopilotPath="airship-autopilot-pending.cfg"
local brakeCheckpointPath="airship-brake-checkpoint.cfg"
local function savePendingAutopilot(profile)
  assert(Autopilot.validCandidate(profile,navNames),"Invalid pending thruster model")
  local file=assert(fs.open(pendingAutopilotPath..".tmp","w"),"Cannot save pending thruster model")
  file.write(textutils.serialize(profile)); file.close()
  local backup=pendingAutopilotPath..".bak"
  if fs.exists(backup) then fs.delete(backup) end
  local previous=fs.exists(pendingAutopilotPath)
  if previous then fs.move(pendingAutopilotPath,backup) end
  local ok,err=pcall(fs.move,pendingAutopilotPath..".tmp",pendingAutopilotPath)
  if not ok then if previous then fs.move(backup,pendingAutopilotPath) end; error(err) end
end
local function clearPendingAutopilot()
  if fs.exists(pendingAutopilotPath) then fs.delete(pendingAutopilotPath) end
  if fs.exists(pendingAutopilotPath..".bak") then fs.delete(pendingAutopilotPath..".bak") end
  if fs.exists(pendingAutopilotPath..".tmp") then fs.delete(pendingAutopilotPath..".tmp") end
end
local function clearBrakeCheckpoint()
  for _,path in ipairs({brakeCheckpointPath,brakeCheckpointPath..".bak",brakeCheckpointPath..".tmp"}) do
    if fs.exists(path) then fs.delete(path) end
  end
end
local function saveBrakeCheckpoint(job)
  local checkpoint={version=2,names=job.candidate.names,channels=job.candidate.channels,candidate=job.candidate,pending=job.pending,
    index=job.index,runs=job.runs,pressure=job.pressure,height=job.height,heightMin=job.heightMin,heightMax=job.heightMax,heading=job.heading}
  local path=brakeCheckpointPath
  local file=assert(fs.open(path..".tmp","w"),"Cannot checkpoint brake calibration")
  file.write(textutils.serialize(checkpoint)); file.close()
  if fs.exists(path..".bak") then fs.delete(path..".bak") end
  local previous=fs.exists(path)
  if previous then fs.move(path,path..".bak") end
  local ok,err=pcall(fs.move,path..".tmp",path)
  if not ok then if previous then fs.move(path..".bak",path) end; error(err) end
end
local function loadBrakeCheckpoint(model,isPending)
  local function number(value) return type(value)=="number" and value==value and math.abs(value)<math.huge end
  local function sameSurvey(saved,current)
    if type(current)~="table" or #saved.names~=#current.names or #saved.channels~=#current.channels then return false end
    for _,key in ipairs({"version","gpsVersion","pressure","yawWeight","maxAccel","maxSpeed","delay","maxThrottle","responseScale","cruiseVersion","validationVersion"}) do
      if saved.candidate[key]~=current[key] then return false end
    end
    for i,name in ipairs(current.names) do if saved.names[i]~=name then return false end end
    for i,channel in ipairs(current.channels) do
      local other=saved.channels[i]
      if type(other)~="table" or channel.name~=other.name or channel.x~=other.x or channel.y~=other.y or type(channel.b)~="table" or type(other.b)~="table" then return false end
      for k=1,3 do if channel.b[k]~=other.b[k] then return false end end
    end
    for _,key in ipairs({"drag","gains"}) do
      local a,b=saved.candidate[key],current[key]
      if type(a)~="table" or type(b)~="table" or #a~=#b then return false end
      for i=1,#a do if a[i]~=b[i] then return false end end
    end
    return true
  end
  for _,path in ipairs({brakeCheckpointPath,brakeCheckpointPath..".bak"}) do
    if fs.exists(path) then
      local ok,value=pcall(function()
        local file=assert(fs.open(path,"r")); local data=textutils.unserialize(file.readAll()); file.close(); return data
      end)
      if ok and type(value)=="table" and type(model)=="table" and value.version==2 and value.pending==isPending
        and number(value.index) and value.index==math.floor(value.index)
        and value.index>=2 and value.index<=9 and type(value.runs)=="table" and type(value.names)=="table" and type(value.channels)=="table"
        and Autopilot.validCandidate(value.candidate,navNames) and value.candidate.gpsVersion==2
        and sameSurvey(value,model)
        and number(value.pressure) and value.pressure>.001 and number(value.height) and number(value.heightMin) and number(value.heightMax)
        and number(value.heading) then
        local valid=true
        for i=1,value.index-1 do
          local run=value.runs[i]
          if type(run)~="table" or not number(run.speed) or run.speed<=0 or not number(run.distance) or run.distance<0
            or not number(run.pressure) or run.pressure<=.001 or not number(run.height) or type(run.samples)~="table" or type(run.coast)~="table" or #run.coast==0 then valid=false; break end
          for _,sample in ipairs(run.samples) do
            if type(sample)~="table" or not number(sample.x) or sample.x<=0 or not number(sample.y) or sample.y<0 or not number(sample.dt) or sample.dt<=0 then valid=false; break end
          end
          for _,sample in ipairs(run.coast) do
            if type(sample)~="table" or not number(sample.x) or not number(sample.y) then valid=false; break end
          end
          if not valid then break end
        end
        if valid then return value end
      end
    end
  end
end
local pendingAutopilot,pendingRetry
if fs.exists(pendingAutopilotPath) then
  local ok,profile=pcall(function()
    local file=assert(fs.open(pendingAutopilotPath,"r")); local value=textutils.unserialize(file.readAll()); file.close(); return value
  end)
  if ok and Autopilot.validCandidate(profile,navNames) then
    if profile.gpsVersion==2 then pendingAutopilot=profile
    else autopilotNotice="Old survey used unchecked GPS; FULL AUTO CAL must measure again." end
  else clearPendingAutopilot() end
end
if fs.exists("airship-autopilot.cfg") then
  local ok,profile=pcall(function()
    local file=assert(fs.open("airship-autopilot.cfg","r")); local value=textutils.unserialize(file.readAll()); file.close(); return value
  end)
  if ok and Autopilot.validProfile(profile,navNames) and profile.gpsVersion==2 then
    navController.profile=profile
    if not pendingAutopilot and profile.cruiseVersion~=3 then
      pendingAutopilot=textutils.unserialize(textutils.serialize(profile))
      pendingAutopilot.validated=nil
    end
  else autopilotNotice="Saved autopilot needs a fresh GPS-checked survey; run FULL AUTO CAL." end
end
local sweepConfirmAt
local sweepSummary
local sweepSummaryUntil
local gpuButtons = {}
local gpuVisibleButtons = {}
local gpuTab, gpuHeightPage = "FLIGHT", 1
local gpuRenderRevision = 0
local learnedHover = {}
local altitudeSensorPressure
local shipHeading
local yawRate
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
if pendingAutopilot then
  autopilotNotice="96 thruster tests saved; FULL AUTO CAL will resume with auto tests only."
  autopilotNoticeColor=colors.orange
end
local velocity = {x=0, y=0, z=0}
local intervalVelocity = {x=0,z=0}
local previousPosition, previousSample
local gpsFilter={}
local sampleDt = .5
local altitudeSensorHeight
local altitudeSensorReady = false
local lastGpsAt
local pressureAt, headingAt, yawAt
local function navigationSample()
  local now=os.clock()
  if not position or not lastGpsAt or now-lastGpsAt>2 or not pressureAt or now-pressureAt>2 or not headingAt or now-headingAt>2 or not yawAt or now-yawAt>2 then return nil end
  return {t=now,x=position.x,y=position.y,z=position.z,vx=velocity.x,vz=velocity.z,rawVx=intervalVelocity.x,rawVz=intervalVelocity.z,heading=shipHeading,yaw=yawRate,pressure=altitudeSensorPressure}
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

local function calibrationReadout()
  local v=fullCalibration and navController.job and navController.job.phase=="VERIFY" and navController.trace and navController.trace.validation
  return v and ("LEFT %+.1fm  SIDE %.1fm  OVER %.2fm"):format(v.remaining,math.abs(v.cross),v.overshoot) or nil
end
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function outputs(cooperative)
  local jobs={}
  for name, thruster in pairs(thrusters) do
    local device,id=thruster,name
    jobs[#jobs+1]=function()
      local command=autopilotCommands and autopilotCommands[id]
      local x,y=vectorX,vectorY
      if autopilotCommands then x,y=command and command.x or 0,command and command.y or 0 end
      device.setVector(x, y)
      if cooperative then yieldComputer() end
      command=autopilotCommands and autopilotCommands[id]
      local thrust=autopilotCommands and (command and command.power or 0) or power
      device.setThrustNormalized(thrust)
      if cooperative then yieldComputer() end
    end
  end
  redstone.setAnalogOutput("front", burner)
  -- These methods wait for server ticks; dispatch corners together instead of 48 serial waits.
  if #jobs>0 then parallel.waitForAll(table.unpack(jobs)) end
end
local function saveFlightReport()
  if fullCalibration or mode=="AUTOPILOT" or mode=="FINAL TURN" then status.text="Press H to hold before saving the flight log."; return end
  local r=flightRecording
  if not r or not r.path or r.count==0 then status.text="No flight log has been recorded yet."; return end
  if r.failed then status.text="Flight log stopped after "..r.count.." samples: "..r.failed; return end
  if r.saved then status.text=(r.truncated and "Log reached 1,200-sample limit: " or "Flight log saved: ")..r.path; return end
  local file
  local ok,err=pcall(function()
    file=assert(fs.open(r.path,"a"),"Cannot open flight log")
    file.writeLine(("# F8 saved; samples=%d; duration=%.1fs; truncated=%s"):format(r.count,math.max(0,r.at-r.t),tostring(r.truncated==true)))
    file.close(); file=nil
  end)
  if file then pcall(file.close) end
  if not ok then status.text="Flight log is at "..r.path.." but could not be finalized: "..tostring(err); return end
  r.saved=true
  status.text=(r.truncated and "Log reached 1,200-sample limit; saved " or "Flight log saved: ")..r.path
end
local function stop()
  calibrationRunning = false
  autopilotNotice=nil
  navController:cancel(); autopilotCommands,fullCalibration=nil,nil
  brakeCalibrationActive=false
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
  brakeCalibrationActive = false
  autopilotNotice=nil
  navController:cancel(); autopilotCommands,fullCalibration=nil,nil
  manualBurner = nil
  burnerSweep = nil
  vectorX, vectorY, power, mode = 0, 0, 0, nextMode
  pid.horizontalX.integral, pid.horizontalZ.integral = 0, 0
end
local gpsReportAt
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
    if ok then headingRate=Autopilot.headingRate(rates,gravityOk and gravity or nil) end
    if headingRate then yawRate=headingRate; yawAt=os.clock() end
  end
  local x, y, z = FreshGPS.locate(2)
  now=os.clock()
  if x then
    if shipHeading then x,z=x+2*math.sin(shipHeading),z+2*math.cos(shipHeading) end
    local profile=navController.job and navController.job.candidate or navController.profile
    local accepted,reason=FreshGPS.accept(gpsFilter,{t=now,x=x,y=y,z=z},math.max(12,profile and profile.maxAccel*2 or 12))
    if not accepted then x=nil; FreshGPS.error=reason end
  end
  if not x and FreshGPS.error and (not gpsReportAt or now-gpsReportAt>=1) then
    gpsReportAt=now
    local file
    pcall(function()
      file=assert(fs.open("airship-gps.log","w"))
      file.write("# Airship v"..AIRSHIP_BUILD.." GPS failure; host positions relative to first host\n"..textutils.serialize(FreshGPS.report()))
    end)
    if file then pcall(file.close) end
  end
  if x then
    local centerX,centerZ=x,z
    if previousPosition and previousSample then
      sampleDt = math.max(.01, now - previousSample)
      intervalVelocity.x,intervalVelocity.z=(centerX-previousPosition.x)/sampleDt,(centerZ-previousPosition.z)/sampleDt
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
  elseif lastGpsAt and os.clock() - lastGpsAt > 6 then
    position = nil
    previousPosition, previousSample = nil, nil
    velocity = {x=0, y=0, z=0}
    intervalVelocity = {x=0,z=0}
    lastGpsAt = nil
    gpsFilter={}
    if mode ~= "STOPPED" then
      vectorX, vectorY, power = 0, 0, 0
      if burnerSweep then
        sweepSummary = ("Sweep aborted: GPS lost. Saved %d/11; skipped %d."):format(burnerSweep.saved, burnerSweep.skipped)
        sweepSummaryUntil = os.clock() + 15
      end
      burnerSweep, manualBurner = nil, burner
      local brakeJob=fullCalibration and navController.job and navController.job.kind=="BRAKES" and navController.job
      if brakeJob then
        brakeJob.interrupted=true; autopilotCommands={}
        autopilotNotice=("GPS paused; %d/8 brake stops kept. Waiting for a valid fix."):format(brakeJob.index-1)
        autopilotNoticeColor=colors.orange
      else
        if fullCalibration then autopilotNotice="GPS invalid: "..tostring(FreshGPS.error).."; calibration stopped, burner held."; autopilotNoticeColor=colors.red
        else autopilotNotice="GPS invalid: "..tostring(FreshGPS.error or "no fix").."; re-engage AUTO."; autopilotNoticeColor=colors.orange end
        navController:cancel(); autopilotCommands,fullCalibration=nil,nil; calibrationRunning=false
      end
      mode = mode == "AUTOPILOT" and "GPS LOST - RE-ENGAGE" or "WAITING FOR GPS"
    end
  elseif FreshGPS.error and (fullCalibration or mode=="AUTOPILOT" or not position) then
    local brakeJob=fullCalibration and navController.job and navController.job.kind=="BRAKES" and navController.job
    autopilotNotice=brakeJob and ("GPS paused; %d/8 brake stops kept: "):format(brakeJob.index-1)..FreshGPS.error or "GPS paused: "..FreshGPS.error
    autopilotNoticeColor=colors.orange
  end
  return false
end
local function updateFlight(gpsFresh)
  if not position then
    vectorX, vectorY, power = 0, 0, 0
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
    if not gpsFresh then
      autopilotCommands={}
      if navController.job and navController.job.phase~="VERIFY" then navController.job.interrupted=true end
      return
    end
    local sample=navigationSample()
    if not navController.job then
      if brakeCalibrationActive then
        fullCalibration,calibrationRunning=false,false
        brakeCalibrationActive=false
        mode="HOLDING ALTITUDE"
        autopilotNotice="BRAKE CAL paused without a live checkpoint; saved profile kept. Start BRAKE CAL to resume measurement."
        autopilotNoticeColor=colors.orange
        return
      end
      local started,message=navController:begin(sample)
      autopilotCommands={}
      autopilotNotice=started and "Navigation ready; starting calibration." or message
      autopilotNoticeColor=colors.orange
      return
    end
    local brakingCalibration=navController.job.kind=="BRAKES"
    if brakingCalibration then mode="BRAKE CAL" end
    local brakingPending=brakingCalibration and navController.job.pending
    local brakeIndex=brakingCalibration and navController.job.index
    local flightJob=navController.job.phase=="VERIFY" and navController.job
    local flightGoal=flightJob and (flightJob.returning and flightJob.legStart or flightJob.goals[flightJob.leg])
    local flightStage=flightJob and ("CAL_"..flightJob.leg.."_TRY_"..(flightJob.retry+1)..(flightJob.returning and "_RETURN" or ""))
    local commands,message,completed=navController:calibrate(sample)
    autopilotCommands=commands or {}
    local job=navController.job
    local checkpointError
    if job and job.kind=="BRAKES" and job.index>brakeIndex then
      local saved,err=pcall(saveBrakeCheckpoint,job)
      if not saved then checkpointError=tostring(err) end
    end
    if flightGoal then recordFlight(sample,flightGoal,flightJob.candidate,commands,flightStage) end
    if job and job.kind=="BRAKES" then recordFlight(sample,job.goal or job.hold,job.candidate,commands,"CAL_BRAKE_"..job.index.."_"..job.phase) end
    if job and job.phase=="VERIFY" and (not pendingAutopilot or pendingRetry~=job.persistRevision) then
      local saved,err=pcall(savePendingAutopilot,job.candidate)
      if saved then pendingAutopilot=job.candidate; pendingRetry=job.persistRevision
      else autopilotNotice="Thruster model is in memory but checkpoint failed: "..tostring(err); autopilotNoticeColor=colors.red end
    end
    if completed then
      local saved,err=pcall(brakingPending and savePendingAutopilot or saveAutopilotProfile,completed)
      if saved then
        if brakingPending then
          clearBrakeCheckpoint()
          pendingAutopilot=completed; pendingRetry=nil
          autopilotNotice="BRAKE CAL saved; press FULL AUTO CAL for the 5 flight tests."
        else
          if brakingCalibration then clearBrakeCheckpoint() end
          if not brakingCalibration then clearPendingAutopilot(); pendingAutopilot=nil; pendingRetry=nil end
          navController.profile=completed; applyGainValues(completed.gains)
          autopilotNotice=brakingCalibration and ("BRAKE CAL saved at Y %.0f, pressure %.3f; 4 directions."):format(completed.braking.height,completed.braking.pressure)
            or ("FULL AUTO CAL saved: %.1f b/s peak, %.0f%% thrust."):format(completed.verification and completed.verification.peakSpeed or completed.maxSpeed,(completed.maxThrottle or .35)*100)
        end
        autopilotNoticeColor=colors.lime
      else autopilotNotice="Calibration passed but could not save: "..tostring(err); autopilotNoticeColor=colors.red end
    elseif not commands then autopilotNotice=message; autopilotNoticeColor=colors.red
    else autopilotNotice=message; autopilotNoticeColor=colors.orange end
    if checkpointError then autopilotNotice="Stop kept in memory, but disk checkpoint failed: "..checkpointError; autopilotNoticeColor=colors.red end
    if completed or not commands then
      brakeCalibrationActive=false
      fullCalibration,calibrationRunning=false,false
      mode="HOLDING ALTITUDE"; autopilotCommands={}; navController:reset(shipHeading)
    end
    return
  end
  if (mode == "AUTOPILOT" or mode == "FINAL TURN" or mode == "DESTINATION REACHED") and target then
    vectorX,vectorY,power=0,0,0
    if not gpsFresh then autopilotCommands={}; return end
    local sample=navigationSample()
    if not Autopilot.validSample(sample) then
      autopilotCommands={}; mode="NAV LOST - RE-ENGAGE"; autopilotNotice="Fresh heading, yaw rate and pressure required; re-engage AUTO."
      autopilotNoticeColor=colors.red
      return
    end
    local wasReached=mode=="DESTINATION REACHED"
    local wasFinalTurn=mode=="FINAL TURN"
    local commands,arrived=navController:navigate(sample,target)
    autopilotCommands=commands or {}
    recordFlight(sample,target,navController.profile,commands,mode)
    if arrived and mode=="AUTOPILOT" then
      navController:setHeadingTarget(nearestCardinalHeading(sample.heading))
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
  local buttons=gpuButtons
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
    local calDetails=calibrationReadout()
    txt(calDetails and "FLIGHT TEST GEOMETRY" or "SENSOR / SWEEP DETAIL",l.margin+14,detailY+8,detailFont,muted,"bold")
    txt(calDetails or ("AVIONICS ALTITUDE  %s     PRESSURE  %s"):format(altitudeSensorHeight and ("%.1f"):format(altitudeSensorHeight) or "--",altitudeSensorPressure and ("%.3f"):format(altitudeSensorPressure) or "--"),l.margin+14,detailY+math.floor(detailH*.38),detailFont,ink)
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
  for _,b in ipairs(buttons) do
    rect(b.x,b.y,b.w,b.h,b.color)
    local textSize=math.max(13,math.floor(h*.022))
    local tx=b.align=="left" and b.x+12 or b.x+math.max(6,math.floor((b.w-#b.label*textSize*.55)/2))
    txt(b.label,tx,b.y+math.floor((b.h-textSize)/2),textSize,ink,"bold")
  end
  gpu.updateDisplay(display)
  gpuVisibleButtons=buttons
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
  local canceledCalibration=calibrationRunning
  if position or canceledCalibration then
    if position then altitude=math.floor(position.y+.5) end
    target=nil; setFlightMode(position and "HOLDING ALTITUDE" or "WAITING FOR GPS")
    if canceledCalibration then
      clearBrakeCheckpoint()
      manualBurner,burnerCommand=burner,burner
      autopilotNotice="Calibration canceled; burner held."
      autopilotNoticeColor=colors.orange; status.text=autopilotNotice; status.foreground=colors.orange
    end
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
  elseif not position or not target then refuse("GPS fix and destination are required.")
  else
    setFlightMode("AUTOPILOT"); navController:reset(sample.heading); autopilotNotice=nil; flightRecording=nil
    status.text = "Autopilot engaged; final heading is cardinal"; status.foreground = colors.lime
  end
end
local function startFullCalibration()
  if calibrationRunning then status.text="Calibration is running; HOLD cancels it."; return end
  if burnerSweep then status.text="Finish or stop the burner sweep first."; return end
  if mode=="STOPPED" then
    autopilotNotice,autopilotNoticeColor="Set a stable burner level before FULL AUTO CAL.",colors.orange
    status.text,status.foreground=autopilotNotice,autopilotNoticeColor
    return
  end
  if not position then
    autopilotNotice,autopilotNoticeColor="GPS: "..tostring(FreshGPS.error or "waiting for a fix").."; FULL AUTO CAL needs a valid fix.",colors.orange
    status.text,status.foreground=autopilotNotice,autopilotNoticeColor
    return
  end
  flightRecording=nil
  brakeCalibrationActive=false
  if pendingAutopilot then
    local sample=navigationSample()
    local resumed,message=navController:resumeVerification(sample,pendingAutopilot)
    if not resumed then
      autopilotNotice,autopilotNoticeColor=message,colors.orange
      status.text,status.foreground=message,colors.orange
      return
    end
    pendingRetry=nil
    manualBurner,burnerCommand=burner,burner
    vectorX,vectorY,power,target=0,0,0,nil
    autopilotCommands={}; fullCalibration,calibrationRunning=true,true
    mode="FULL AUTO CAL"; autopilotNotice="Resuming auto tests from saved thruster measurements."
    autopilotNoticeColor=colors.orange; status.text=autopilotNotice; status.foreground=colors.orange
    return
  end
  navController:cancel()
  brakeCalibrationActive=false
  manualBurner,burnerCommand=burner,burner
  vectorX,vectorY,power,target=0,0,0,nil
  autopilotCommands={}; fullCalibration,calibrationRunning=true,true
  mode="FULL AUTO CAL"; autopilotNotice="Measuring, then long cruise tests; HOLD cancels."
  autopilotNoticeColor=colors.orange
  status.text=autopilotNotice; status.foreground=colors.orange
end
local function startBrakingCalibration()
  if calibrationRunning or burnerSweep then status.text="Finish or cancel the current calibration first."; return end
  if mode=="STOPPED" then status.text="Set a stable burner level before BRAKE CAL."; return end
  local sample=navigationSample()
  local model=pendingAutopilot or navController.profile
  local checkpoint=loadBrakeCheckpoint(model,pendingAutopilot~=nil)
  local started,message=navController:beginBraking(sample,pendingAutopilot)
  if not started then status.text=message; autopilotNotice=message; autopilotNoticeColor=colors.orange; return end
  if checkpoint then
    local job=navController.job
    job.candidate,job.pending=checkpoint.candidate,checkpoint.pending
    job.index,job.runs=checkpoint.index,checkpoint.runs
    job.pressure,job.height=checkpoint.pressure,checkpoint.height
    job.heightMin,job.heightMax=checkpoint.heightMin,checkpoint.heightMax
    job.heading,job.previous=checkpoint.heading,{}
    for key,value in pairs(sample) do job.previous[key]=value end
    job.hold,job.origin={x=sample.x,z=sample.z},{x=sample.x,z=sample.z}
    job.phase,job.at,job.stable,job.run,job.interrupted="SETTLE",sample.t,nil,nil,true
  elseif fs.exists(brakeCheckpointPath) or fs.exists(brakeCheckpointPath..".bak") then clearBrakeCheckpoint() end
  altitude=altitude or position.y
  manualBurner,burnerCommand=burner,burner
  vectorX,vectorY,power,target=0,0,0,nil
  autopilotCommands={}; fullCalibration,calibrationRunning=true,true; flightRecording=nil
  brakeCalibrationActive=true
  mode="BRAKE CAL"
  autopilotNotice=checkpoint and ("BRAKE CAL resumed: %d/8 completed stops kept; repeating only the unfinished stop."):format(checkpoint.index-1)
    or "BRAKE CAL: 8 automatic stops; current burner held. H cancels."
  autopilotNoticeColor=colors.orange; status.text=autopilotNotice; status.foreground=colors.orange
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
    local bw=math.floor((l.w-2*l.margin-2*l.gap)/3)
    local y=l.contentY+math.max(18,math.floor(l.h*.032))+4
    local bh=math.max(24,math.floor(l.h*.045))
    addGpuButton(l.margin,y,bw,bh,"BRAKE CAL",{46,107,130},startBrakingCalibration)
    addGpuButton(l.margin+bw+l.gap,y,bw,bh,"FULL AUTO CAL",{46,107,130},startFullCalibration)
    addGpuButton(l.margin+2*(bw+l.gap),y,bw,bh,"CANCEL",{100,75,35},holdHere)
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
  monitorFrame:addButton({x=29, y=8, width=14, height=1, text="BRAKE CAL F6", background=colors.blue}):onClick(startBrakingCalibration)
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
  frame:addButton({x=24, y=15, width=12, height=1, text="BRAKE CAL F6", background=colors.blue}):onClick(startBrakingCalibration)
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
calibrationPage:addButton({x=2, y=10, width=20, height=1, text="BRAKE CAL  F6", background=colors.blue}):onClick(startBrakingCalibration)
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
  if navText then navText.text=calibrationReadout() or (compact and "SPD %.1f  D %s  ETA %s" or "SPEED %.1f b/s   DISTANCE %s   ETA %s"):format(speed,distance and ("%.1f"):format(distance) or "--",eta) end
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
  elseif mode == "FINAL TURN" then status.text, status.foreground = "AT DESTINATION  |  ALIGNING CARDINAL", colors.orange
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
    monitorSystem.text = calibrationReadout() or ("B%d/15 T%.1f SPD %.1f GPS %s"):format(burner, measuredThrust,speed,position and "OK" or "LOST")
    monitorStatus.text, monitorStatus.foreground = status.text, status.foreground
  end
  if directGpu and (gpuRedrawPending or os.clock() - lastGpuRender >= .025) then
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
  for _, button in ipairs(gpuVisibleButtons) do
    if x >= button.x and x < button.x + button.w and y >= button.y and y < button.y + button.h then
      local key = button.x .. ":" .. button.y .. ":" .. button.w .. ":" .. button.h
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
      sleep(gpuRedrawPending and 0 or .025)
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
        elseif key == keys.f6 then startBrakingCalibration()
        elseif key == keys.f8 then saveFlightReport()
        end
      end
    end
  end)
  print("Starting flight panel...")
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
