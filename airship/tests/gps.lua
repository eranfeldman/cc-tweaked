-- Lua 5.2+: fresh-query GPS, range validation, and impossible-fix rejection.
local file=assert(io.open('airship/airship.lua','r'))
local source=file:read('*a'):gsub('\r\n','\n'); file:close()
local core=assert(source:match('%-%- BEGIN GPS FIX CORE\n(.-)\n%-%- END GPS FIX CORE'))
local function api(env) return assert(load(core..'\nreturn FreshGPS','@gps-core','t',env or _G))() end
local G=api()
local hosts={{0,0,0},{80,0,0},{0,0,80},{0,60,0},{70,50,70}}
local function fixes(x,y,z,offset)
  local result={}
  for _,host in ipairs(hosts) do
    local distance=math.sqrt((x-host[1])^2+(y-host[2])^2+(z-host[3])^2)
    result[#result+1]={x=host[1]+offset,y=host[2],z=host[3]+offset,distance=distance}
  end
  return result
end
for _,offset in ipairs({0,20000000}) do
  local p=assert(G.solve(fixes(15,110,-35,offset)))
  assert(math.abs(p.x-offset-15)<1e-6 and math.abs(p.y-110)<1e-6 and math.abs(p.z-offset+35)<1e-6)
end
local bad=fixes(15,110,-35,0); bad[4].distance=bad[4].distance+20
assert(not G.solve(bad),'Inconsistent ranges must not become a usable position')
local flat=fixes(15,110,-35,0)
table.remove(flat,5); flat[4]={x=80,y=0,z=80,distance=100}
assert(not G.solve(flat),'Coplanar hosts must not return an arbitrary mirror position')
print('PASS: GPS solves translated coordinates and rejects inconsistent ranges and flat host geometry')

local recorded={
  {x=0,y=0,z=0,distance=82.088344387723666},
  {x=7,y=-3,z=-17,distance=88.604051784589927},
  {x=-3,y=-3,z=-6,distance=87.072611118126034},
  {x=-14,y=-3,z=10,distance=87.919300339155669},
}
for _,offset in ipairs({0,20000000}) do
  local input={}
  for _,f in ipairs(recorded) do input[#input+1]={x=f.x+offset,y=f.y,z=f.z+offset,distance=f.distance} end
  local p=assert(G.solve(input),'The actual reported ranges must fit without raising the rejection threshold')
  assert(G.residual<.005,'Refinement must bring the recorded worst range error below 0.005 blocks')
  assert(math.abs(p.x-offset-19.91719066)<1e-5 and math.abs(p.y-77.81224016)<1e-5 and math.abs(p.z-offset-16.94960202)<1e-5)
end
local atHost=assert(G.solve(fixes(0,0,0,0)))
assert(math.abs(atHost.x)+math.abs(atHost.y)+math.abs(atHost.z)<1e-6,'A receiver at a host must not divide by a zero range')
print('PASS: actual v1.6.9 range-disagreement report fits within 0.005 blocks, including large coordinates and zero range')

local state={}
assert(G.accept(state,{t=0,x=0,y=20,z=0},12))
assert(G.accept(state,{t=.6,x=.039738,y=20,z=.030128},12))
assert(G.accept(state,{t=1.2,x=.36784,y=20,z=.21105},12))
assert(not G.accept(state,{t=1.8,x=-8.304,y=20,z=6.7619},12),'Recorded GPS jump must not update navigation')
assert(state.last.t==1.2,'A rejected position must preserve the last good fix')
assert(G.accept(state,{t=2.4,x=.8,y=20,z=.4},12),'A good fix must recover after a rejected spike')
assert(not G.accept(state,{t=3,x=74,y=20,z=-94},12),'119-block excursion must be rejected')
local moving={}
for i=0,100 do assert(G.accept(moving,{t=i*.6,x=i*6,y=20,z=0},12),'Legitimate 10 b/s cruise must not be capped') end
moving={}
for i=0,100 do assert(G.accept(moving,{t=i*.6,x=i*14.4,y=20,z=0},12),'GPS startup must accept legitimate 24 b/s cruise') end
print('PASS: rejects actual recorded excursions while allowing continuous 10 b/s flight')

local stationary={}
for i=0,120 do
  assert(G.accept(stationary,{t=i*.05,x=i==1 and 1 or 0,y=20,z=0},12),
    'A one-block GPS wobble must not trap a stationary ship in jump rejection')
end
local gap={}
assert(G.accept(gap,{t=0,x=0,y=20,z=0},12))
assert(G.accept(gap,{t=.5,x=0,y=20,z=0},12))
assert(G.accept(gap,{t=4.5,x=32,y=20,z=0},12),'Acceleration allowance must cover the entire gap, not just two seconds')
assert(not G.accept(gap,{t=4.6,x=200,y=20,z=0},12),'Gap recovery must still reject a subsequent impossible jump')
local check=G.check
assert(check and check.allowance and check.dx>check.allowance,'Rejected fixes must retain relative jump evidence')
G.fixes=fixes(123456,20,-654321,20000000); G.error='GPS jump rejected'
local report=G.report()
assert(report.error==G.error and report.check==check and #report.hosts==5)
for i,host in ipairs(report.hosts) do
  assert(host.x==hosts[i][1]-hosts[1][1] and host.z==hosts[i][3]-hosts[1][3], 'GPS report must not expose absolute host X/Z')
end
print('PASS: stationary jitter and long gaps recover without accepting impossible jumps; failure report uses relative positions')

for _,diskFailure in ipairs({false,true}) do
  local written,closed
  local env=setmetatable({AIRSHIP_BUILD='test',navController={},colors={orange=1},mode='WAITING FOR GPS',
    gpsFilter={last={t=.5,x=0,y=20,z=0},vx=0,vz=0,dt=.5}}, {__index=_G})
  env.os={clock=function() return 1 end}
  env.FreshGPS={locate=function() return 100,20,0 end,accept=G.accept,report=function() return G.report() end}
  env.textutils={serialize=function(value) assert(value.check.dx>value.check.allowance); return 'relative rejection evidence' end}
  env.fs={open=function(path,mode)
    assert(path=='airship-gps.log' and mode=='w')
    if diskFailure then error('disk full') end
    return {write=function(value) written=value end,close=function() closed=true end}
  end}
  local code=assert(source:match('(local function locate%b()%s*.-)\nlocal function updateFlight'))
  local locate=assert(load('local gpsReportAt\n'..code..'\nreturn locate','@gps-failure-log','t',env))()
  assert(locate()==false and env.FreshGPS.error=='GPS jump rejected')
  assert(diskFailure or written:find('relative rejection evidence',1,true) and closed,'GPS failure report must be written and closed')
end
print('PASS: controller saves a GPS failure report before auto tests; disk errors do not stop the controller')

for _,scenario in ipairs({'success','recorded_ranges','timeout','bad_ranges','error'}) do
  local opened,closed,canceled,events,query={}, {},false,{},nil
  local modem={isOpen=function(channel) return opened[channel] or false end,
    open=function(channel) opened[channel]=true end,close=function(channel) closed[channel]=true end}
  modem.transmit=function(channel,reply,message)
    assert(channel==65534 and reply~=65534 and message=='PING')
    if query then assert(reply~=query,'Each query must have its own reply channel') end
    query=reply
    if scenario=='error' then error('modem failure') end
    events={{'modem_message','top',65534,65534,{100,100,100},1},
      {'modem_message','top',reply-1,65534,{100,100,100},1}}
    local input=fixes(15,110,-35,0)
    if scenario=='recorded_ranges' then input=recorded end
    if scenario=='bad_ranges' then input[4].distance=input[4].distance+20 end
    for i,f in ipairs(input) do
      if scenario~='timeout' or i<4 then events[#events+1]={'modem_message','top',reply,65534,{f.x,f.y,f.z},f.distance} end
    end
    events[#events+1]={'timer',42}
  end
  local env=setmetatable({}, {__index=_G})
  env.rs={getSides=function() return {'top'} end}
  env.peripheral={getType=function() return 'modem' end,call=function() return true end,wrap=function() return modem end}
  env.os={startTimer=function() return 42 end,cancelTimer=function(id) assert(id==42); canceled=true end,
    pullEvent=function() return table.unpack((assert(table.remove(events,1),'query waited past timeout'))) end}
  local gps=api(env)
  for _=1,2 do
    local x,y,z=gps.locate(.5)
    assert(closed[query] and canceled,'GPS must close its temporary channel and cancel its timer')
    if scenario=='success' then assert(math.abs(x-15)<1e-6 and math.abs(y-110)<1e-6 and math.abs(z+35)<1e-6)
    elseif scenario=='recorded_ranges' then assert(math.abs(x-19.91719066)<1e-5 and gps.residual<.005 and not gps.error,'Fresh query must accept the actual refined ranges')
    else assert(x==nil and gps.error,'Failed query must not return a stale position') end
  end
end
print('PASS: fresh reply channels isolate late replies; timeout, bad ranges, and errors clean up correctly')
