-- Run from the repository root with Lua 5.2 or newer.
local file = assert(io.open("airship/airship.lua", "r"))
local source = file:read("*a"):gsub("\r\n", "\n")
file:close()
assert(load(source, "@airship.lua"))

local function fixture()
  local queue, calls, sinceYield, presented = {}, 0, 0, 0
  local env = setmetatable({print=function() end}, {__index=_G})
  env.os = {
    queueEvent=function(name) queue[#queue + 1] = name end,
    pullEvent=function(name) return coroutine.yield(name) end,
    clock=function() return 0 end,
  }
  env.sleep = function() error("GPU checkpoints must not wait for timers") end
  local function call()
    calls, sinceYield = calls + 1, sinceYield + 1
  end
  local function run(fn, onYield)
    local co = coroutine.create(fn)
    local ok, result = coroutine.resume(co)
    while ok and coroutine.status(co) ~= "dead" do
      assert(sinceYield <= 1, "Peripheral calls ran back to back without yielding")
      assert(table.remove(queue, 1) == result, "Checkpoint did not queue its wake event")
      assert(result:match("^airship_yield_"), "Checkpoint must yield to a private event")
      sinceYield = 0
      if onYield then onYield() end
      ok, result = coroutine.resume(co, result)
    end
    assert(sinceYield <= 1, "Peripheral calls ran back to back without yielding")
    return ok, result
  end
  env.directGpu = {clear=call, fillRect=call, drawText=call,
    updateDisplay=function() call(); presented = presented + 1 end}
  return env, run, call, function() return calls, presented end
end

-- Discovery traverses hubs cooperatively even when remote calls do not yield.
do
  local env, run, call, counts = fixture()
  local names = {}
  for i=1,8 do names[i] = "liquid_vector_thruster_" .. i end
  local hub = {
    getNamesRemote=function() call(); return names end,
    getTypeRemote=function() call(); return "liquid_vector_thruster" end,
    callRemote=function() end,
  }
  env.require = function() return {} end
  env.peripheral = {find=function() end, getNames=function() return {"hub"} end,
    getType=function() return "peripheral_hub" end, wrap=function() return hub end}
  local startup = assert(source:match("^(.-)local thrusters = findThrusters%(%)"))
  local ok, devices = run(assert(load(startup .. "return findThrusters()", "@discovery", "t", env)))
  assert(ok, devices)
  local count = 0
  for _ in pairs(devices) do count = count + 1 end
  assert(count == 8 and counts() > 8, "Hub thrusters were not discovered")
end

local helper = assert(source:match("(local yieldSequence = 0.-)\nyieldComputer%(%)"))
do
  local env, run, call, counts = fixture()
  env.thrusters, env.redstone = {}, {setAnalogOutput=function() end}
  for i=1,8 do env.thrusters[i] = {setVector=call, setThrustNormalized=call} end
  local outputs = assert(source:match("(local function outputs%b()%s*.-)\nlocal function stop"))
  local send = assert(load(helper .. "\n" .. outputs .. "\nreturn outputs", "@outputs", "t", env))()
  local ok, err = run(function() send(true) end)
  assert(ok, err)
  assert(counts() == 16, "Every thruster must receive both commands")
end
local layout = assert(source:match("(local function gpuLayout%(%)%s*.-)\nlocal function addGpuButton"))
local renderer = assert(source:match("(local function renderDirectMonitor%(%)%s*.-)\nlocal monitorStatus"))
local function renderFixture(tab)
  local env, run, _, counts = fixture()
  env.directDisplay, env.directWidth, env.directHeight = 1, 1024, 768
  env.AIRSHIP_BUILD = "test"
  env.gpuTab, env.gpuHeightPage, env.gpuRenderRevision = tab, 1, 0
  env.mode, env.burner, env.measuredThrust = "HOLDING ALTITUDE", 6, 0
  env.position, env.velocity, env.altitude = {x=1,y=70,z=2}, {y=.1}, 75
  env.learnedHover, env.gpuButtons = {}, {{x=1,y=1,w=60,h=35,label="FLIGHT",color={1,2,3}}}
  env.savedHoverPoints = function() return {} end
  local render = assert(load(helper .. "\n" .. layout .. "\n" .. renderer .. "\nreturn renderDirectMonitor", "@renderer", "t", env))()
  return env, run, render, counts
end

for _, tab in ipairs({"FLIGHT", "CALIBRATION", "HEIGHTS"}) do
  local _, run, render, counts = renderFixture(tab)
  local ok, err = run(render)
  assert(ok, err)
  local calls, presented = counts()
  assert(calls > 5 and presented == 1, "Only a complete frame should be presented")
end

do
  local env, run, render, counts = renderFixture("FLIGHT")
  local ok, err = run(render, function() env.gpuRenderRevision = 1 end)
  assert(not ok and err == "DirectGPU frame superseded", tostring(err))
  local _, presented = counts()
  assert(presented == 0, "An interrupted frame must not be presented")
end
print("PASS: discovery, thruster commands, all GPU tabs, and interrupted frames yield without timer delays")
