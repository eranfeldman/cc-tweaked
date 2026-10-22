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
  for i=0,23 do names[#names+1] = "liquid_vector_thruster_" .. i end
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
  assert(count == 24 and counts() > 24, "All 24 hub thrusters were not discovered")
end

local helper = assert(source:match("(local yieldSequence = 0.-)\nyieldComputer%(%)"))
for _, reused in ipairs({false, true}) do
  local env, run, call = fixture()
  local enabled = false
  local gpu = {
    listDisplays=function() call(); return reused and {1} or {} end,
    getDisplayInfo=function() call(); return {pixelWidth=1024, pixelHeight=768} end,
    autoDetectAndCreateDisplayWithResolution=function(name, resolution) call(); assert(name == "monitor_0" and resolution == 1); return 1 end,
    enableMouseEvents=function(id, value, hover)
      call(); assert(id == 1 and value == true and hover == false); enabled = value
    end,
  }
  env.require = function() return {} end
  env.peripheral = {find=function(kind) if kind == "monitor" then return {} end end,
    getName=function() return "monitor_0" end, wrap=function() return gpu end}
  env.fs = {open=function() return {writeLine=function() end,close=function() end} end}
  local setup = assert(source:match("^(.-)%-%- Resolve devices"))
  local ok, ready = run(assert(load(setup .. "return configureDirectGpu()", "@display-setup", "t", env)))
  assert(ok and ready and enabled, "New and reused displays must enable mouse input")
end

do
  local env, run, call = fixture()
  local clicked, fallback, events = 0, false, {}
  for i=1,24 do events[i] = {type="mouse_hover",x=5,y=5} end
  events[#events+1] = {type="mouse_click",x=5,y=5}
  events[#events+1] = {type="mouse_up",x=5,y=5}
  events[#events+1] = {type="mouse_click",x="bad",y=5}
  env.gpuButtons = {{x=0,y=0,w=10,h=10,label="CALIBRATION",action=function() clicked=clicked+1 end}}
  env.directDisplay, env.gpuRenderRevision = 1, 0
  env.directGpuEnabled = true
  env.createMonitorUi = function() fallback=true end
  env.directGpu.pollEvent = function() call(); return table.remove(events, 1) end
  env.monitorName, env.directWidth, env.directHeight = "monitor_0", 1000, 800
  env.monitor = {getSize=function() return 100, 80 end}
  local input = assert(source:match("(local lastGpuClick%s*.-)\nlocal ok, err = pcall"))
  local poll, touch, gpuMouse = assert(load(helper .. "\n" .. input .. "\nreturn pollGpuInput, handleMonitorTouch, handleGpuMouse", "@input", "t", env))()
  local ok, err = run(poll)
  assert(ok, err)
  assert(clicked == 1 and #events == 0 and env.gpuRedrawPending and env.gpuRenderRevision == 1,
    "Movement events must not block a click or run its action twice")
  touch("monitor_0", 1, 1)
  gpuMouse(1, "mouse_click", 5, 5)
  assert(clicked == 1, "One click reported through all three routes must run once")
  env.os.clock = function() return 1 end
  touch("monitor_9", 1, 1)
  gpuMouse(2, "mouse_click", 5, 5)
  gpuMouse(1, "mouse_drag", 5, 5)
  assert(clicked == 1, "Other displays and drag events must not activate buttons")
  touch("monitor_0", 1, 1)
  touch("monitor_0", 1, 1)
  assert(clicked == 3, "Normal CC monitor touches must map to GPU buttons and allow double taps")
  env.os.clock = function() return 2 end
  gpuMouse(1, "mouse_click", 5, 5)
  assert(clicked == 4, "Native GPU mouse events must activate buttons without polling")
  env.directGpu.pollEvent = function() call(); error("GPU disconnected") end
  ok, err = run(poll)
  assert(ok, err)
  assert(fallback and env.directGpu == nil and not env.directGpuEnabled, "Polling errors must restore the monitor fallback")
end

do
  local env, run, call, counts = fixture()
  env.thrusters, env.redstone = {}, {setAnalogOutput=function() end}
  for i=1,24 do env.thrusters[i] = {setVector=call, setThrustNormalized=call} end
  local outputs = assert(source:match("(local function outputs%b()%s*.-)\nlocal function stop"))
  local send = assert(load(helper .. "\n" .. outputs .. "\nreturn outputs", "@outputs", "t", env))()
  local ok, err = run(function() send(true) end)
  assert(ok, err)
  assert(counts() == 48, "Every thruster must receive both commands")
end
do
  local env, run = fixture()
  local powers={}
  env.thrusters={t0={setVector=function() end,setThrustNormalized=function(value) powers[#powers+1]=value end}}
  env.redstone={setAnalogOutput=function() end}
  env.autopilotCommands={t0={x=.2,y=0,power=.3}}
  env.power=.3
  local outputs=assert(source:match("(local function outputs%b()%s*.-)\nlocal function stop"))
  local send=assert(load(helper.."\n"..outputs.."\nreturn outputs","@cancel-output","t",env))()
  assert(run(function() send(true) end,function() env.autopilotCommands=nil; env.power=0 end))
  assert(#powers==1 and powers[1]==0,"A canceled output pass must not restore stale thruster power")
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
  env.navigationReadout = function() return 0, nil, "--" end
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
  local env=fixture()
  local started,canceled=false,false
  env.directWidth,env.directHeight,env.gpuTab=656,492,"CALIBRATION"
  env.startFullCalibration=function() started=true end
  env.holdHere=function() canceled=true end
  local add=assert(source:match("(local function addGpuButton%b()%s*.-)\nlocal function renderDirectMonitor"))
  local menu=assert(source:match("(local function createGpuButtons%(%)%s*.-)\ncreateGpuButtons%(%)"))
  local build=assert(load(layout.."\n"..add.."\n"..menu.."\nreturn createGpuButtons","@calibration-menu","t",env))()
  build()
  for _,button in ipairs(env.gpuButtons) do
    assert(button.x>=0 and button.y>=0 and button.x+button.w<=656 and button.y+button.h<=492,"Calibration control does not fit 4x3 monitor")
    if button.label=="FULL AUTO CAL" then button.action() end
  end
  assert(started,"Full calibration button is missing from the monitor menu")
  env.fullCalibration=true; build()
  for _,button in ipairs(env.gpuButtons) do if button.label=="CANCEL AUTO CAL" then button.action() end end
  assert(canceled,"Running calibration must have a monitor cancel control")
end

do
  local env, run, render, counts = renderFixture("FLIGHT")
  local ok, err = run(render, function() env.gpuRenderRevision = 1 end)
  assert(not ok and err == "DirectGPU frame superseded", tostring(err))
  local _, presented = counts()
  assert(presented == 0, "An interrupted frame must not be presented")
end
print("PASS: discovery, input setup, native monitor and GPU clicks, duplicate clicks, backlog, fallback, thrusters, all tabs, and interrupted frames")
