-- Lua 5.2+: exercise the real setup code with a cooperative CC event scheduler.
local file=assert(io.open("airship/airship.lua","r"))
local source=file:read("*a"):gsub("\r\n","\n"); file:close()
local startup=assert(source:match("^(.-)%-%- Resolve devices"))

local function check(scenario)
  local queue,log,timer={}, {},false
  local env=setmetatable({}, {__index=_G})
  env.print=function() end
  env.require=function() return {} end
  env.fs={open=function(_,mode)
    if mode=="w" then log={} end
    return {writeLine=function(line) log[#log+1]=line end,close=function() end}
  end}
  env.os={queueEvent=function(name) queue[#queue+1]={name} end,
    pullEvent=function(filter) return coroutine.yield(filter) end}
  env.sleep=function(seconds) assert(seconds==8); timer=true; coroutine.yield("timer") end
  env.parallel={waitForAny=function(...)
    local threads,filters={},{}
    for i,fn in ipairs({...}) do threads[i]=coroutine.create(fn) end
    local event={}
    while true do
      for i,thread in ipairs(threads) do
        if not filters[i] or filters[i]==event[1] then
          local ok,filter=coroutine.resume(thread,table.unpack(event))
          assert(ok,filter)
          if coroutine.status(thread)=="dead" then return i end
          filters[i]=filter
        end
      end
      event={coroutine.yield()}
    end
  end}
  local created,mouse,scanned=0,false,false
  local gpu={
    listDisplays=function()
      if scenario=="timeout" then coroutine.yield("never") end
      return (scenario=="new" or scenario=="no-monitor" or scenario=="create-error") and {} or {0}
    end,
    getDisplayInfo=function(id)
      assert(id==0,"Display ID zero must be usable")
      if scenario=="bad-info" then return true end
      return {pixelWidth=656,pixelHeight=492}
    end,
    autoDetectAndCreateDisplayWithResolution=function(name,scale)
      assert(name=="monitor_7" and scale==1,"Creation must target the exact wired monitor without a broad scan")
      created=created+1
      if scenario=="create-error" then error("monitor unavailable") end
      return 0
    end,
    enableMouseEvents=function(id,on,hover)
      assert(id==0 and on and hover==false)
      if scenario=="mouse-error" then error("mouse unavailable") end
      mouse=true
    end,
    autoDetectMonitor=function() scanned=true; return {} end,
  }
  env.peripheral={find=function(kind) if kind=="monitor" then return {} end end,
    getName=function() if scenario=="no-monitor" then error("not wrapped") end; return "monitor_7" end,
    wrap=function() return gpu end}
  local thread=coroutine.create(assert(load(startup..
    "\nreturn setupDirectGpu(), directGpuEnabled, directGpu, directGpuError", "@gpu-startup","t",env)))
  local event={}
  while true do
    local result=table.pack(coroutine.resume(thread,table.unpack(event)))
    assert(result[1],result[2])
    if coroutine.status(thread)=="dead" then
      assert(not scanned,"Setup failure must not start a broad diagnostic scan")
      local ready,enabled,device,err=result[2],result[3],result[4],result[5]
      if scenario=="new" or scenario=="reused" then
        assert(ready and enabled and device==gpu and mouse and not err)
        assert(created==(scenario=="new" and 1 or 0),"Reused displays must not be recreated")
        assert(log[#log]=="ready")
      else
        assert(not ready and not enabled and device==nil and type(err)=="string")
        if scenario=="timeout" then assert(err=="Timed out at listDisplays") end
        if scenario=="no-monitor" then assert(created==0 and err:find("Monitor name unavailable",1,true)) end
        assert(log[#log]:find("using CC monitor",1,true))
      end
      break
    end
    event=table.remove(queue,1)
    if not event then assert(timer,"Setup deadlocked without a deadline"); timer=false; event={"timer",1} end
  end
end
for _,scenario in ipairs({"new","reused","timeout","bad-info","mouse-error","no-monitor","create-error"}) do check(scenario) end
do
  local env=setmetatable({print=function() end,require=function() return {} end,
    peripheral={find=function() end},os={queueEvent=function() end,pullEvent=function() end}}, {__index=_G})
  local enabled=assert(load(startup.."\nreturn directGpuEnabled","@gpu-recovery","t",env))
  assert(enabled() and not enabled("--no-gpu"),"Recovery argument must skip GPU only for that invocation")
end
print("PASS: exact monitor creation, display zero reuse, resolution 1, timeout and failure fallback without broad scans")
