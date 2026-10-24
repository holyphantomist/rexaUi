local cyan = 0x37DFDC
local white = 0xECF5F9
local muted = 0x7D97A6

function draw()
    rexa.clear(0x0B1220)
    local now = rexa.clock()
    if now then
        rexa.text(70, 70, string.format("%02d:%02d:%02d",
                   now.hour, now.minute, now.second), cyan, 3)
        rexa.text(100, 112, string.format("%04d-%02d-%02d",
                   now.year, now.month, now.day), white, 1)
    else
        rexa.text(100, 90, "TIME UNAVAILABLE", 0xFFB15E, 1)
    end
    rexa.text(100, 140, "REXAUI SD LUA APP", muted, 1)
end
