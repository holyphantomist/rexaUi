local cyan = 0x37DFDC
local white = 0xECF5F9
local muted = 0x7D97A6
local elapsed = 0
local sensor = nil
local sensor_message = "READING QMI8658"
local scan_requested = false
local selected_device = 1

function update(input, dt)
    elapsed = elapsed + dt
    if elapsed >= 0.25 then
        elapsed = 0
        local latest, err = rexa.imu()
        sensor = latest
        sensor_message = err or ""
    end
    local devices = rexa.ble_devices()
    if #devices > 0 and input.direction == "up" then
        selected_device = (selected_device - 2) % #devices + 1
    elseif #devices > 0 and input.direction == "down" then
        selected_device = selected_device % #devices + 1
    end
    if input.pressed then
        if not scan_requested then
            scan_requested = rexa.ble_scan() or false
        elseif #devices > 0 then
            rexa.ble_connect(selected_device)
        else
            rexa.ble_scan()
        end
    end
end

function draw()
    rexa.clear(0x0B1220)
    rexa.text(14, 30, "MOTION / BLE", cyan, 1)
    if sensor then
        rexa.text(14, 52, string.format("ACC X %.1f", sensor.acceleration.x),
                  white, 1)
        rexa.text(14, 68, string.format("ACC Y %.1f", sensor.acceleration.y),
                  white, 1)
        rexa.text(14, 84, string.format("ACC Z %.1f", sensor.acceleration.z),
                  white, 1)
        rexa.text(170, 52, string.format("GYRO X %.1f", sensor.gyro_dps.x),
                  muted, 1)
        rexa.text(170, 68, string.format("TEMP %.1f C", sensor.temperature_c),
                  muted, 1)
    else
        rexa.text(14, 60, sensor_message, 0xFFB15E, 1)
    end

    local devices = rexa.ble_devices()
    rexa.text(14, 112, string.format("BLE DEVICES %d", #devices), cyan, 1)
    if #devices > 0 then
        selected_device = math.min(selected_device, #devices)
        rexa.text(14, 128, string.format("%d: %s",
                  selected_device, devices[selected_device].name), white, 1)
        rexa.text(14, 148, "UP/DOWN SELECT / CLICK", muted, 1)
    else
        rexa.text(14, 128, "CLICK TO SCAN", muted, 1)
    end
end
