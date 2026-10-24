local cyan = 0x37DFDC
local white = 0xECF5F9
local muted = 0x7D97A6
local orange = 0xFFB15E
local provinces = {
    {"ADANA", 37.0000, 35.3213},
    {"ADIYAMAN", 37.7648, 38.2786},
    {"AFYONKARAHISAR", 38.7507, 30.5567},
    {"AGRI", 39.7191, 43.0506},
    {"AMASYA", 40.6539, 35.8330},
    {"ANKARA", 39.9334, 32.8597},
    {"ANTALYA", 36.8969, 30.7133},
    {"ARTVIN", 41.1828, 41.8183},
    {"AYDIN", 37.8560, 27.8416},
    {"BALIKESIR", 39.6484, 27.8826},
    {"BILECIK", 40.1500, 29.9833},
    {"BINGOL", 38.8854, 40.4980},
    {"BITLIS", 38.4006, 42.1095},
    {"BOLU", 40.7395, 31.6116},
    {"BURDUR", 37.7203, 30.2908},
    {"BURSA", 40.1885, 29.0610},
    {"CANAKKALE", 40.1553, 26.4142},
    {"CANKIRI", 40.6013, 33.6134},
    {"CORUM", 40.5506, 34.9556},
    {"DENIZLI", 37.7765, 29.0864},
    {"DIYARBAKIR", 37.9144, 40.2306},
    {"EDIRNE", 41.6771, 26.5557},
    {"ELAZIG", 38.6810, 39.2264},
    {"ERZINCAN", 39.7500, 39.5000},
    {"ERZURUM", 39.9055, 41.2658},
    {"ESKISEHIR", 39.7767, 30.5206},
    {"GAZIANTEP", 37.0662, 37.3833},
    {"GIRESUN", 40.9128, 38.3895},
    {"GUMUSHANE", 40.4603, 39.4814},
    {"HAKKARI", 37.5744, 43.7408},
    {"HATAY", 36.2021, 36.1600},
    {"ISPARTA", 37.7648, 30.5566},
    {"MERSIN", 36.8121, 34.6415},
    {"ISTANBUL", 41.0082, 28.9784},
    {"IZMIR", 38.4237, 27.1428},
    {"KARS", 40.6013, 43.0975},
    {"KASTAMONU", 41.3766, 33.7765},
    {"KAYSERI", 38.7312, 35.4787},
    {"KIRKLARELI", 41.7333, 27.2167},
    {"KIRSEHIR", 39.1458, 34.1639},
    {"KOCAELI", 40.8533, 29.8815},
    {"KONYA", 37.8746, 32.4932},
    {"KUTAHYA", 39.4167, 29.9833},
    {"MALATYA", 38.3552, 38.3095},
    {"MANISA", 38.6191, 27.4289},
    {"KAHRAMANMARAS", 37.5753, 36.9228},
    {"MARDIN", 37.3122, 40.7351},
    {"MUGLA", 37.2153, 28.3636},
    {"MUS", 38.9462, 41.7539},
    {"NEVSEHIR", 38.6244, 34.7239},
    {"NIGDE", 37.9698, 34.6766},
    {"ORDU", 40.9839, 37.8764},
    {"RIZE", 41.0201, 40.5234},
    {"SAKARYA", 40.6940, 30.4358},
    {"SAMSUN", 41.2867, 36.3300},
    {"SIIRT", 37.9333, 41.9500},
    {"SINOP", 42.0231, 35.1531},
    {"SIVAS", 39.7477, 37.0179},
    {"TEKIRDAG", 40.9780, 27.5110},
    {"TOKAT", 40.3167, 36.5500},
    {"TRABZON", 41.0027, 39.7168},
    {"TUNCELI", 39.1079, 39.5401},
    {"SANLIURFA", 37.1674, 38.7955},
    {"USAK", 38.6823, 29.4082},
    {"VAN", 38.5012, 43.3730},
    {"YOZGAT", 39.8181, 34.8147},
    {"ZONGULDAK", 41.4564, 31.7987},
    {"AKSARAY", 38.3687, 34.0370},
    {"BAYBURT", 40.2552, 40.2249},
    {"KARAMAN", 37.1811, 33.2150},
    {"KIRIKKALE", 39.8468, 33.5153},
    {"BATMAN", 37.8812, 41.1351},
    {"SIRNAK", 37.5164, 42.4611},
    {"BARTIN", 41.6344, 32.3375},
    {"ARDAHAN", 41.1105, 42.7022},
    {"IGDIR", 39.9237, 44.0450},
    {"YALOVA", 40.6500, 29.2667},
    {"KARABUK", 41.2061, 32.6204},
    {"KILIS", 36.7184, 37.1212},
    {"OSMANIYE", 37.0742, 36.2478},
    {"DUZCE", 40.8438, 31.1565}
}

local selected = 34
local result = nil
local result_province = 0
local message = "LOADING WEATHER"
local elapsed = 600
local refresh_pending = true

local function refresh_weather()
    local province = provinces[selected]
    local latest, err = rexa.weather(province[2], province[3])
    elapsed = 0
    refresh_pending = false
    if latest then
        result = latest
        result_province = selected
        message = ""
    else
        message = err or "WEATHER UNAVAILABLE"
    end
end

function update(input, dt)
    if input.direction == "up" then
        selected = (selected - 2) % #provinces + 1
        message = "CLICK TO LOAD THIS CITY"
    elseif input.direction == "down" then
        selected = selected % #provinces + 1
        message = "CLICK TO LOAD THIS CITY"
    end

    if input.pressed then
        refresh_pending = true
    end
    elapsed = elapsed + dt
    if refresh_pending or elapsed >= 600 then
        refresh_weather()
    end
end

function draw()
    local province = provinces[selected]
    rexa.clear(0x0B1220)
    rexa.text(14, 30, string.format("%02d / 81  %s",
              selected, province[1]), cyan, 1)
    rexa.text(14, 44, "TURKEY WEATHER", muted, 1)

    if result and result_province == selected then
        rexa.text(14, 66, string.format("%.1f C", result.temperature_c),
                  white, 3)
        rexa.text(170, 66, string.format("FEELS %.1f C",
                  result.apparent_temperature_c), muted, 1)
        rexa.text(170, 82, string.format("HUMIDITY %.0f%%",
                  result.relative_humidity_percent), muted, 1)
        rexa.text(170, 98, string.format("WIND %.1f KM/H",
                  result.wind_speed_kmh), muted, 1)
        rexa.text(170, 114, string.format("RAIN %.1f MM",
                  result.precipitation_mm), muted, 1)
    elseif message ~= "" then
        rexa.text(14, 80, string.sub(message, 1, 40), orange, 1)
    end

    rexa.text(14, 128, "UP/DOWN: CITY", cyan, 1)
    rexa.text(14, 144, "CLICK: GET / REFRESH", muted, 1)
end
