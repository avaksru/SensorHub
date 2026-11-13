-- aurora.lua — северное сияние: слои value-noise + палитра
-- RGB setup, затем: rgb_effect("fx:aurora")
local SPD   = 1.0    -- скорость (0.1..3)
local SCALE = 0.55   -- крупность узора (0.2..1)

function on_frame(fx)
  if fx.frame == 1 then
    fx.setpal{'#010208', '#082c4f', '#0b5f77', '#1fa88f', '#7cf08a', '#e9ffd9'}
  end
  local t = fx.t * 0.00045 * SPD
  for y = 0, fx.rows-1 do
    local fy = (fx.rows > 1) and (y / (fx.rows-1)) or 0
    local top = 1 - fy
    for x = 0, fx.cols-1 do
      local v1 = fx.noise2(x * SCALE, y * 0.35 + t, t)
      local v2 = fx.noise2(x * SCALE * 2 + 40, y * 0.2 - t * 0.6, t * 1.7)
      local m  = (v1 * 190 + v2 * 65) / 255
      local h  = (m - 0.15) / 0.85
      if h < 0 then h = 0 end
      local sh = 0.28 + 0.72 * top * top
      local r, g, b = fx.grad(h)
      fx.px(x, y, r * sh, g * sh, b * sh)
    end
  end
end
