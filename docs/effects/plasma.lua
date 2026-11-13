-- plasma.lua — классическая плазма на синусах
-- RGB setup, затем: rgb_effect("fx:plasma")
local SPD = 1.0

function on_frame(fx)
  if fx.frame == 1 then
    fx.setpal{'#120026', '#3b0f70', '#8a2be2', '#ff69b4', '#ffb347', '#fff7d6'}
  end
  local t = fx.t * 0.002 * SPD
  for y = 0, fx.rows-1 do
    for x = 0, fx.cols-1 do
      local v = 0
        + math.sin(x * 0.7 + t)
        + math.sin((y * 0.5 - t) * 1.3)
        + math.sin((x + y) * 0.4 - t * 0.7)
        + math.sin(math.sqrt(x*x + y*y) * 0.5 - t * 1.1)
      fx.pxg(x, y, (v + 4) / 8)
    end
  end
end
