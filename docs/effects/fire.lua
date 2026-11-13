-- fire.lua — пламя: heat-симуляция + палитра (C-примитивы)
-- RGB setup, затем: rgb_effect("fx:fire")
local COOL  = 55   -- 0..255: охлаждение (больше — гаснет быстрее)
local SPARK = 90   -- 0..255: вероятность искры в нижнем ряду за кадр
local PUFF  = 40   -- 0..255: вероятность бокового дрейфа ("дыхание")

function on_frame(fx)
  if fx.frame == 1 then
    fx.setpal{'#000000', '#3b0a00', '#8a2500', '#d45500', '#f08c00', '#ffc935', '#fff0a0', '#ffffff'}
  end
  local cols, rows = fx.cols, fx.rows
  for i = 0, fx.count-1 do
    local h = fx.heat(i) - math.random(0, COOL)
    if h < 0 then h = 0 end
    fx.heat_set(i, h)
  end
  for y = 1, rows-1 do
    for x = 0, cols-1 do
      local h1 = fx.heat((y-1)*cols + x)
      local h2 = (y >= 2) and fx.heat((y-2)*cols + x) or 0
      fx.heat_set(y*cols + x, (h1*2 + h2) // 3)
    end
  end
  for y = 0, rows-1 do
    for x = 0, cols-1 do
      if math.random(0, 255) < PUFF then
        local dx = x + (math.random(0,1)*2 - 1)
        if dx >= 0 and dx < cols then
          local a = (fx.heat(y*cols+x) + fx.heat(y*cols+dx)) // 2
          fx.heat_set(y*cols+x, a)
          fx.heat_set(y*cols+dx, a)
        end
      end
    end
  end
  for x = 0, cols-1 do
    if math.random(0, 255) < SPARK then fx.heat_set(x, 255) end
  end
  for y = 0, rows-1 do
    for x = 0, cols-1 do
      fx.pxg(x, y, fx.heat(y*cols+x) / 255)
    end
  end
end
