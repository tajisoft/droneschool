-- RC8スイッチがONの時現在のモードを監視してALT_HOLDの時はALT_HOLD_SIMPLE(100)に変更する
-- 同様にLOITERの時はLOITER_SUPERSIMPLE(101)に変更する
-- 坂本案１

local MODE_ALT_HOLD = 2
local MODE_LOITER = 5
local MODE_ALT_HOLD_SIMPLE = 100
local MODE_LOITER_SUPERSIMPLE = 101
local prior_mode = -1
local prior_mode_name = "NONE"

-- the main update function
function update()
    pwm8 = rc:get_pwm(8)
    if pwm8 and pwm8 > 1500 then
      -- RC 8 がONの時のみ有効化
      local mode = vehicle:get_mode()                   -- get current mode
      if mode == MODE_ALT_HOLD then
        -- ALT_HOLDならALT_HOLD_SIMPLEにモード変更する
        gcs:send_text(MAV_SEVERITY.INFO, "Lua : ALT_HOLD -> ALT_HOLD_SIMPLE")
        prior_mode = mode
        prior_mode_name = "ALT_HOLD"
        vehicle:set_mode(MODE_ALT_HOLD_SIMPLE)
      else if mode == MODE_LOITER then
        -- LOITERならLOITER_SUPERSIMPLEにモード変更する
        gcs:send_text(MAV_SEVERITY.INFO, "Lua : LOITER -> LOITER_SUPERSIMPLE")
        prior_mode = mode
        prior_mode_name = "LOITER"
        vehicle:set_mode(MODE_LOITER_SUPERSIMPLE)
      end
    else if prior_mode > 0 then
      -- RC 8 がOFFの時切り替え済みの場合は切り替え前のモードに戻す
      gcs:send_text(MAV_SEVERITY.INFO, "Lua : RESET to %s",prior_mode_name)
      vehicle:set_mode(prior_mode)
      prior_mode = -1
    end

  return update, 1000
end

return update()
