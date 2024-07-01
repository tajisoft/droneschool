-- Lua script to write and read from a serial

---@diagnostic disable: need-check-nil

local port = serial:find_serial(0)
local MODE_ACRO = 1
local OUTPUT_PWM = 1000 * 0.07 + 1000
local EXP_RPM = 1600
local OK_RANGE = 0.2

local tries = 0
local motor = -1

port:begin(115200)
port:set_flow_control(0)

local discard_bytes = 0
while port:available() > 0 do
  local byte = port:read()
  discard_bytes = discard_bytes + 1
end
gcs:send_text(0, ">> Discarded serial buffer: " .. discard_bytes)

local ok_count = 0
local last_mode = 0

-- ４バイト一組のデータを受信する
-- 0: 必ず0xff
-- 1: 16ビット整数データのLOW8bit
-- 2: 16ビット整数データのHIGH8bit
-- 3: 1と2の8bitの排他的論理和

function get_rpm()
  while port:available() > 0 do
    if port:available() >= 4 then  -- 必要なバイト数があるか確認

      -- 各バイトを整数に変換
      local byte0 = port:read()

      -- 最初のバイトが0xffであることを確認
      if byte0 == 0xff then
        local byte1 = port:read()
        local byte2 = port:read()
        local byte3 = port:read()

        -- 16ビット整数データを組み立てる
        local rpm = byte1 + (byte2 * 256)  -- LOW8bitとHIGH8bitから16ビット整数を構築

        -- 排他的論理和をチェック
        if byte3 == (byte1 ~ byte2) then
          -- 正しいデータを取得
          return rpm
        else
          -- データエラー時の処理
          gcs:send_text(0, "Data error: XOR mismatch")
        end
      else
        -- フォーマットエラー時、読み捨て
        gcs:send_text(0, "Data error: First byte is not 0xff, discarding byte")
      end
    else
      -- データが不足している場合の処理
      gcs:send_text(0, "Insufficient data available")
    end
  end
  return -1
end

-- データを読み込む関数
function task ()
  local current_mode = vehicle:get_mode()
  
  if last_mode ~= MODE_ACRO and current_mode == MODE_ACRO then
    if  arming:is_armed() then
      gcs:send_text(0, "*** ERROR: Cannot be performed when already armed")
    else
      motor = 0
      tries = 0
    end
  end
  last_mode = current_mode

  if motor >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, OUTPUT_PWM, 1100)
    SRV_Channels:set_output_pwm_chan_timeout(motor, OUTPUT_PWM, 1100)
    tries = tries + 1
    if tries > 15 then
      gcs:send_text(0, "*** MTR" .. motor .. ": NG")
      motor = -1
    end
  end

  local rpm = get_rpm()
  if rpm >= 0 and motor >= 0 then      
    gcs:send_text(0, "Motor rpm: " .. rpm)
    if rpm > EXP_RPM * (1 - OK_RANGE) and rpm < EXP_RPM * (1 + OK_RANGE) then
      ok_count = ok_count + 1
      if ok_count >= 3 then 
        gcs:send_text(0, ">>> MTR" .. motor .. ": OK")
        tries = 0
        ok_count = 0
        motor = motor + 1
        if motor >= 4 then
          gcs:send_text(0, ">>> arming...")
          motor = -1
          arming:arm()
        end
      end
    else
      ok_count = 0
    end
  end

  return task, 500
end

return task, 500
