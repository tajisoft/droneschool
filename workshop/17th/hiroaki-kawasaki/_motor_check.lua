-- Lua script to write and read from a serial

---@diagnostic disable: need-check-nil

local port = serial:find_serial(0)

port:begin(115200)
port:set_flow_control(0)

local rpm

-- ４バイト一組のデータを受信する
-- 0: 必ず0xff
-- 1: 16ビット整数データのLOW8bit
-- 2: 16ビット整数データのHIGH8bit
-- 3: 1と2の8bitの排他的論理和


-- データを読み込む関数
function spit ()
  while port:available() > 0 do
    if port:available() >= 4 then  -- 必要なバイト数があるか確認

      -- 各バイトを整数に変換
      local byte0 = port:read()
      local byte1 = port:read()
      local byte2 = port:read()
      local byte3 = port:read()

      -- 最初のバイトが0xffであることを確認
      if byte0 == 0xff then
        -- 16ビット整数データを組み立てる
        rpm = byte1 + (byte2 * 256)  -- LOW8bitとHIGH8bitから16ビット整数を構築

        -- 排他的論理和をチェック
        if byte3 == (byte1 ~ byte2) then
          -- 正しいデータが得られた場合、GCSに送信
          gcs:send_text(0, "Received 16-bit data: " .. rpm)
        else
          -- データエラー時の処理
          gcs:send_text(0, "Data error: XOR mismatch")
        end
      else
        -- フォーマットエラー時、最初のバイトを読み捨て
        port:read()
        gcs:send_text(0, "Data error: First byte is not 0xff, discarding byte")
      end
    else
      -- データが不足している場合の処理
      gcs:send_text(0, "Insufficient data available")
    end
  end
  -- port:write(rpm)
  return spit, 1000
end

return spit, 1000
