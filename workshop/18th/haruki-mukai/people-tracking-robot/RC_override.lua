-- example of overriding RC inputs

---@diagnostic disable: need-check-nil
---@diagnostic disable: param-type-mismatch

local RC5 = rc:get_channel(5)
local RC7 = rc:get_channel(7)

function update()
   -- mirror RC1 onto RC4
   rc1_input = rc:get_pwm(1)
   rc2_input = rc:get_pwm(2)
   rc3_input = rc:get_pwm(3)
   rc4_input = rc:get_pwm(4)
   
   if(rc1_input < 1000 or  rc3_input < 1000) then
      rc5_input_data = rc2_input
      rc7_input_data = rc4_input
   elseif(rc2_input < 1000 or rc4_input < 1000) then
      rc5_input_data = rc1_input
      rc7_input_data = rc3_input
   elseif(rc1_input > 1520 or rc1_input < 1480 or rc3_input > 1520 or rc3_input < 1480) then
      rc5_input_data = rc1_input
      rc7_input_data = rc3_input
   else
      rc5_input_data = rc2_input
      rc7_input_data = rc4_input
   end
   RC5:set_override(rc5_input_data)
   RC7:set_override(rc7_input_data)
   return update, 10
end

gcs:send_text(0, "RC_override example")

return update()
