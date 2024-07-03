-- This script is Altitude-Based Speed Limit in LOITER and AUTO Mode.
-- Limit the drone's flight speed for safety when its altitude is low.

local altitude_limit = 10

local LOIT_SPEED_DEFAULT  = 1250
local LOIT_SPEED_LIMIT  = 100

local WPNAV_SPEED_DEFAULT = 1000
local WPNAV_SPEED_LIMIT = 100

function update() -- this is the loop which periodically runs
  local loit_speed  = LOIT_SPEED_LIMIT
  local wpnav_speed = WPNAV_SPEED_LIMIT

  local home = ahrs:get_home()
  local curr_loc = ahrs:get_position()

  if curr_loc then 
    local vec_from_home = home:get_distance_NED(curr_loc)
    -- gcs:send_text(0, "Altitude above home: " .. tostring(math.floor(-vec_from_home:z())))

    if -vec_from_home:z() < altitude_limit then
      loit_speed = LOIT_SPEED_LIMIT
      wpnav_speed = WPNAV_SPEED_LIMIT
    else
      loit_speed = LOIT_SPEED_DEFAULT
      wpnav_speed = WPNAV_SPEED_DEFAULT
    end
  end

  local curr_loit_speed = param:get('LOIT_SPEED')
  if curr_loit_speed then
    if curr_loit_speed ~= loit_speed then
      if (param:set('LOIT_SPEED', loit_speed)) then
        gcs:send_text(0, 'LOIT_SPEED: ' .. loit_speed)
      else
        gcs:send_text(0, 'LUA: LOIT_SPEED set failed')
      end
    end
  else
      gcs:send_text(0, 'LUA: LOIT_SPEED get failed')
  end

  local curr_wpnav_speed = param:get('WPNAV_SPEED')
  if curr_wpnav_speed then
    if curr_wpnav_speed ~= wpnav_speed then
      if (param:set('WPNAV_SPEED', wpnav_speed)) then
      gcs:send_text(0, 'WPNAV_SPEED: ' .. wpnav_speed)
      else
        gcs:send_text(0, 'LUA: WPNAV_SPEED set failed')
      end
    end
  else
    gcs:send_text(0, 'LUA: WPNAV_SPEED get failed')
  end
  
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
