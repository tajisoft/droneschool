-- This script checks Proximity

function update()
  sensor_count = proximity:num_sensors()
  -- gcs:send_text(0, string.format("%d proximity sensors found.", sensor_count))
  local isNear = false
  if sensor_count > 0 then
    object_count = proximity:get_object_count()
    -- gcs:send_text(0, string.format("%d objects found.", object_count))

    closest_angle, closest_distance = proximity:get_closest_object()
    if closest_angle and closest_distance then
      -- gcs:send_text(0, "Closest object at angle "..closest_angle.." distance "..closest_distance)
      if closest_distance < 0.5 then
        gcs:send_text(0, "Warning: Closest object is less than 40 cm away")
        isNear = true
      end
      if isNear then
        vehicle:set_mode(4) -- 4 is the mode number for HOLD
      end
      if not isNear and vehicle:get_mode() == 4 then
        vehicle:set_mode(0) -- 0 is the mode number for MANUAL
      end
    end

    for i = 0, object_count do
      angle, distance = proximity:get_object_angle_and_distance(i)
      if angle and distance then
        -- gcs:send_text(0, "Object "..i.." at angle "..angle.." distance "..distance)
      end
    end
  end

  return update, 500 -- check again in 0.5Hz
end

return update(), 500 -- first message may be displayed 0.5 seconds after start-up
