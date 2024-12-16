import math

FACTOR = 111320.0

def get_center_lat_lon(lat, lon, x, y):
  lat_to_meter = FACTOR
  lon_to_meter = FACTOR * math.cos(math.radians(lat))

  delta_lat = y / lat_to_meter
  delta_lon = x / lon_to_meter

  new_lat = lat + delta_lat
  new_lon = lon + delta_lon

  return new_lat, new_lon

def create_circle(
  center_lat,
  center_lon,
  radius,
  altitute,
  points=36
):
  waypoints = []
  for i in range(points):
    angle = 2 * math.pi * i / points
    dlat = radius * math.cos(angle) / FACTOR
    dlon = radius * math.sin(angle) / (FACTOR * math.cos(math.radians(center_lat)))
    waypoints.append((center_lat + dlat, center_lon + dlon, altitute))
  return waypoints