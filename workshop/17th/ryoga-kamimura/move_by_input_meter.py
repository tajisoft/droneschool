import time
from math import radians, degrees, sin, cos, asin, sqrt

from dronekit import connect, VehicleMode, LocationGlobalRelative


#ドローンとの接続アドレス
CONNECTION_STRING = 'tcp:127.0.0.1:5762'
#到着判定の閾値[m]
ARRIVE_THRESHOLD = 1
#地球の半径[m]
EARTH_RADIUS_METERS = 6378137.0


def body_to_NED(distance_x, distance_y, vehicle):
    """
    指定距離と、機体の方位角からNED座標系への変換を行う
    """
    heading_rad = radians(vehicle.heading)
    dNorth = distance_x * cos(heading_rad) - distance_y * sin(heading_rad)
    dEast = distance_x * sin(heading_rad) + distance_y * cos(heading_rad)
    return dNorth, dEast


def get_location_meter(dNorth, dEast, vehicle):
    """
    現在地点から特定の距離だけオフセットされた地点を算出する
    """
    current_location = vehicle.location.global_relative_frame

    #距離を緯度経度に変換
    dLat = dNorth / EARTH_RADIUS_METERS
    dLon = dEast / (EARTH_RADIUS_METERS * cos(radians(current_location.lat)))

    #新しい地点の緯度経度を算出
    new_lat = current_location.lat + degrees(dLat)
    new_lon = current_location.lon + degrees(dLon)

    return LocationGlobalRelative(new_lat, new_lon, current_location.alt)


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    haversine公式を使って、二つの緯度経度間の距離を計算する
    """
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)

    a = (sin(dlat/2) ** 2 +
         cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2) ** 2)
    c = 2 * asin(sqrt(a))

    distance = EARTH_RADIUS_METERS * c
    return distance


def get_distance_meters(location1, location2):
    """
    2つのLocationGlobalオブジェクトから、距離を算出する
    """
    return haversine_distance(location1.lat, location1.lon, location2.lat, location2.lon)


def has_arrived(current_location, target_location):
    """
    機体が目的地に到着したかを判定する
    """
    distance = get_distance_meters(current_location, target_location)
    return distance <= ARRIVE_THRESHOLD


def main():
    #機体と接続
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    #アーム可能になるまで待機
    while not vehicle.is_armable:
        print('Waiting for vehicle to become armable')
        time.sleep(1)

    current_location = vehicle.location.global_relative_frame
    print(current_location)

    try:
        while True:
            print("\n-----------------------------------------------------------------------------------------")
            #移動先をx,y方向で入力(xが進行方向に対して右向き正、yが進行方向に対して正面が正)
            input_str = input("Please input moving point in x,y format (meter) (type 'quit' to finish):")
            
            #入力チェック
            if input_str.lower() == 'quit':
                break
            
            #入力文字列の処理
            try:
                distance_x, distance_y = map(float, input_str.split(','))
                
                #入力された距離を機体進行方向に合わせて変換
                dNorth, dEast = body_to_NED(distance_x, distance_y, vehicle)

                #移動先の緯度経度を算出する
                target_location = get_location_meter(dNorth, dEast, vehicle)
            except ValueError:
                raise ValueError("Input must be in x,y format and represent valid distance")

            #移動開始
            #GUIDEDモードに変更
            if vehicle.mode != VehicleMode('GUIDED'):
                vehicle.mode = VehicleMode('GUIDED')
                
                #モードが変わるまで待つ
                while vehicle.mode != VehicleMode('GUIDED'):
                    print(f'Changing mode to GUIDED')
                    time.sleep(1)

            #simle_goto()で入力された位置に移動
            vehicle.simple_goto(target_location)

            #指定位置にたどり着くまで待機
            while True:
                current_location = vehicle.location.global_frame
                if has_arrived(current_location, target_location):
                    break

                #到着してない場合はループを継続
                distance = get_distance_meters(current_location, target_location)
                print(f'Distance to target location: {distance:.2f} m')
                time.sleep(1)
            
            #到着した後にAUTOモードに変更
            if vehicle.mode != VehicleMode('AUTO'):
                vehicle.mode = VehicleMode('AUTO')
                
            #モードが変わるまで待つ
            while vehicle.mode != VehicleMode('AUTO'):
                print(f'Changing mode to AUTO')
                time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nScript interrupted by the user")

    finally:
        vehicle.close()                


if __name__ == '__main__':
    main()


            
            
