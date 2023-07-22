# encoding: utf-8
import math
import time

from dronekit import connect, LocationGlobalRelative

vehicles = None
wait_ready = False

def heartbeat_handler(self, name, val):
    self.my_type = val.type
    log('My type is {}'.format(self.my_type))
    self.remove_message_listener('HEARTBEAT', heartbeat_handler)

def prepare_vehicles():
    plane = connect('127.0.0.1:14551', wait_ready=wait_ready, timeout=60)
    log('Plane connected')
    copter = connect('127.0.0.1:14561', wait_ready=wait_ready, timeout=60)
    log('Copter connected')
    boat = connect('127.0.0.1:14571', wait_ready=wait_ready, timeout=60)
    log('Boat connected')
    rover1 = connect('127.0.0.1:14581', wait_ready=wait_ready, timeout=60)
    log('Rover1 connected')
    rover2 = connect('127.0.0.1:14591', wait_ready=wait_ready, timeout=60)
    log('Rover2 connected')
    log('All vehicles were connected.')
    plane.add_message_listener('HEARTBEAT', heartbeat_handler)
    copter.add_message_listener('HEARTBEAT', heartbeat_handler)
    boat.add_message_listener('HEARTBEAT', heartbeat_handler)
    rover1.add_message_listener('HEARTBEAT', heartbeat_handler)
    rover2.add_message_listener('HEARTBEAT', heartbeat_handler)
    log('Wait all vehicles are armable.')
    plane.wait_for_armable()
    copter.wait_for_armable()
    boat.wait_for_armable()
    rover1.wait_for_armable()
    rover2.wait_for_armable()
    all_vehicles = [plane, copter, boat, rover1, rover2]

    while True:
        is_ok = True
        for v in all_vehicles:
            if not hasattr(v, 'my_type'):
                log('Wait heartbeat')
                is_ok = False
                time.sleep(1)
                break
        if is_ok:
            break

    # two rover
    all_vehicles[3].my_type = 100
    all_vehicles[4].my_type = 101
    return all_vehicles

    
class ControlTower:
    targets = None
    is_complete = False
    routes = {
        1: { # QuadPlane
            'from': [35.760215, 140.379330, 0],
            'to': [35.878275, 140.338069, 100],
            'wait': None,
            'instance': None
        },
        2: { # Copter
            'from': [35.878275, 140.338069, 0],
            'to': [35.867003, 140.305987, 50],
            'wait': 1,
            'instance': None
        },
        11: { # Boat
            'from': [35.878275, 140.338069, 0],
            'to': [35.879768, 140.348495, 0],
            'wait': 1,
            'instance': None
        },
        100: {
            'from': [35.879768, 140.348495, 0],
            'to': [35.876991, 140.348026, 0],
            'wait': 11,
            'instance': None
        },
        101: {
            'from': [35.867003, 140.305987, 0],
            'to': [35.877518, 140.295439, 0],
            'wait': 2,
            'instance': None
        }
    }

    def __init__(self, vehicles):
        self.targets = vehicles
        for v in self.targets:
            my_route = self.routes[v.my_type]
            my_route['instance'] = v
    
    def _launch_vehicle(self, vehicle):
        log('Will launch {}'.format(vehicle.my_type))
        if vehicle.my_type == 2 or vehicle.my_type == 1:
            vehicle.wait_for_mode('GUIDED')
            if not vehicle.armed:
                vehicle.arm()
            if vehicle.location.global_frame.alt < 10:
                vehicle.wait_simple_takeoff(20)
        elif vehicle.my_type == 10 or vehicle.my_type == 11 or vehicle.my_type == 100 or vehicle.my_type == 101:
            vehicle.wait_for_mode('GUIDED')
            if not vehicle.armed:
                vehicle.arm()
    
    def _fly_to(self, vehicle, dest):
        log('{} will fly to {}'.format(vehicle.my_type, dest))
        target = LocationGlobalRelative(dest[0], dest[1], dest[2])
        vehicle.simple_goto(target)
    
    def _is_arrived(self, vehicle, dest):
        target = LocationGlobalRelative(dest[0], dest[1], dest[2])
        dlat = vehicle.location.global_frame.lat - target.lat
        dlong = vehicle.location.global_frame.lon - target.lon
        dist = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        log('Arrived check for {} dist {}'.format(vehicle.my_type, dist))
        # Plane
        if vehicle.my_type == 1:
            return dist <= 150
        else:
            return dist <= 10
    
    def _complete(self, vehicles):
        complete = True
        for v in vehicles:
            complete = self._is_arrived(v, self.routes[v.my_type]['to'])
        return complete
            
    def start(self):
        while not self.is_complete:
            log('==TowerLoop==')
            for v in vehicles:
                log('check vehicle {}'.format(v.my_type))
                my_route = self.routes[v.my_type]
                if not my_route['wait'] and not hasattr(my_route['instance'], 'status'):
                    self._launch_vehicle(v)
                    self._fly_to(v, my_route['to'])
                    my_route['instance'].status = 'OK'
                else:
                    # Already done
                    if not my_route['wait'] or (hasattr(v, 'status') and v.status == 'OK'):
                        continue

                    wait_target = self.routes[my_route['wait']]['instance']
                    if self._is_arrived(wait_target, my_route['from']):
                        log('{} arrived.'.format(wait_target.my_type))
                        wait_target.status = 'OK'
                        if wait_target.my_type == 1:
                            wait_target.wait_for_mode('QLAND')
                        elif wait_target.my_type == 2:
                            wait_target.wait_for_mode('LAND')
                        self._launch_vehicle(v)
                        self._fly_to(v, my_route['to'])
                    else:
                        log('Wait for {}'.format(self.routes[my_route['wait']]['instance'].my_type))
            if self._complete(vehicles=vehicles):
                break
            time.sleep(5)
    
    def cleanup(self, vehicles):
        for v in vehicles:
            v.close()
        log('Clean up done.')

def log(msg):
    print('log: {}'.format(msg))

if __name__ == '__main__':
    log('Started!')
    vehicles = prepare_vehicles()
    control = ControlTower(vehicles=vehicles)
    control.start()
    control.cleanup(vehicles=vehicles)
    log('Finished!')
