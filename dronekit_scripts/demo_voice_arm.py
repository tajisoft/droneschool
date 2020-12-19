import pyaudio
import numpy as np
from dronekit import connect
import time

seconds = 20
CHUNK = 2**11
RATE = 44100
threshold=16
tkeep = 5

# armed callback
def armed_callback(self, name, val):
    print("Vehicle is Armed!")
def check_callback(self, name, val):
    print("Parameter %s = %s" % (name, val))

vehicle = connect("/dev/ttyS3", wait_ready=True);
print("Version %s" % vehicle.version)

# disable arming check
vehicle.parameters['ARMING_CHECK'] = 0

vehicle.add_attribute_listener('location.armed', armed_callback)
vehicle.parameters.add_attribute_listener('ARMING_CHECK', check_callback)

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input = True,
                frames_per_buffer = CHUNK)

for i in range(int(seconds * 44100 / 1024)):
    data = np.fromstring(stream.read(CHUNK), dtype = np.int16)
    peak = np.average(np.abs(data)) * 2
    cnt = int(50 * peak / 2 ** 16)
    print(cnt)
    bars = "#" * cnt
    # more safety
    #if (cnt > threshold and vehicle.is_armable and not vehicle.armed):
    if (cnt > threshold and not vehicle.armed):
        tkeep -= 1
    print("%04d %05d %s" % (i, peak, bars))
    if (tkeep == 0 and not vehicle.armed):
        vehicle.armed = True
        tkeep = 5

stream.stop_stream()
stream.close()
p.terminate()
vehicle.remove_attribute_listener('location.armed', armed_callback)
vehicle.parameters.remove_attribute_listener('ARMING_CHECK', check_callback)
