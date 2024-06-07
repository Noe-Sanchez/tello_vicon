from djitellopy import Tello

# Connect via ip
tello = Tello('192.168.0.148', 8889)

tello.connect()

#Get battery percentage
battery = tello.get_battery()

print(battery)

tello.end()


