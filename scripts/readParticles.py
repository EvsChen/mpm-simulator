import struct

# filename = "./particles_0002.txt"
# file = open(filename, mode="rb")
# filedata = file.read()

# # Total number of particles
# num = struct.unpack("i", filedata[0: 4])[0]

# # Positions of particles
# positions = struct.unpack(str(num * 3) + "f", filedata[4:])

filename = "./v_0003.bin"
file = open(filename, mode="rb")
filedata = file.read()

# Total number of particles
res = struct.unpack("3i", filedata[0: 12])
print(res)
spacing = struct.unpack("f", filedata[12: 16])[0]
print(spacing)
num = res[0] * res[1] * res[2]
# Velocity of grid
velocity = struct.unpack(str(num * 3) + "f", filedata[16:])
vx = velocity[0::3]
vy = velocity[1::3]
vz = velocity[2::3]

for i in range(len(velocity)):
    if velocity[i] > 0:
        print(i)