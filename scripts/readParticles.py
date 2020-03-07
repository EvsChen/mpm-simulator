import struct

filename = "./particles_0002.txt"
file = open(filename, mode="rb")
filedata = file.read()

# Total number of particles
num = struct.unpack("i", filedata[0: 4])[0]

# Positions of particles
positions = struct.unpack(str(num * 3) + "f", filedata[4:])
