import struct
import smbus

i2c_bus=smbus.SMBus(1)

def val(freq):
    if freq==0:
        return freq
    data= int((32768/(2*freq)))
    if data>65535:
        return int(0)
    return data

def pack(data):
    data=[ord(c) for c in struct.pack("!H",data)]
    return data

def pack4(data):
    data=[ord(c) for c in struct.pack('!i', data)]
    return data

def send(addr, cmd, data):
    i2c_bus.write_i2c_block_data(addr, cmd, data)

