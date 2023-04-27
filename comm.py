from crc import Calculator, Crc8

import numpy as np

def motor_encode(speed,left,right, calc = Calculator(Crc8.CCITT,optimized = True)):
    prefix = 0x83
    direction = np.array([(left << 1) | right], dtype='uint8')
    print("Direction: " + str(direction))
    msg = np.array(np.concatenate([direction,speed], axis=None), dtype='uint8')
    print("Message to bytes: " + str(msg.tobytes().hex()))
    checksum = calc.checksum(msg.tobytes())
    #0x80 last bit of the polynomial
    msg = np.array(np.concatenate([prefix,direction+0x80,speed,checksum], axis=None), dtype='uint8').tobytes()
    return msg
    



