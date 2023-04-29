from crc import Calculator, Crc8

import numpy as np

def encode(speed,left,right, calc = Calculator(Crc8.CCITT,optimized = True)):
    msg = np.array(np.concatenate([(left << 1) | right,speed], axis=None), dtype='uint8')
    checksum = calc.checksum(msg.tobytes())
    msg = np.array(np.concatenate([msg,checksum], axis=None), dtype='uint8').tobytes()
    #print(speed)
    return msg
    


