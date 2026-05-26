#!/usr/bin/env python3
"""
StecaRS485protocol.py — very early and poor implementation of my attempt to mimic the RS485 protocol for StecaGrid 3600

For refrence only, don't use.
"""

import serial
import time
import binascii
import struct
from ctypes import c_ushort
import datetime

DEBUG = False

# set PLAYBACK to True to process included recorded data
PLAYBACK = False 

bauds = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800];

if __name__ == '__main__':
    # https://pythonhosted.org/pyserial/pyserial_api.html
    ser = serial.Serial(    
        port='/dev/ttyS0',
        baudrate=38400,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.3
    ) 
    ser.flush()
    br = 0;

#settings = ser.get_settings()
#print(settings)

def dump_bytes(formatted_hex_bytes, printable):
    print("# ",formatted_hex_bytes)
    print("# ",printable)
    print()

def process_telegram(t):
    print("# Parsed =", process_steca485(t))
    print()
     
def decode_stecaFloat_a(ac_bytes):
    if ac_bytes[0] == 0x0B:
        unit = "W"
    elif ac_bytes[0] == 0x07:
        unit = "A"
    elif ac_bytes[0] == 0x05:
        unit = "V"
    elif ac_bytes[0] == 0x0D:
        unit = "Hz"
    elif ac_bytes[0] == 0x09:
        unit = "Wh"
    elif ac_bytes[0] == 0x00:
        unit = "NUL"
    else:
        unit = f'0x{ac_bytes[0]:02x}'
   
    iacpower = ((ac_bytes[3] << 8 | ac_bytes[1]) << 8 | ac_bytes[2]) << 7 # formula to float - conversion according to Steca
    facpower, = struct.unpack('f', struct.pack('I', iacpower))

    if DEBUG:
        print("# b:", format_hex_bytes(ac_bytes))
        print("# i: 0x%0X" % iacpower,"=", str(iacpower))
        print("# f:", facpower)

    return [facpower, unit]

def decode_stecaFloat(in_bytes):
    results = decode_stecaFloat_a(in_bytes)
    return f"{results[0]:0.2f} {results[1]}"

def decode_TotalYield_a(ba):
    #five byte array, 
    bits = ba[3] << 24 | ba[2] << 16 | ba[1] << 8 | ba[0]
    ieee , = struct.unpack('f', struct.pack('I', bits))
    return [ieee, "Wh"]

# Grid voltage = L1 MeasurementValues ENS1 (measurement 1/2, value 1/4)
# Grid power = AC Power
# Grid frequency = L1 MeasurementValues ENS1 (measurement 2/2, value 2/4)
# Panel voltage 
# Panel current 
# Panel power 
# Daily yield 
# Total yield = ?
# Time

def process_steca485(t):
    """
    parse telegram from StecaGrid RS485 protocol
    
    returns an array
        msg group
        msg topic
        clear text topic
        values
        or payload has hex string

    :param str telegram:
    """    
    if is_one_full_telegram(t):
        results = [t[4], t[5], t[7], t[11]]
        total_length = (t[2] << 8 | t[3])
        if DEBUG:
            print("#",format_hex_bytes(t))
            print("# dgram:","",end="")
#            print("# ",t[4:-1])
#            print("start:",t[0]," ",end="")
            print("to:",t[4]," ",end="")
            print("from:",t[5]," ",end="")
            print("len:",total_length," ",end="")
            print(f"crc1: {t[6]:02x}"," ",end="")
            print(f"crc2: {t[-3]:02x}{t[-2]:02x}"," ",end="")
            print() #        print("stop:",t[-1])
            # Payload started 7
            print("# payload:", format_hex_bytes(t[7:-3]) ,"",end="")
            print("  ", format_printable(t[7:-3]))
        if t[7] == 0x40: # 64: Requests
            topic=""
            if t[11] == 0x1d: # 29:
                topic = " (Nominal Power)"
            elif t[11] == 0x22: # 34:
                topic = " (Panel Power)"
            elif t[11] == 0x23: # 35: 
                topic = " (Panel Voltage)"
            elif t[11] == 0x24: # 36:
                topic = " (Panel Current)"
            elif t[11] == 0x29: # 41:
                topic = " (ACPower)"
            elif t[11] == 0x3c: # 60:
                topic = " (Daily Yield)"
            if DEBUG:
                print(f"# RequestA for 0x{t[11]:02x}{topic} from {t[4]}")
        elif t[7] == 0x41: # 65: Responses
            if t[8] == 0x00:
                len = (t[9] << 8 | t[10])
                if DEBUG:
                    print(f"# ReponseA for 0x{t[11]:02x} from {t[4]} len={len}")
                if t[11] == 0x51: # 81: Label Value Value Value Value byte Label Value Value Value Value byte
                    i_labelA = 15
                    i_valA1 = i_labelA + (t[i_labelA-2] << 8 | t[i_labelA-1])
                    i_valA2 = i_valA1+4
                    i_valA3 = i_valA2+4
                    i_valA4 = i_valA3+4
                    #print(i_labelA,t[i_labelA-2],t[i_labelA-1],i_valA1,i_valA2,i_valA3,i_valA4)
                    i_labelB = i_valA4+4+1+2
                    i_valB1 = i_labelB + (t[i_labelB-2] << 8 | t[i_labelB-1])
                    i_valB2 = i_valB1+4
                    i_valB3 = i_valB2+4
                    i_valB4 = i_valB3+4                    
                    #print(i_labelB,t[i_labelB-2],t[i_labelB-1],i_valB1,i_valB2,i_valB3,i_valB4)
                    #label = t[15:15+t[14]]
                    if DEBUG:
                        print("#", str(t[i_labelA:i_valA1]), 
                            decode_stecaFloat(t[i_valA1:i_valA2]), 
                            decode_stecaFloat(t[i_valA2:i_valA3]), 
                            decode_stecaFloat(t[i_valA3:i_valA4]), 
                            decode_stecaFloat(t[i_valA4:i_valA4+4])) 
                        print("#", str(t[i_labelB:i_valB1]), 
                            decode_stecaFloat(t[i_valB1:i_valB2]), 
                            decode_stecaFloat(t[i_valB2:i_valB3]), 
                            decode_stecaFloat(t[i_valB3:i_valB4]), 
                            decode_stecaFloat(t[i_valB4:i_valB4+4])) 
                elif t[11] == 0x3c: # 60:
                    label = "Daily Yield"
                    val = decode_stecaFloat_a(t[12:16])
                    results += [label, val]
                    if DEBUG:
                        print("#", label, val[0], val[1])                   
                else:
                    label = t[15:15+t[14]].decode("ascii")
                    val = decode_stecaFloat_a(t[15+t[14]:15+t[14]+5])
                    results += [label, val]
                    if DEBUG:
                        print("#", label, val[0], val[1])
        elif t[7] == 0x64: # 100: Requests
            if DEBUG:
                print(f"# RequestB for 0x{t[11]:02x} from {t[4]}")
        elif t[7] == 0x65: # 101: Responses 
            if DEBUG:
                print(f"# ReponseB for 0x{t[11]:02x} from {t[4]}")
            if t[11] == 0xF1: #  ???
                results += ["Total Yield", decode_TotalYield_a(t[12:16])]
                if DEBUG:
                    print("# (",format_hex_bytes(t[12:17]),")")
                    print("#", decode_TotalYield_a(t[12:16]))
            elif t[11] == 0x05: # 5: Time 
                time = datetime.datetime(2000+t[12], t[13], t[14], t[15], t[16], t[17]) # ignoring final 3 byte for now. TZ, millis, ...?
                results += ["Time", time]
                if DEBUG:
                    print(f"# {time} (",format_hex_bytes(t[12:21]),")")
            elif t[11] == 0x08: # 8: ???
                results += ["???", format_hex_bytes(t[12:17])]
                if DEBUG:
                    print("# (",format_hex_bytes(t[12:17]),")")
                    print("#", decode_TotalYield_a(t[12:16]))
            elif t[11] == 0x09: # 9: Serial
                results += ["Serial Number", t[12:-4].decode("ascii")]
                if DEBUG:
                    print("# (",format_hex_bytes(t[12:17]),")")
            else:
                results += ["???", format_hex_bytes(t[12:17])]
        elif t[7] == 0x21: # Responses 
            if t[8] == 0x00:
                len = (t[9] << 8 | t[10])
                results += ["???", format_hex_bytes(t[12:17])]
                if DEBUG:
                    print("# ReponseC for", t[11], "from", t[4], "len=",len)
        return results
    else:
        if DEBUG:
            print("# NOT a single full Steca485 Telegram")
            
def format_hex_bytes(b):
    formatted_hex_bytes = ''
    for byte in b:
        hex_byte = f'{byte:02x}'
        formatted_hex_bytes += f'{hex_byte:>2} '
    return formatted_hex_bytes.strip()

def format_printable(b):
    printable = ''
    for byte in b:
        if not 32 <= byte <= 126:
            printable += '.'
        else:
            printable += chr(byte)
    return printable

def xprocess_telegram(t):
    formatted_hex_bytes = format_hex_bytes(t)
    printable = format_printable(t)
    print(f'hx += bytes.fromhex("{formatted_hex_bytes}") # {printable} ')

def split_byte_array(byte_array):
    sub_arrays = []
    start_index = 0
    for i in range(len(byte_array)):
        if byte_array[i] == 0x03 and (i + 1 < len(byte_array) and byte_array[i + 1] == 0x02):
            sub_arrays.append(byte_array[start_index:i+1])
            start_index = i + 1
            break
    if start_index < len(byte_array):
        sub_arrays.append(byte_array[start_index:])
    return sub_arrays

def is_one_full_telegram(t):
    if t[0] != 2:
        #print("not starting w/ 0x02")
        return False
    if t[len(t)-1] != 3:
        #print("not ending w/ 0x03")
        return False
    if len(t) != (t[2] << 8 | t[3]):
        #print("wrong length",len(t), "!=", (t[2] << 8 | t[3]))
        return False
    return True
    
def process_telegrams(t):
    if len(t) == 0:
        return b''
    sub_arrays = split_byte_array(t)
    if len(sub_arrays) > 0:
        if is_one_full_telegram(sub_arrays[0]):
            process_telegram(sub_arrays[0])
        else:
            return sub_arrays[0]
    if len(sub_arrays) > 1:
        return process_telegrams(sub_arrays[1])
    return b''

buffer = b''

hx = b''

##
## Data recorded while pressing Data refresh in the StecaGrid Software
##

hx += bytes.fromhex("02 01 00 10 04 7b bf 40 03 00 01 1d 72 da 81 03") # .....{.@....r...
# 02 01 00 10 04 7b bf 40 03 00 01 1d 72 da 81 03
# dgram: to: 4  from: 123  len: 16  crc1: bf  crc2: da81
# payload: 40 03 00 01 1d 72    @....r
# RequestA for 0x1d (Nominal Power) from 4
# Parsed = [4, 123, 64, 29]

hx += bytes.fromhex("02 01 00 10 01 7b b5 40 03 00 01 22 77 12 ee 03") # .....{.@..."w...
# 02 01 00 10 01 7b b5 40 03 00 01 22 77 12 ee 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 12ee
# payload: 40 03 00 01 22 77    @..."w
# RequestA for 0x22 (Panel Power) from 1
# Parsed = [1, 123, 64, 34]

hx += bytes.fromhex("02 01 00 21 7b 01 15 41 00 00 12 22 00 00 0a 50") # ...!{..A..."...P
hx += bytes.fromhex("61 6e 65 6c 50 6f 77 65 72 0b 5e c2 85 2e 86 83") # anelPower.^.....
hx += bytes.fromhex("03 02 01 00 10 01 7b b5 40 03 00 01 23 78 78 e4") # ......{.@...#xx.
# 02 01 00 21 7b 01 15 41 00 00 12 22 00 00 0a 50 61 6e 65 6c 50 6f 77 65 72 0b 5e c2 85 2e 86 83 03
# dgram: to: 123  from: 1  len: 33  crc1: 15  crc2: 8683
# payload: 41 00 00 12 22 00 00 0a 50 61 6e 65 6c 50 6f 77 65 72 0b 5e c2 85 2e    A..."...PanelPower.^...
# ReponseA for 0x22 from 123 len=18
# i: 0x42AF6100 = 1118789888
# f: 87.689453125
# PanelPower 87.689453125 W
# Parsed = [123, 1, 65, 34, 'PanelPower', [87.689453125, 'W']]

hx += bytes.fromhex("03 02 01 00 23 7b 01 e4 41 00 00 14 23 00 00 0c") # ....#{..A...#...
# 02 01 00 10 01 7b b5 40 03 00 01 23 78 78 e4 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 78e4
# payload: 40 03 00 01 23 78    @...#x
# RequestA for 0x23 (Panel Voltage) from 1
# Parsed = [1, 123, 64, 35]

hx += bytes.fromhex("50 61 6e 65 6c 56 6f 6c 74 61 67 65 05 c1 85 87") # PanelVoltage....
hx += bytes.fromhex("18 34 4e 03 02 01 00 10 01 7b b5 40 03 00 01 24") # .4N......{.@...$
# 02 01 00 23 7b 01 e4 41 00 00 14 23 00 00 0c 50 61 6e 65 6c 56 6f 6c 74 61 67 65 05 c1 85 87 18 34 4e 03
# dgram: to: 123  from: 1  len: 35  crc1: e4  crc2: 344e
# payload: 41 00 00 14 23 00 00 0c 50 61 6e 65 6c 56 6f 6c 74 61 67 65 05 c1 85 87 18    A...#...PanelVoltage.....
# ReponseA for 0x23 from 123 len=20
# i: 0x43E0C280 = 1138803328
# f: 449.51953125
# PanelVoltage 449.51953125 V
# Parsed = [123, 1, 65, 35, 'PanelVoltage', [449.51953125, 'V']]

hx += bytes.fromhex("79 a0 b6 03 02 01 00 23 7b 01 e4 41 00 00 14 24") # y......#{..A...$
# 02 01 00 10 01 7b b5 40 03 00 01 24 79 a0 b6 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: a0b6
# payload: 40 03 00 01 24 79    @...$y
# RequestA for 0x24 (Panel Current) from 1
# Parsed = [1, 123, 64, 36]

hx += bytes.fromhex("00 00 0c 50 61 6e 65 6c 43 75 72 72 65 6e 74 07") # ...PanelCurrent.
hx += bytes.fromhex("8d 4f 7c b7 55 51 03 02 01 00 10 01 7b b5 40 03") # .O|.UQ......{.@.
# 02 01 00 23 7b 01 e4 41 00 00 14 24 00 00 0c 50 61 6e 65 6c 43 75 72 72 65 6e 74 07 8d 4f 7c b7 55 51 03
# dgram: to: 123  from: 1  len: 35  crc1: e4  crc2: 5551
# payload: 41 00 00 14 24 00 00 0c 50 61 6e 65 6c 43 75 72 72 65 6e 74 07 8d 4f 7c b7    A...$...PanelCurrent..O|.
# ReponseA for 0x24 from 123 len=20
# i: 0x3E46A780 = 1044817792
# f: 0.1939983367919922
# PanelCurrent 0.1939983367919922 A
# Parsed = [123, 1, 65, 36, 'PanelCurrent', [0.1939983367919922, 'A']]

hx += bytes.fromhex("00 01 29 7e 98 5b 03 02 01 00 1e 7b 01 3d 41 00") # ..)~.[.....{.=A.
# 02 01 00 10 01 7b b5 40 03 00 01 29 7e 98 5b 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 985b
# payload: 40 03 00 01 29 7e    @...)~
# RequestA for 0x29 (ACPower) from 1
# Parsed = [1, 123, 64, 41]

hx += bytes.fromhex("00 0f 29 00 00 07 41 43 50 6f 77 65 72 0b 74 8f") # ..)...ACPower.t.
hx += bytes.fromhex("85 a9 4a 6a 03 02 01 00 10 01 7b b5 40 03 00 01") # ..Jj......{.@...
# 02 01 00 1e 7b 01 3d 41 00 00 0f 29 00 00 07 41 43 50 6f 77 65 72 0b 74 8f 85 a9 4a 6a 03
# dgram: to: 123  from: 1  len: 30  crc1: 3d  crc2: 4a6a
# payload: 41 00 00 0f 29 00 00 07 41 43 50 6f 77 65 72 0b 74 8f 85 a9    A...)...ACPower.t...
# ReponseA for 0x29 from 123 len=15
# i: 0x42BA4780 = 1119504256
# f: 93.1396484375
# ACPower 93.1396484375 W
# Parsed = [123, 1, 65, 41, 'ACPower', [93.1396484375, 'W']]

hx += bytes.fromhex("51 a6 d4 c0 03") # Q....
# 02 01 00 10 01 7b b5 40 03 00 01 51 a6 d4 c0 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: d4c0
# payload: 40 03 00 01 51 a6    @...Q.
# RequestA for 0x51 from 1
# Parsed = [1, 123, 64, 81]

hx += bytes.fromhex("02 01 00 0c 7b 01 eb 41 11 26 7c 03") # ....{..A.&|.
# 02 01 00 0c 7b 01 eb 41 11 26 7c 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: 267c
# payload: 41 11    A.
# Parsed = [123, 1, 65, 3]

hx += bytes.fromhex("02 01 00 68 7b 01 e2 41 00 00 59 51 00 00 19 4c") # ...h{..A..YQ...L
hx += bytes.fromhex("31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c") # 1 MeasurementVal
hx += bytes.fromhex("75 65 73 20 45 4e 53 31 05 d6 0d 86 0d 8f f7 84") # ues ENS1........
hx += bytes.fromhex("07 00 00 00 05 d6 c6 86 00 00 19 4c 31 20 4d 65") # ...........L1 Me
hx += bytes.fromhex("61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20") # asurementValues
hx += bytes.fromhex("45 4e 53 32 05 d6 73 86 0d 8f f9 84 07 47 ae 77") # ENS2..s......G.w
hx += bytes.fromhex("05 d7 2b 86 0d c4 e0 03 02 01 00 10 01 7b b5 40") # ..+..........{.@
# 02 01 00 68 7b 01 e2 41 00 00 59 51 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 31 05 d6 0d 86 0d 8f f7 84 07 00 00 00 05 d6 c6 86 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 32 05 d6 73 86 0d 8f f9 84 07 47 ae 77 05 d7 2b 86 0d c4 e0 03
# dgram: to: 123  from: 1  len: 104  crc1: e2  crc2: c4e0
# payload: 41 00 00 59 51 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 31 05 d6 0d 86 0d 8f f7 84 07 00 00 00 05 d6 c6 86 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 32 05 d6 73 86 0d 8f f9 84 07 47 ae 77 05 d7 2b 86 0d    A..YQ...L1 MeasurementValues ENS1...................L1 MeasurementValues ENS2..s......G.w..+..
# ReponseA for 0x51 from 123 len=89
# i: 0x436B0680 = 1131087488
# f: 235.025390625
# i: 0x4247FB80 = 1112013696
# f: 49.99560546875
# i: 0x0 = 0
# f: 0.0
# i: 0x436B6300 = 1131111168
# f: 235.38671875
# b'L1 MeasurementValues ENS1' 235.03 V 50.00 Hz 0.00 A 235.39 V
# i: 0x436B3980 = 1131100544
# f: 235.224609375
# i: 0x4247FC80 = 1112013952
# f: 49.99658203125
# i: 0x3BA3D700 = 1000593152
# f: 0.004999995231628418
# i: 0x436B9580 = 1131124096
# f: 235.583984375
# b'L1 MeasurementValues ENS2' 235.22 V 50.00 Hz 0.00 A 235.58 V
# Parsed = [123, 1, 65, 81]

hx += bytes.fromhex("03 00 01 51 a6 d4 c0 03 02 01 00 68 7b 01 e2 41") # ...Q.......h{..A
# 02 01 00 10 01 7b b5 40 03 00 01 51 a6 d4 c0 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: d4c0
# payload: 40 03 00 01 51 a6    @...Q.
# RequestA for 0x51 from 1
# Parsed = [1, 123, 64, 81]

hx += bytes.fromhex("00 00 59 51 00 00 19 4c 31 20 4d 65 61 73 75 72") # ..YQ...L1 Measur
hx += bytes.fromhex("65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 31") # ementValues ENS1
hx += bytes.fromhex("05 d6 06 86 0d 8f f7 84 07 06 24 75 05 d6 c3 86") # ..........$u....
hx += bytes.fromhex("00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e") # ...L1 Measuremen
hx += bytes.fromhex("74 56 61 6c 75 65 73 20 45 4e 53 32 05 d6 68 86") # tValues ENS2..h.
hx += bytes.fromhex("0d 8f f7 84 07 06 24 77 05 d7 28 86 c7 35 95 03") # ......$w..(..5..
# 02 01 00 68 7b 01 e2 41 00 00 59 51 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 31 05 d6 06 86 0d 8f f7 84 07 06 24 75 05 d6 c3 86 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 32 05 d6 68 86 0d 8f f7 84 07 06 24 77 05 d7 28 86 c7 35 95 03
# dgram: to: 123  from: 1  len: 104  crc1: e2  crc2: 3595
# payload: 41 00 00 59 51 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 31 05 d6 06 86 0d 8f f7 84 07 06 24 75 05 d6 c3 86 00 00 19 4c 31 20 4d 65 61 73 75 72 65 6d 65 6e 74 56 61 6c 75 65 73 20 45 4e 53 32 05 d6 68 86 0d 8f f7 84 07 06 24 77 05 d7 28 86 c7    A..YQ...L1 MeasurementValues ENS1..........$u.......L1 MeasurementValues ENS2..h.......$w..(..
# ReponseA for 0x51 from 123 len=89
# i: 0x436B0300 = 1131086592
# f: 235.01171875
# i: 0x4247FB80 = 1112013696
# f: 49.99560546875
# i: 0x3A831200 = 981668352
# f: 0.0009999871253967285
# i: 0x436B6180 = 1131110784
# f: 235.380859375
# b'L1 MeasurementValues ENS1' 235.01 V 50.00 Hz 0.00 A 235.38 V
# i: 0x436B3400 = 1131099136
# f: 235.203125
# i: 0x4247FB80 = 1112013696
# f: 49.99560546875
# i: 0x3B831200 = 998445568
# f: 0.003999948501586914
# i: 0x436B9400 = 1131123712
# f: 235.578125
# b'L1 MeasurementValues ENS2' 235.20 V 50.00 Hz 0.00 A 235.58 V
# Parsed = [123, 1, 65, 81]

hx += bytes.fromhex("02 01 00 10 01 7b b5 40 03 00 01 52 a7 21 a4 03") # .....{.@...R.!..
# 02 01 00 10 01 7b b5 40 03 00 01 52 a7 21 a4 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 21a4
# payload: 40 03 00 01 52 a7    @...R.
# RequestA for 0x52 from 1
# Parsed = [1, 123, 64, 82]

hx += bytes.fromhex("02 01 00 0c 7b 01 eb 41 01 6d 06 03 02 01 00 10") # ....{..A.m......
# 02 01 00 0c 7b 01 eb 41 01 6d 06 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: 6d06
# payload: 41 01    A.
# Parsed = [123, 1, 65, 3]

hx += bytes.fromhex("01 7b b5 40 03 00 01 52 a7 21 a4 03 02 01 00 0c") # .{.@...R.!......
# 02 01 00 10 01 7b b5 40 03 00 01 52 a7 21 a4 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 21a4
# payload: 40 03 00 01 52 a7    @...R.
# RequestA for 0x52 from 1
# Parsed = [1, 123, 64, 82]

hx += bytes.fromhex("7b 01 eb 41 01 6d 06 03 02 01 00 10 01 7b b5 40") # {..A.m.......{.@
# 02 01 00 0c 7b 01 eb 41 01 6d 06 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: 6d06
# payload: 41 01    A.
# Parsed = [123, 1, 65, 3]

hx += bytes.fromhex("03 00 01 53 a8 4b ae 03 02 01 00 0c 7b 01 eb 41") # ...S.K......{..A
# 02 01 00 10 01 7b b5 40 03 00 01 53 a8 4b ae 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 4bae
# payload: 40 03 00 01 53 a8    @...S.
# RequestA for 0x53 from 1
# Parsed = [1, 123, 64, 83]

hx += bytes.fromhex("01 6d 06 03 02 01 00 10 01 7b b5 40 03 00 01 53") # .m.......{.@...S
# 02 01 00 0c 7b 01 eb 41 01 6d 06 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: 6d06
# payload: 41 01    A.
# Parsed = [123, 1, 65, 3]

hx += bytes.fromhex("a8 4b ae 03 02 01 00 0c 7b 01 eb 41 01 6d 06 03") # .K......{..A.m..
# 02 01 00 10 01 7b b5 40 03 00 01 53 a8 4b ae 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 4bae
# payload: 40 03 00 01 53 a8    @...S.
# RequestA for 0x53 from 1
# Parsed = [1, 123, 64, 83]

# 02 01 00 0c 7b 01 eb 41 01 6d 06 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: 6d06
# payload: 41 01    A.
# Parsed = [123, 1, 65, 3]

hx += bytes.fromhex("02 01 00 10 01 7b b5 40 03 00 01 3c 91 e1 c9 03") # .....{.@...<....
# 02 01 00 10 01 7b b5 40 03 00 01 3c 91 e1 c9 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: e1c9
# payload: 40 03 00 01 3c 91    @...<.
# RequestA for 0x3c (Daily Yield) from 1
# Parsed = [1, 123, 64, 60]

hx += bytes.fromhex("02 01 00 14 7b 01 43 41 00 00 05 3c 09 1a 80 89") # ....{.CA...<....
hx += bytes.fromhex("bd 8e 6a 03 02 01 00 10 01 7b b5 64 03 00 01 f1") # ..j......{.d....
# 02 01 00 14 7b 01 43 41 00 00 05 3c 09 1a 80 89 bd 8e 6a 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: 8e6a
# payload: 41 00 00 05 3c 09 1a 80 89 bd    A...<.....
# ReponseA for 0x3c from 123 len=5
# i: 0x448D4000 = 1150107648
# f: 1130.0
# Daily Yield 1130.0 Wh
# Parsed = [123, 1, 65, 60, 'Daily Yield', [1130.0, 'Wh']]

hx += bytes.fromhex("46 cc 79 03 02 01 00 14 7b 01 43 65 00 00 05 f1") # F.y.....{.Ce....
# 02 01 00 10 01 7b b5 64 03 00 01 f1 46 cc 79 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: cc79
# payload: 64 03 00 01 f1 46    d....F
# RequestB for 0xf1 from 1
# Parsed = [1, 123, 100, 241]

hx += bytes.fromhex("0a 58 24 4c 18 dc 46 03 02 01 00 10 01 7b b5 64") # .X$L..F......{.d
# 02 01 00 14 7b 01 43 65 00 00 05 f1 0a 58 24 4c 18 dc 46 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: dc46
# payload: 65 00 00 05 f1 0a 58 24 4c 18    e.....X$L.
# ReponseB for 0xf1 from 123
# ( 0a 58 24 4c 18 )
# i: 0x262C1200 = 640422400
# f: 5.969888212248309e-16
# 0.00 0x0a
# Parsed = [123, 1, 101, 241, '???', '0a 58 24 4c 18']

hx += bytes.fromhex("03 00 01 05 5a 3a 44 03 02 01 00 17 7b 01 56 65") # ....Z:D.....{.Ve
# 02 01 00 10 01 7b b5 64 03 00 01 05 5a 3a 44 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 3a44
# payload: 64 03 00 01 05 5a    d....Z
# RequestB for 0x05 from 1
# Parsed = [1, 123, 100, 5]

hx += bytes.fromhex("00 00 08 05 18 02 04 0e 1b 29 01 cb b2 9e 03 02") # .........)......
# 02 01 00 17 7b 01 56 65 00 00 08 05 18 02 04 0e 1b 29 01 cb b2 9e 03
# dgram: to: 123  from: 1  len: 23  crc1: 56  crc2: b29e
# payload: 65 00 00 08 05 18 02 04 0e 1b 29 01 cb    e.........)..
# ReponseB for 0x05 from 123
# 2024-02-04 14:27:41 ( 18 02 04 0e 1b 29 01 cb b2 )
# Parsed = [123, 1, 101, 5, 'Time', datetime.datetime(2024, 2, 4, 14, 27, 41)]

hx += bytes.fromhex("01 00 10 01 7b b5 64 03 00 01 08 5d 02 a9 03 02") # ....{.d....]....
# 02 01 00 10 01 7b b5 64 03 00 01 08 5d 02 a9 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 02a9
# payload: 64 03 00 01 08 5d    d....]
# RequestB for 0x08 from 1
# Parsed = [1, 123, 100, 8]

hx += bytes.fromhex("01 00 14 7b 01 43 65 00 00 05 08 08 56 05 33 f3") # ...{.Ce.....V.3.
hx += bytes.fromhex("6d da 03") # m..
# 02 01 00 14 7b 01 43 65 00 00 05 08 08 56 05 33 f3 6d da 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: 6dda
# payload: 65 00 00 05 08 08 56 05 33 f3    e.....V.3.
# ReponseB for 0x08 from 123
# ( 08 56 05 33 f3 )
# i: 0x19AB0280 = 430637696
# f: 1.768199533527965e-23
# 0.00 0x08
# Parsed = [123, 1, 101, 8, '???', '08 56 05 33 f3']

hx += bytes.fromhex("02 01 00 10 05 7b bd 40 03 00 01 1d 72 0f f7 03") # .....{.@....r...
# 02 01 00 10 05 7b bd 40 03 00 01 1d 72 0f f7 03
# dgram: to: 5  from: 123  len: 16  crc1: bd  crc2: 0ff7
# payload: 40 03 00 01 1d 72    @....r
# RequestA for 0x1d (Nominal Power) from 5
# Parsed = [5, 123, 64, 29]

hx += bytes.fromhex("02 01 00 14 01 7b 6e 50 03 00 05 0d 00 ff 03 e8") # .....{nP........
hx += bytes.fromhex("4c 5a 00 03 02 01 00 0c 7b 01 eb 51 00 eb c2 03") # LZ......{..Q....
# 02 01 00 14 01 7b 6e 50 03 00 05 0d 00 ff 03 e8 4c 5a 00 03
# dgram: to: 1  from: 123  len: 20  crc1: 6e  crc2: 5a00
# payload: 50 03 00 05 0d 00 ff 03 e8 4c    P........L
# Parsed = [1, 123, 80, 13]

# 02 01 00 0c 7b 01 eb 51 00 eb c2 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: ebc2
# payload: 51 00    Q.
# Parsed = [123, 1, 81, 3]

##
## Data recorded while rediscovering inverters in the StecaGrid Software
##

hx += bytes.fromhex("02 01 00 0c 01 7b c6 20 03 79 8c 03") # .....{. .y..
# 02 01 00 0c 01 7b c6 20 03 79 8c 03
# dgram: to: 1  from: 123  len: 12  crc1: c6  crc2: 798c
# payload: 20 03     .
# Parsed = [1, 123, 32, 3]

hx += bytes.fromhex("02 01 02 a8 7b 01 77 21 00 02 99 53 74 65 63 61") # ....{.w!...Steca
hx += bytes.fromhex("47 72 69 64 20 33 36 30 30 00 37 34 38 36 31 33") # Grid 3600.748613
hx += bytes.fromhex("30 30 35 32 31 32 38 35 30 30 32 39 00 00 11 48") # 005212850029...H
hx += bytes.fromhex("4d 49 20 42 46 41 50 49 20 00 02 05 00 00 00 00") # MI BFAPI .......
hx += bytes.fromhex("31 39 2e 30 33 2e 32 30 31 33 20 31 34 3a 33 38") # 19.03.2013 14:38
hx += bytes.fromhex("3a 35 39 00 48 4d 49 20 46 42 4c 20 00 01 02 00") # :59.HMI FBL ....
hx += bytes.fromhex("03 00 00 30 35 2e 30 34 2e 32 30 31 33 20 31 31") # ...05.04.2013 11
hx += bytes.fromhex("3a 34 36 3a 32 30 00 48 4d 49 20 41 50 50 20 00") # :46:20.HMI APP .
hx += bytes.fromhex("01 0f 00 00 00 00 32 36 2e 30 37 2e 32 30 31 33") # ......26.07.2013
hx += bytes.fromhex("20 31 33 3a 31 39 3a 30 36 00 48 4d 49 20 50 41") #  13:19:06.HMI PA
hx += bytes.fromhex("52 20 00 02 00 00 01 00 00 32 36 2e 30 37 2e 32") # R .......26.07.2
hx += bytes.fromhex("30 31 33 20 31 33 3a 31 39 3a 30 36 00 48 4d 49") # 013 13:19:06.HMI
hx += bytes.fromhex("20 4f 45 4d 20 00 01 00 00 01 00 00 31 31 2e 30") #  OEM .......11.0
hx += bytes.fromhex("36 2e 32 30 31 33 20 30 38 3a 31 31 3a 32 39 00") # 6.2013 08:11:29.
hx += bytes.fromhex("50 55 20 42 46 41 50 49 20 00 02 05 00 00 00 00") # PU BFAPI .......
hx += bytes.fromhex("31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38") # 19.03.2013_14:38
hx += bytes.fromhex("3a 34 32 00 50 55 20 46 42 4c 20 00 01 01 00 01") # :42.PU FBL .....
hx += bytes.fromhex("00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a") # ..19.12.2012_16:
hx += bytes.fromhex("33 36 3a 30 34 00 50 55 20 41 50 50 20 00 05 04") # 36:04.PU APP ...
hx += bytes.fromhex("00 00 00 00 30 33 2e 30 35 2e 32 30 31 33 5f 30") # ....03.05.2013_0
hx += bytes.fromhex("39 3a 33 37 3a 35 35 00 50 55 20 50 41 52 20 00") # 9:37:55.PU PAR .
hx += bytes.fromhex("05 03 00 00 00 00 33 31 2e 30 31 2e 32 30 31 33") # ......31.01.2013
hx += bytes.fromhex("5f 31 33 3a 34 37 3a 32 34 00 45 4e 53 31 20 42") # _13:47:24.ENS1 B
hx += bytes.fromhex("46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30") # FAPI .......19.0
hx += bytes.fromhex("33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35 31 00") # 3.2013_14:38:51.
hx += bytes.fromhex("45 4e 53 31 20 46 42 4c 20 00 01 01 00 01 00 00") # ENS1 FBL .......
hx += bytes.fromhex("31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 34") # 19.12.2012_16:34
hx += bytes.fromhex("3a 34 37 00 45 4e 53 31 20 41 50 50 20 00 03 27") # :47.ENS1 APP ..'
hx += bytes.fromhex("00 00 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31") # ....11.07.2013_1
hx += bytes.fromhex("34 3a 33 39 3a 35 30 00 45 4e 53 31 20 50 41 52") # 4:39:50.ENS1 PAR
hx += bytes.fromhex("20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e 32 30") #  .......11.07.20
hx += bytes.fromhex("31 33 5f 31 34 3a 34 30 3a 30 33 00 45 4e 53 32") # 13_14:40:03.ENS2
hx += bytes.fromhex("20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39") #  BFAPI .......19
hx += bytes.fromhex("2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35") # .03.2013_14:38:5
hx += bytes.fromhex("31 00 45 4e 53 32 20 46 42 4c 20 00 01 01 00 01") # 1.ENS2 FBL .....
hx += bytes.fromhex("00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a") # ..19.12.2012_16:
hx += bytes.fromhex("33 34 3a 34 37 00 45 4e 53 32 20 41 50 50 20 00") # 34:47.ENS2 APP .
hx += bytes.fromhex("03 27 00 00 00 00 31 31 2e 30 37 2e 32 30 31 33") # .'....11.07.2013
hx += bytes.fromhex("5f 31 34 3a 33 39 3a 35 30 00 45 4e 53 32 20 50") # _14:39:50.ENS2 P
hx += bytes.fromhex("41 52 20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e") # AR .......11.07.
hx += bytes.fromhex("32 30 31 33 5f 31 34 3a 34 30 3a 30 33 00 03 48") # 2013_14:40:03..H
hx += bytes.fromhex("4d 49 00 01 50 55 00 03 45 4e 53 32 00 02 4e 65") # MI..PU..ENS2..Ne
hx += bytes.fromhex("74 31 31 00 30 46 79 03 02 01 00 10 01 7b b5 64") # t11.0Fy......{.d
# 02 01 02 a8 7b 01 77 21 00 02 99 53 74 65 63 61 47 72 69 64 20 33 36 30 30 00 37 34 38 36 31 33 30 30 35 32 31 32 38 35 30 30 32 39 00 00 11 48 4d 49 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 20 31 34 3a 33 38 3a 35 39 00 48 4d 49 20 46 42 4c 20 00 01 02 00 03 00 00 30 35 2e 30 34 2e 32 30 31 33 20 31 31 3a 34 36 3a 32 30 00 48 4d 49 20 41 50 50 20 00 01 0f 00 00 00 00 32 36 2e 30 37 2e 32 30 31 33 20 31 33 3a 31 39 3a 30 36 00 48 4d 49 20 50 41 52 20 00 02 00 00 01 00 00 32 36 2e 30 37 2e 32 30 31 33 20 31 33 3a 31 39 3a 30 36 00 48 4d 49 20 4f 45 4d 20 00 01 00 00 01 00 00 31 31 2e 30 36 2e 32 30 31 33 20 30 38 3a 31 31 3a 32 39 00 50 55 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 34 32 00 50 55 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 36 3a 30 34 00 50 55 20 41 50 50 20 00 05 04 00 00 00 00 30 33 2e 30 35 2e 32 30 31 33 5f 30 39 3a 33 37 3a 35 35 00 50 55 20 50 41 52 20 00 05 03 00 00 00 00 33 31 2e 30 31 2e 32 30 31 33 5f 31 33 3a 34 37 3a 32 34 00 45 4e 53 31 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35 31 00 45 4e 53 31 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 34 3a 34 37 00 45 4e 53 31 20 41 50 50 20 00 03 27 00 00 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 33 39 3a 35 30 00 45 4e 53 31 20 50 41 52 20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 34 30 3a 30 33 00 45 4e 53 32 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35 31 00 45 4e 53 32 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 34 3a 34 37 00 45 4e 53 32 20 41 50 50 20 00 03 27 00 00 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 33 39 3a 35 30 00 45 4e 53 32 20 50 41 52 20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 34 30 3a 30 33 00 03 48 4d 49 00 01 50 55 00 03 45 4e 53 32 00 02 4e 65 74 31 31 00 30 46 79 03
# dgram: to: 123  from: 1  len: 680  crc1: 77  crc2: 4679
# payload: 21 00 02 99 53 74 65 63 61 47 72 69 64 20 33 36 30 30 00 37 34 38 36 31 33 30 30 35 32 31 32 38 35 30 30 32 39 00 00 11 48 4d 49 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 20 31 34 3a 33 38 3a 35 39 00 48 4d 49 20 46 42 4c 20 00 01 02 00 03 00 00 30 35 2e 30 34 2e 32 30 31 33 20 31 31 3a 34 36 3a 32 30 00 48 4d 49 20 41 50 50 20 00 01 0f 00 00 00 00 32 36 2e 30 37 2e 32 30 31 33 20 31 33 3a 31 39 3a 30 36 00 48 4d 49 20 50 41 52 20 00 02 00 00 01 00 00 32 36 2e 30 37 2e 32 30 31 33 20 31 33 3a 31 39 3a 30 36 00 48 4d 49 20 4f 45 4d 20 00 01 00 00 01 00 00 31 31 2e 30 36 2e 32 30 31 33 20 30 38 3a 31 31 3a 32 39 00 50 55 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 34 32 00 50 55 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 36 3a 30 34 00 50 55 20 41 50 50 20 00 05 04 00 00 00 00 30 33 2e 30 35 2e 32 30 31 33 5f 30 39 3a 33 37 3a 35 35 00 50 55 20 50 41 52 20 00 05 03 00 00 00 00 33 31 2e 30 31 2e 32 30 31 33 5f 31 33 3a 34 37 3a 32 34 00 45 4e 53 31 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35 31 00 45 4e 53 31 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 34 3a 34 37 00 45 4e 53 31 20 41 50 50 20 00 03 27 00 00 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 33 39 3a 35 30 00 45 4e 53 31 20 50 41 52 20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 34 30 3a 30 33 00 45 4e 53 32 20 42 46 41 50 49 20 00 02 05 00 00 00 00 31 39 2e 30 33 2e 32 30 31 33 5f 31 34 3a 33 38 3a 35 31 00 45 4e 53 32 20 46 42 4c 20 00 01 01 00 01 00 00 31 39 2e 31 32 2e 32 30 31 32 5f 31 36 3a 33 34 3a 34 37 00 45 4e 53 32 20 41 50 50 20 00 03 27 00 00 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 33 39 3a 35 30 00 45 4e 53 32 20 50 41 52 20 00 13 00 00 0e 00 00 31 31 2e 30 37 2e 32 30 31 33 5f 31 34 3a 34 30 3a 30 33 00 03 48 4d 49 00 01 50 55 00 03 45 4e 53 32 00 02 4e 65 74 31 31 00 30    !...StecaGrid 3600.748613005212850029...HMI BFAPI .......19.03.2013 14:38:59.HMI FBL .......05.04.2013 11:46:20.HMI APP .......26.07.2013 13:19:06.HMI PAR .......26.07.2013 13:19:06.HMI OEM .......11.06.2013 08:11:29.PU BFAPI .......19.03.2013_14:38:42.PU FBL .......19.12.2012_16:36:04.PU APP .......03.05.2013_09:37:55.PU PAR .......31.01.2013_13:47:24.ENS1 BFAPI .......19.03.2013_14:38:51.ENS1 FBL .......19.12.2012_16:34:47.ENS1 APP ..'....11.07.2013_14:39:50.ENS1 PAR .......11.07.2013_14:40:03.ENS2 BFAPI .......19.03.2013_14:38:51.ENS2 FBL .......19.12.2012_16:34:47.ENS2 APP ..'....11.07.2013_14:39:50.ENS2 PAR .......11.07.2013_14:40:03..HMI..PU..ENS2..Net11.0
# ReponseC for 83 from 123 len= 665
# Parsed = [123, 1, 33, 83, '???', '74 65 63 61 47']

hx += bytes.fromhex("03 00 01 09 5e 85 6e 03 02 01 00 24 7b 01 2a 65") # ....^.n....${.*e
# 02 01 00 10 01 7b b5 64 03 00 01 09 5e 85 6e 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 856e
# payload: 64 03 00 01 09 5e    d....^
# RequestB for 0x09 from 1
# Parsed = [1, 123, 100, 9]

hx += bytes.fromhex("00 00 15 09 37 34 38 36 31 33 59 49 30 30 35 32") # ....748613YI0052
hx += bytes.fromhex("31 32 38 35 30 30 32 39 9f ab 3b 03 02 01 00 10") # 12850029..;.....
# 02 01 00 24 7b 01 2a 65 00 00 15 09 37 34 38 36 31 33 59 49 30 30 35 32 31 32 38 35 30 30 32 39 9f ab 3b 03
# dgram: to: 123  from: 1  len: 36  crc1: 2a  crc2: ab3b
# payload: 65 00 00 15 09 37 34 38 36 31 33 59 49 30 30 35 32 31 32 38 35 30 30 32 39 9f    e....748613YI005212850029.
# ReponseB for 0x09 from 123
# ( 37 34 38 36 31 )
# i: 0x1B1A1C00 = 454695936
# f: 1.2747628721266425e-22
# 0.00 0x37
# Parsed = [123, 1, 101, 9, 'Serial Number', '748613YI005212850029']

hx += bytes.fromhex("01 7b b5 54 03 00 01 32 87 e1 78 03") # .{.T...2..x.
# 02 01 00 10 01 7b b5 54 03 00 01 32 87 e1 78 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: e178
# payload: 54 03 00 01 32 87    T...2.
# Parsed = [1, 123, 84, 50]

hx += bytes.fromhex("02 01 00 0c 7b 01 eb 55 11 bf 92 03") # ....{..U....
# 02 01 00 0c 7b 01 eb 55 11 bf 92 03
# dgram: to: 123  from: 1  len: 12  crc1: eb  crc2: bf92
# payload: 55 11    U.
# Parsed = [123, 1, 85, 3]

hx += bytes.fromhex("02 01 00 14 7b 01 43 55 00 00 05 32 0f 13 24 01") # ....{.CU...2..$.
hx += bytes.fromhex("ce b5 cb 03 02 01 00 10 01 7b b5 40 03 00 01 1d") # .........{.@....
# 02 01 00 14 7b 01 43 55 00 00 05 32 0f 13 24 01 ce b5 cb 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: b5cb
# payload: 55 00 00 05 32 0f 13 24 01 ce    U...2..$..
# Parsed = [123, 1, 85, 50]

hx += bytes.fromhex("72 30 95 03 02 01 00 25 7b 01 ce 41 00 00 16 1d") # r0.....%{..A....
# 02 01 00 10 01 7b b5 40 03 00 01 1d 72 30 95 03
# dgram: to: 1  from: 123  len: 16  crc1: b5  crc2: 3095
# payload: 40 03 00 01 1d 72    @....r
# RequestA for 0x1d (Nominal Power) from 1
# Parsed = [1, 123, 64, 29]

hx += bytes.fromhex("00 00 0e 4e 6f 6d 69 6e 61 6c 20 50 6f 77 65 72") # ...Nominal Power
hx += bytes.fromhex("3a 0b c2 00 8a 0c f4 46 03") # :......F......{.
# 02 01 00 25 7b 01 ce 41 00 00 16 1d 00 00 0e 4e 6f 6d 69 6e 61 6c 20 50 6f 77 65 72 3a 0b c2 00 8a 0c f4 46 03
# dgram: to: 123  from: 1  len: 37  crc1: ce  crc2: f446
# payload: 41 00 00 16 1d 00 00 0e 4e 6f 6d 69 6e 61 6c 20 50 6f 77 65 72 3a 0b c2 00 8a 0c    A.......Nominal Power:.....
# ReponseA for 0x1d from 123 len=22
# i: 0x45610000 = 1163984896
# f: 3600.0
# Nominal Power: 3600.0 W
# Parsed = [123, 1, 65, 29, 'Nominal Power:', [3600.0, 'W']]

hx += bytes.fromhex("02 01 00 14 7b 01 43 65 00 00 05 f1 0a 58 24 4c 18 dc 46 03") # 43081.77 kWh Total Yield
# 02 01 00 14 7b 01 43 65 00 00 05 f1 0a 58 24 4c 18 dc 46 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: dc46
# payload: 65 00 00 05 f1 0a 58 24 4c 18    e.....X$L.
# ReponseB for 0xf1 from 123
# ( 0a 58 24 4c 18 )
# [43081768.0, 'Wh']
# Parsed = [123, 1, 101, 241, 'Total Yield', [43081768.0, 'Wh']]

hx += bytes.fromhex("02 01 00 14 7b 01 43 65 00 00 05 f1 ef 56 24 4c fb 95 d1 03") # 43080.64 kWh Total Yield
# 02 01 00 14 7b 01 43 65 00 00 05 f1 ef 56 24 4c fb 95 d1 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: 95d1
# payload: 65 00 00 05 f1 ef 56 24 4c fb    e.....V$L.
# ReponseB for 0xf1 from 123
# ( ef 56 24 4c fb )
# [43080636.0, 'Wh']
# Parsed = [123, 1, 101, 241, 'Total Yield', [43080636.0, 'Wh']]

hx += bytes.fromhex("02 01 00 14 7b 01 43 65 00 00 05 f1 26 58 24 4c 34 5d 50 03") # 43081,88 kWh Total Yield
# 02 01 00 14 7b 01 43 65 00 00 05 f1 26 58 24 4c 34 5d 50 03
# dgram: to: 123  from: 1  len: 20  crc1: 43  crc2: 5d50
# payload: 65 00 00 05 f1 26 58 24 4c 34    e....&X$L4
# ReponseB for 0xf1 from 123
# ( 26 58 24 4c 34 )
# [43081880.0, 'Wh']
# Parsed = [123, 1, 101, 241, 'Total Yield', [43081880.0, 'Wh']]

if PLAYBACK: 
    xprocess_telegram(hx)
    rest = process_telegrams(hx)
    print ("rest:",rest)
else:
    while True:
        try:
            data = ser.read(16)
            if data:
                xprocess_telegram(data)
                buffer = buffer + data
                buffer = process_telegrams(buffer)

        except KeyboardInterrupt:
            break

# Close the serial port
ser.close()
