# =======================
#  REGISTER HELPERS
# =======================

def write_reg_1(addr, val, ser):
    ser.write(bytes([0x20, addr, val & 0xFF]))

def write_reg_2(addr, val, ser):
    ser.write(bytes([
        0x21, addr,
        val & 0xFF,
        (val >> 8) & 0xFF
        ]))

def write_reg_4(addr, val, ser):
    ser.write(bytes([0x22, addr]) + val.to_bytes(4, 'little'))

def write_reg_8(addr, val, ser):
    ser.write(bytes([0x23, addr]) + val.to_bytes(8, 'little'))

def read_reg_1(addr, ser):
    ser.write(bytes([0x10, addr]))
    return ser.read(1)

def read_reg_2(addr, ser):
    ser.write(bytes([0x11, addr]))
    return ser.read(2)

def read_reg_4(addr, ser):
    ser.write(bytes([0x12, addr]))
    return ser.read(4)

def read_fifo(addr, count, ser):
    ser.write(bytes([0x18, addr, count]))
    return ser.read(count * 32)   # 32 bytes per point
