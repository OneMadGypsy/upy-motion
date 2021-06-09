from machine import Pin, I2C
from micropython import const
from collections import namedtuple
import struct, utime

#accel
ACCEL_FS_2      = const(0x00)
ACCEL_FS_4      = const(0x01)
ACCEL_FS_8      = const(0x02)
ACCEL_FS_16     = const(0x03)

#gyro
GYRO_FS_250     = const(0x00)
GYRO_FS_500     = const(0x01)
GYRO_FS_1000    = const(0x02)
GYRO_FS_2000    = const(0x03)

#dlpf
DLPF_BW_256     = const(0x00)
DLPF_BW_188     = const(0x01)
DLPF_BW_98      = const(0x02)
DLPF_BW_42      = const(0x03)
DLPF_BW_20      = const(0x04)
DLPF_BW_10      = const(0x05)
DLPF_BW_5       = const(0x06)
    
_D   = namedtuple('D', ('acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'))
_P   = namedtuple('P', ('x', 'y', 'z'))
_SEP = '-'*60
_OUT = '[{:<16}] x: {}{:<10.2f} y: {}{:<10.2f} z: {}{:<10.2f}'
_W   = (' ', '-')
_C   = '{}\n{}\n{}'
_F   = '\t{} failed Self-Test with values: {}'


#__> I2C
class __I2CHelper(object):
    def __init__(self, bus:int, sda, scl, addr:int, freq:int=400000) -> None:
        if isinstance(sda, int): sda = Pin(sda)
        if isinstance(scl, int): scl = Pin(scl)
        self.__addr    = addr
        self.__bus     = I2C(bus, scl=scl, sda=sda, freq=freq)
        self.__buffer  = bytearray(16)
        self.__data    = [memoryview(self.__buffer[0:n]) for n in range(1, 17)]

    def __writeBytes(self, reg:int, buff) -> None:
        self.__bus.writeto_mem(self.__addr, reg, buff)
        
    def __writeWords(self, reg:int, length:int, val) -> None:
        if isinstance(val, int):
            L = int(length * 2)
            val = bytearray(val.to_bytes(L, 'big'))
            
        if isinstance(val, (list, tuple)):
            val = bytearray(val)
            
        if isinstance(val, (bytearray, bytes, memoryview)):
            self.__writeBytes(reg, val)

    def __writeByte(self, reg:int, val) -> None:
        if isinstance(val, int):
            val = bytearray([val]) 
            
        if isinstance(val, (bytearray, bytes, memoryview)):
            self.__writeBytes(reg, val)

    def __writeBit(self, reg:int, bit:int, data:int) -> None:
        b = self.__readByte(reg)
        self.__data[0][0] = (b | (1 << bit)) if data else (b & ~(1 << bit))
        self.__writeByte(reg, self.__data[0][0])

    def __writeBits(self, reg:int, bitstart:int, length:int, data:int) -> None:   
        shift = (bitstart - length + 1)
        mask  = ((1 << length) - 1) << shift
        self.__readByte(reg)
        data <<= shift
        data &= mask
        self.__data[0][0] &= ~(mask)
        self.__data[0][0] |= data
        self.__writeByte(reg, self.__data[0])

    def __readBytes(self, reg:int, length:int) -> int:
        if length > 0:
            if length in range(1, 17):
                self.__bus.readfrom_mem_into(self.__addr, reg, self.__data[length-1])
                return self.__data[length-1]
            else:
                buff = bytearray([0x00]*length)
                self.__bus.readfrom_mem_into(self.__addr, reg, buff)
                return buff
        else:
            return bytearray()

    def __readWords(self, reg:int, length:int) -> int:
        fmt = '>{}'.format('h'*length)
        return struct.unpack(fmt, self.__readBytes(reg, length*2))

    def __readByte(self, reg:int) -> int:
        return self.__readBytes(reg, 1)[0]

    def __readBit(self, reg:int, bit:int) -> int:
        return (self.__readByte(reg) & (1 << bit))

    def __readBits(self, reg:int, bitstart:int, length:int) -> int:
        shift = (bitstart - length + 1)
        mask  = ((1 << length) - 1) << shift
        return ((self.__readByte(reg) & mask) >> shift)


#__> MPU6050
class MPU6050(__I2CHelper):
    
    #__>        PROPERTIES       <__#

    @property
    def device_id(self) -> int:
        return self.__readBits(0x75, 0x6, 0x6)
        
    @property
    def connected(self) -> bool:
        return self.device_id == 0x34
        
    @property
    def data(self) -> namedtuple:
        data = None
        ax, ay, az, gx, gy, gz = 0, 0, 0, 0, 0, 0
        if self.__usefifo:
            data = self.__get_fifo_packet(12)
            if not data is None:
                ax = data[0] * self.__accfact
                ay = data[1] * self.__accfact
                az = data[2] * self.__accfact
                gx = data[3] * self.__gyrofact 
                gy = data[4] * self.__gyrofact 
                gz = data[5] * self.__gyrofact 
            
        if (not self.__usefifo) or (data is None):
            data = struct.unpack(">hhhhhhh", self.__readBytes(0x3b, 14))
            ax = data[0] * self.__accfact 
            ay = data[1] * self.__accfact 
            az = data[2] * self.__accfact 
            gx = data[4] * self.__gyrofact
            gy = data[5] * self.__gyrofact
            gz = data[6] * self.__gyrofact
            
        return _D(ax, ay, az, gx, gy, gz)
     
    @property
    def passed_self_test(self) -> bool:
        if self.connected:
            spec = range(-14, 15) #factory min/max specs
            
            self.__enable_accel_tests() #test accelerometer
            a, b, c = self.__accel_str
            x, y, z = self.__accel_trims
            ares    = (int((a-x)/x), int((a-y)/y), int((a-z)/z))
            accel   = sum([r in spec for r in ares]) == 3
            
            self.__enable_gyro_tests()  #test gyroscope
            a, b, c = self.__gyro_str
            x, y, z = self.__gyro_trims
            gres    = (int((a-x)/x), int((a-y)/y), int((a-z)/z))
            gyro    = sum([r in spec for r in gres]) == 3
            
            self.__disable_tests()      # disble tests and revert to pre-test states
            if not (accel and gyro):
                print('To pass Self-Test values must be between the min and max specs (-14 to 14):')
                if not gyro:
                    print(_F.format('Gyroscope', gres))
                if not accel:
                    print(_F.format('Accelerometer', ares))
                return False
        else:
            print('No Connection To Device')
            return False
        return True
        
    @property
    def celsius(self) -> float:
        return self.__temperature/340 + 36.53
        
    @property
    def fahrenheit(self) -> float:
        return self.celsius * 1.8 + 32    
        
    #__> CONSTRUCTOR
    def __init__(self, bus:int, sda, scl, intr=None, ofs:tuple=None, callback=None, gyro:int=GYRO_FS_500, accel:int=ACCEL_FS_2, rate:int=4, dlpf:int=DLPF_BW_188, addr:int=0x68, freq:int=400000) -> None:
        super().__init__(bus, sda, scl, addr, freq)
        self.__accsense , self.__accfact , self.__accfs   = 0, 0, accel
        self.__gyrosense, self.__gyrofact, self.__gyrofs  = 0, 0, gyro
        self.__rate     , self.__dlpf                     = rate, dlpf
        self.__offsets  , self.__intr                     = None, None
        self.__usefifo                                    = False
        
        self.__enable_interrupts(False)
        self.__enable_fifo_regs(False)
        
        self.__set_clock(0x1)
        self.set_gyro (gyro)
        self.set_accel(accel)
        self.set_rate(rate)
        self.set_dlpf(dlpf)
        self.__disable_tests()
        self.__enable_sleep(False)
        
        if not ofs is None:
            self.__offsets = ofs
            self.__set_accel_offsets(ofs[0], ofs[1], ofs[2])
            self.__set_gyro_offsets (ofs[3], ofs[4], ofs[5])
        else:
            self.calibrate(6)
            
        if (not intr is None) and (not callback is None):
            self.__intr     = Pin(intr, Pin.IN, Pin.PULL_DOWN) if isinstance(intr, int) else Pin(sda)
            self.__intr.irq(self.__handler, Pin.IRQ_RISING)
            self.__callback = callback
            self.__full_reset_fifo()
            self.__enable_fifo_regs(True)
            self.__usefifo  = True
            
    #__>        PUBLIC METHODS       <__#
    
    def start(self) -> None:
        if not self.__usefifo is None:
            self.__enable_interrupts(True)
        
    def stop(self) -> None:
        if not self.__usefifo is None:
            self.__enable_interrupts(False)
            
    def set_dlpf(self, dlpf:int):
        self.__writeBits(0x1a, 0x2, 0x3, dlpf)
        
    def set_rate(self, rate:int) -> None:
        self.__writeByte(0x19, rate)
        
    def set_gyro(self, rng:int) -> None:
        self.__gyrosense = [250, 500, 1000, 2000][rng]
        self.__gyrofact  = self.__gyrosense/32768.0;
        self.__writeBits(0x1b, 0x4, 0x2, rng)

    def set_accel(self, rng:int) -> None:
        self.__accsense = [2, 4, 8, 16][rng]
        self.__accfact  = self.__accsense/32768.0;
        self.__writeBits(0x1c, 0x4, 0x2, rng)
        
    def calibrate(self, loops:int) -> None:
        self.__calibrate_gyro (loops)
        self.__calibrate_accel(loops)
        self.print_offsets()

    def print_offsets(self) -> None:
        ax, ay, az = self.__readWords(0x6   , 3)
        gx, gy, gz = self.__readWords(0x13, 3)
        print('ofs = ({}, {}, {}, {}, {}, {})'.format(ax, ay, az, gx, gy, gz))
        
    def print_data(self):
        self.print_from_data(self.data)
        
    def print_from_data(self, data:tuple) -> None:
        ax, ay, az, gx, gy, gz = data
        a = _OUT.format('ACCELEROMETER', _W[ax<0], abs(ax), _W[ay<0], abs(ay), _W[az<0], abs(az))
        g = _OUT.format('GYROSCOPE'    , _W[gx<0], abs(gx), _W[gy<0], abs(gy), _W[gz<0], abs(gz))
        print(_C.format(a, g, _SEP))
        
    #__>        PRIVATE PROPERTIES     <__#

    @property
    def __temperature(self) -> int:
        return struct.unpack('>h', self.__readBytes(0x41, 2))[0]
        
    @property
    def __fifo_count(self) -> int:
        msb = self.__readByte(0x72)
        lsb = self.__readByte(0x73)
        return (msb << 8) | lsb
        
    @property
    def __accel_str(self) -> tuple: #self-test response
        x = self.__readBit(0x1c, 0x7)
        y = self.__readBit(0x1c, 0x6)
        z = self.__readBit(0x1c, 0x5)
        return x, y, z
       
    @property
    def __accel_trims(self) -> tuple:
        a  = self.__readByte(0x10)	
        x_ = self.__readByte(0xd)
        x  = ((x_>>3) & 0x1C) | ((a>>4) & 0x03)
        y_ = self.__readByte(0xe)
        y  = ((y_>>3) & 0x1C) | ((a>>2) & 0x03) 
        z_ = self.__readByte(0xf)
        z  = ((z_>>3) & 0x1C) | (a & 0x03) 
        return x, y, z
        
    @property
    def __gyro_str(self) -> tuple:  #self-test response
        x = self.__readBit(0x1b, 0x7)
        y = self.__readBit(0x1b, 0x6)
        z = self.__readBit(0x1b, 0x5)
        return x, y, z
    
    @property
    def __gyro_trims(self) -> tuple:
        x = self.__readByte(0xd) & 0x1F
        y = self.__readByte(0xe) & 0x1F
        z = self.__readByte(0xf) & 0x1F
        return x, y, z

    #__>        PRIVATE METHODS     <__#
    
    def __handler(self, pin:Pin) -> None:
        if (not self.__intr is None) and (not self.__callback is None):
            self.__callback(self.data)
            
    def __set_clock(self, source:int) -> None:
        self.__writeBits(0x6b, 0x2, 0x3, source)
        
    def __set_accel_offsets(self, x:int, y:int, z:int) -> None:
        self.__writeWords(0x6, 1, x)
        self.__writeWords(0x8, 1, y)
        self.__writeWords(0xa, 1, z)

    def __set_gyro_offsets(self, x:int, y:int, z:int) -> None:
        self.__writeWords(0x13, 1, x)
        self.__writeWords(0x15, 1, y)
        self.__writeWords(0x17, 1, z)
        
    def __disable_tests(self) -> None:
        self.__disable_accel_tests()
        self.__disable_gyro_tests()
        
    def __enable_accel_tests(self) -> None:
        self.set_accel(ACCEL_FS_8)
        self.__writeBit(0x1c, 0x7, True)
        self.__writeBit(0x1c, 0x6, True)
        self.__writeBit(0x1c, 0x5, True)
        
    def __disable_accel_tests(self) -> None:
        self.set_accel(self.__accfs)
        self.__writeBit(0x1c, 0x7, False)
        self.__writeBit(0x1c, 0x6, False)
        self.__writeBit(0x1c, 0x5, False)
        
    def __enable_gyro_tests(self) -> None:
        self.set_gyro(GYRO_FS_500)
        self.__writeBit(0x1b, 0x7, True)
        self.__writeBit(0x1b, 0x6, True)
        self.__writeBit(0x1b, 0x5, True)
        
    def __disable_gyro_tests(self) -> None:
        self.set_gyro(self.__gyrofs)
        self.__writeBit(0x1b, 0x7, False)
        self.__writeBit(0x1b, 0x6, False)
        self.__writeBit(0x1b, 0x5, False)
        
    def __enable_sleep(self, e:bool) -> None:
        self.__writeBit(0x6b, 0x6, e)

    def __fifo_bytes(self, length:int) -> bytearray:
        return self.__readBytes(0x74, length) if length > 0 else None
        
    def __enable_fifo(self, e:bool) -> None:
        self.__writeBit(0x6a, 0x6, e)
        
    def __enable_fifo_regs(self, e:bool) -> None:
        self.__writeByte(0x23, (0x78 if e else 0x00))
    
    def __reset_fifo(self) -> None:
        self.__writeBit(0x6a, 0x2, True)
        self.__enable_fifo_regs(False)
        
    def __reset_dmp(self) -> None:
        self.__writeBit(0x6a, 0x3, True)
    
    def __enable_dmp(self, e:bool) -> None:
        self.__writeBit(0x6a, 0x7, e)
        
    def __full_reset_fifo(self) -> None:
        self.__writeBit(0x6a, 0x6, False)
        self.__writeBit(0x6a, 0x2, True)
        self.__writeBit(0x6a, 0x6, True)
		  
    def __enable_interrupts(self, e:bool) -> None:
        self.__writeByte(0x38, (0x11 if e else 0x00))
            
    def __map(self:int, x:int, in_min:int, in_max:int, out_min:int, out_max:int) -> int:
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def __calibrate_gyro(self, loops:int) -> None:
        kP, kI = 0.3, 90
        x = (100 - self.__map(loops, 1, 5, 20, 0)) * .01
        kP *= x
        kI *= x
        self.__pid(0x43,  kP, kI, loops)
    
    def __calibrate_accel(self, loops:int) -> None:
        kP, kI = 0.3, 20
        x = (100 - self.__map(loops, 1, 5, 20, 0)) * .01
        kP *= x
        kI *= x
        self.__pid(0x3b, kP, kI, loops)

    def __pid(self, readaddr:int, kP:float, kI:float, loops:int) -> None:
        saveaddr = 0x06 if readaddr == 0x3B else 0x13
        shift    = 3    if saveaddr == 0x77 else 2
        data, reading, bitzero = 0  , 0.0, [0]*3
        esample, esum          = 0  , 0
        error, pterm, iterm    = 0.0, 0.0, [0.0]*3
        
        for i in range(3):
            data = self.__readWords(saveaddr + (i * shift), 1)[0]
            reading = data
            if not (saveaddr == 0x13):
                bitzero[i] = data & 1
                iterm[i]   = float(reading) * 8
            else:
                iterm[i]   = float(reading) * 4
        
        for L in range(loops):
            esample = 0
            for c in range(100):
                esum = 0
                for i in range(3):
                    data    = self.__readWords(readaddr + (i * 2), 1)[0]
                    reading = data
                    
                    if ((readaddr == 0x3B) and (i == 2)):
                        reading -= 16384
                        
                    error     = -reading
                    esum     += abs(reading)
                    pterm     = kP * error
                    iterm[i] += (error * 0.001) * kI
                    			
                    if saveaddr != 0x13:
                        data = round((pterm + iterm[i]) / 8)		
                        data = (data & 0xFFFE) | bitzero[i]			
                    else:
                        data = round((pterm + iterm[i]) / 4)
                        
                    self.__writeWords(saveaddr + (i * shift), 1, data)
                
                c = 0 if (c == 99) and (esum > 1000) else c
                
                if (esum * (.05 if readaddr == 0x3B else 1)) < 5: 
                    esample+= 1	
                if (esum < 100) and (c > 10) and (esample >= 10):
                     break		
                utime.sleep_ms(1)
            
            kP *= .75
            kI *= .75
            for i in range(3):
                if not (saveaddr == 0x13):
                    data = round(iterm[i] / 8)		
                    data = (data & 0xFFFE) | bitzero[i]
                else:
                    data = round(iterm[i] / 4)
                    
                self.__writeWords(saveaddr + (i * shift), 1, data)
            
        self.__reset_fifo()
        self.__reset_dmp()
        
    def __get_fifo_packet(self, length:int=12) -> bytearray:
        fifoC = self.__fifo_count
        extra = fifoC%length
        self.__fifo_bytes(extra)
        fifoC = self.__fifo_count
        return None if fifoC < length else struct.unpack(">hhhhhh", self.__fifo_bytes(fifoC)[-length:])
        
        
        
         
