from machine import Pin, I2C
from micropython import const
from collections import namedtuple
import struct, utime, math

_R2D            = 180/math.pi

NONE            = const(0x00)

#filter type flags
ANGLE_KAL       = const(0x01)
ANGLE_COMP      = const(0x02)
ANGLE_BOTH      = const(0x03)

#filter 'which' flags
FILTER_ACCEL    = const(0x01)
FILTER_GYRO     = const(0x02)
FILTER_ANGLES   = const(0x04)
FILTER_ALL      = const(0x07)

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

#clock
CLK_INTERNAL    = const(0x00)
CLK_PLL_XGYRO   = const(0x01)
CLK_PLL_YGYRO   = const(0x02)
CLK_PLL_ZGYRO   = const(0x03)
CLK_PLL_EXT32K  = const(0x04)
CLK_PLL_EXT19M  = const(0x05)
CLK_KEEP_RESET  = const(0x07)

   
# Data Structure
_D   = namedtuple('D', ('acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'))
_A   = namedtuple('A', ('roll', 'pitch'))

# Data Formatting
_SEP = '-'*60
_W   = (' ', '-')
_C   = '{}\n{}\n{}'
_F   = '\t{} failed Self-Test'
_OUT = '[{:<16}] x: {}{:<10.2f} y: {}{:<10.2f} z: {}{:<10.2f}'


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


#__> Kalman Filter
class __Filters(object):
    def __init__(self, R:float, Q:float, alpha:float) -> None:
        self.__cov = float('nan')
        self.__x   = float('nan')
        self.__c   = float('nan')
        self.__A, self.__B, self.__C = 1, 0, 1
        self.__R, self.__Q = R, Q
        
        self.__alpha = alpha
        self.__time  = utime.ticks_us()
        self.__delta = utime.ticks_diff(utime.ticks_us(), self.__time)/1000000

    def kalman(self, angle:float) -> float:
        u = 0
        if math.isnan(self.__x):
            self.__x   = (1 / self.__C) * angle
            self.__cov = (1 / self.__C) * self.__Q * (1 / self.__C)
        else:
            px         = (self.__A * self.__x) + (self.__B * u)
            pc         = ((self.__A * self.__cov) * self.__A) + self.__R
            K          = pc * self.__C * (1 / ((self.__C * pc * self.__C) + self.__Q))
            self.__x   = px + K * (angle - (self.__C * px))
            self.__cov = pc - (K * self.__C * pc)

        return self.__x
        
    def complementary(self, rate:float, angle:float) -> float:
        if math.isnan(self.__c):
            self.__c = angle
            
        self.__delta = utime.ticks_diff(utime.ticks_us(), self.__time)/1000000
        self.__time  = utime.ticks_us()
        self.__c     = (1-self.__alpha) * (self.__c + rate * self.__delta) + self.__alpha * angle
        return self.__c

#__> MPU6050
class MPU6050(__I2CHelper):
    
    #__>        PROPERTIES       <__#

    @property
    def device_id(self) -> int:
        return self.__readBits(0x75, 0x6, 0x6)
        
    @property
    def data(self) -> namedtuple:
        data = None
        ax, ay, az, gx, gy, gz = 0, 0, 0, 0, 0, 0
        if self.__usefifo:
            data = self.__get_fifo_packet(12)
            if not data is None:
                ax = data[0] / self.__accfact
                ay = data[1] / self.__accfact
                az = data[2] / self.__accfact
                gx = data[3] / self.__gyrofact 
                gy = data[4] / self.__gyrofact 
                gz = data[5] / self.__gyrofact 
            
        if (not self.__usefifo) or (data is None):
            data = struct.unpack(">hhhhhhh", self.__readBytes(0x3b, 14))
            ax = data[0] / self.__accfact 
            ay = data[1] / self.__accfact 
            az = data[2] / self.__accfact 
            gx = data[4] / self.__gyrofact
            gy = data[5] / self.__gyrofact
            gz = data[6] / self.__gyrofact
            
        if self.__filtered & (FILTER_GYRO | FILTER_ACCEL):
           if self.__filtered & FILTER_GYRO:
               gx = self.__fil_gx.kalman(gx)
               gy = self.__fil_gy.kalman(gy)
               gz = self.__fil_gz.kalman(gz)
                   
           if self.__filtered & FILTER_ACCEL:
               ax = self.__fil_ax.kalman(ax)
               ay = self.__fil_ay.kalman(ay)
               az = self.__fil_az.kalman(az)
            
        return _D(ax, ay, az, gx, gy, gz)
            
    @property
    def angles(self) -> tuple:
        if (self.__aftype & ANGLE_BOTH) and (self.__filtered & FILTER_ANGLES):
            return self.__filtered_angles()
            
        ax, ay, az, gx, gy, gz = self.data
        z2    = az**2
        roll  = math.atan(ax/math.sqrt(ay**2+z2))*_R2D
        pitch = math.atan(ay/math.sqrt(ax**2+z2))*_R2D
                
        return _A(roll, pitch)
           
    @property
    def int_angles(self) -> tuple:
        if (self.__aftype & ANGLE_BOTH) and (self.__filtered & FILTER_ANGLES):
            return self.__filtered_angles(True)
            
        ax, ay, az, gx, gy, gz = self.data
        z2    = az**2
        roll  = math.atan(ax/math.sqrt(ay**2+z2))*_R2D
        pitch = math.atan(ay/math.sqrt(ax**2+z2))*_R2D
                
        return _A(int(roll), int(pitch))
        
    @property
    def connected(self) -> bool:
        return self.device_id == 0x34
      
    @property
    def passed_self_test(self) -> bool:
        if self.connected:                          
            self.__enable_tests(True)                       #enable tests
            accel   = self.__test(*self.__accel_st_data)    #test accelerometer
            gyro    = self.__test(*self.__gyro_st_data)     #test gyroscope
            self.__enable_tests(False) #disble tests and revert to pre-test states
            if not (accel and gyro):
                if not gyro:
                    print(_F.format('Gyroscope'))
                if not accel:
                    print(_F.format('Accelerometer'))
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
    def __init__(self, bus:int, sda, scl, ofs:tuple=None, intr=None, callback=None, angles:bool=False, clock:int=CLK_PLL_XGYRO, gyro:int=GYRO_FS_500, accel:int=ACCEL_FS_2, dlpf:int=DLPF_BW_188, rate:int=4, filtered:int=NONE, anglefilter:int=NONE, R:float=0.003, Q:float=0.001, A:float=0.8, addr:int=0x68, freq:int=400000) -> None:
        super().__init__(bus, sda, scl, addr, freq)
        self.__accsense , self.__accfact , self.__accfs   = 0, 0, accel
        self.__gyrosense, self.__gyrofact, self.__gyrofs  = 0, 0, gyro
        self.__rate     , self.__dlpf                     = rate, dlpf
        self.__intr     , self.__usefifo                  = None, False
        self.__fil_r    , self.__fil_p                    = None, None
        self.__fil_gx   , self.__fil_gy  , self.__fil_gz  = None, None, None 
        self.__fil_ax   , self.__fil_ay  , self.__fil_az  = None, None, None
        self.__useangles, self.__filtered, self.__aftype  = angles, filtered, anglefilter
        
        if filtered & FILTER_ANGLES:
            self.__fil_r , self.__fil_p                   = __Filters(R, Q, A), __Filters(R, Q, A)
        if filtered & FILTER_GYRO:
            self.__fil_gx, self.__fil_gy, self.__fil_gz   = __Filters(R, Q, A), __Filters(R, Q, A), __Filters(R, Q, A)
        if filtered & FILTER_ACCEL:
            self.__fil_ax, self.__fil_ay, self.__fil_az   = __Filters(R, Q, A), __Filters(R, Q, A), __Filters(R, Q, A)
        
        self.__enable_interrupts(False)
        self.__enable_fifo (False)
        self.__enable_tests(False)
        self.__enable_sleep(False)
        
        self.__set_clock(clock)
        self.__set_gyro (gyro )
        self.__set_accel(accel)
        self.__set_rate (rate )
        self.__set_dlpf (dlpf )
        
        utime.sleep_ms(100)                     #a moment to stabilize
        if (filtered & (FILTER_ANGLES | FILTER_GYRO)) or (anglefilter & ANGLE_KAL):
            for _ in range(50): self.angles     #this primes the Kalman filters
        elif not anglefilter & NONE:
            for _ in range(10): self.angles     #primes delta for complimentary filter
        
        self.__time  = utime.ticks_us()
        self.__delta = utime.ticks_diff(utime.ticks_us(), self.__time)/1000000
        self.__cx, self.__cy = self.angles
        
        if isinstance(ofs, tuple):
            self.__set_offsets(*ofs) if (len(ofs) == 6) else self.__calibrate(6)
        else:
            self.__calibrate(6)
            
        if (not intr is None) and (not callback is None):
            self.__intr     = Pin(intr, Pin.IN, Pin.PULL_DOWN) if isinstance(intr, int) else Pin(sda)
            self.__intr.irq(self.__handler, Pin.IRQ_RISING)
            self.__callback = callback
            self.__usefifo  = True
            self.__reset_fifo()
            self.__enable_fifo(True)
            
    #__>        PUBLIC METHODS       <__#
    
    #INTERRUPT CONTROL____>
    def start(self) -> None:
        if not self.__usefifo is None:
            self.__enable_interrupts(True)
        
    def stop(self) -> None:
        if not self.__usefifo is None:
            self.__enable_interrupts(False)
         
    #PRINT________________>
    def print_offsets(self) -> None:
        ax, ay, az = self.__readWords(0x6   , 3)
        gx, gy, gz = self.__readWords(0x13, 3)
        print('ofs=({}, {}, {}, {}, {}, {})'.format(ax, ay, az, gx, gy, gz))
        
    def print_data(self):
        self.print_from_data(self.data)
        
    def print_from_data(self, data:tuple) -> None:
        ax, ay, az, gx, gy, gz = data
        a = _OUT.format('ACCELEROMETER', _W[ax<0], abs(ax), _W[ay<0], abs(ay), _W[az<0], abs(az))
        g = _OUT.format('GYROSCOPE'    , _W[gx<0], abs(gx), _W[gy<0], abs(gy), _W[gz<0], abs(gz))
        print(_C.format(a, g, _SEP))
      
    def print_angles(self, asint:bool=False) -> None:
        if asint:
            self.print_from_angles(self.int_angles)
            return
        self.print_from_angles(self.angles) 
        
    def print_from_angles(self, angles:tuple) -> None:
        r, p = angles[0:2]
        print('[{:<16}] roll: {}{:<10.2f} pitch: {}{:<10.2f}'.format('ANGLES', _W[r<0], abs(r), _W[p<0], abs(p)))
         
    def print_celsius(self) -> None:
        print('[{:<16}] {:>6.2f} C'.format('TEMPERATURE', self.celsius)) 
        
    def print_fahrenheit(self) -> None:
        print('[{:<16}] {:>6.2f} F'.format('TEMPERATURE', self.fahrenheit)) 
        
    def print_all(self):
        self.print_celsius()
        self.print_fahrenheit()
        self.print_angles()
        self.print_data()
        
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
    def __accel_st_data(self) -> tuple:             #self-test data
        a  = self.__readBit(0x1c, 0x7)
        b  = self.__readBit(0x1c, 0x6)
        c  = self.__readBit(0x1c, 0x5)
        v  = self.__readByte(0x10)	
        x_ = self.__readByte(0xd)
        x  = ((x_>>3) & 0x1C) | ((v>>4) & 0x03)
        y_ = self.__readByte(0xe)
        y  = ((y_>>3) & 0x1C) | ((v>>2) & 0x03) 
        z_ = self.__readByte(0xf)
        z  = ((z_>>3) & 0x1C) | (v & 0x03) 
        return (a, b, c), (x, y, z)
        
    @property
    def __gyro_st_data(self) -> tuple:              #self-test data
        a = self.__readBit(0x1b, 0x7)
        b = self.__readBit(0x1b, 0x6)
        c = self.__readBit(0x1b, 0x5)
        x = self.__readByte(0xd) & 0x1F
        y = self.__readByte(0xe) & 0x1F
        z = self.__readByte(0xf) & 0x1F
        return (a, b, c), (x, y, z)

    #__>        PRIVATE METHODS     <__#
    
    #SETTERS______________>
    def __set_dlpf(self, dlpf:int):
        self.__writeBits(0x1a, 0x2, 0x3, dlpf)
        
    def __set_rate(self, rate:int) -> None:
        self.__writeByte(0x19, rate)
        
    def __set_gyro(self, rng:int) -> None:
        self.__gyrosense = [250, 500, 1000, 2000][rng]
        self.__gyrofact  = [131, 66.5, 32.8, 16.4][rng]
        self.__writeBits(0x1b, 0x4, 0x2, rng)

    def __set_accel(self, rng:int) -> None:
        self.__accsense = [2, 4, 8, 16][rng]
        self.__accfact  = 32768.0/self.__accsense
        self.__writeBits(0x1c, 0x4, 0x2, rng)
     
    def __set_clock(self, source:int) -> None:
        self.__writeBits(0x6b, 0x2, 0x3, source)
        
    #MISC_________________>
    def __handler(self, pin:Pin) -> None:           #interrupt handler
        if (not self.__intr is None) and (not self.__callback is None):
            if self.__useangles:
                self.__callback(self.angles)
                return
            self.__callback(self.data)
           
    def __filtered_angles(self, asint:bool=False):                    #manages all angle filtering
        smps   = self.__rate//2
        fx, fy = [0.00]*(smps), [0.00]*(smps)
        for s in range(smps):
            ax, ay, az, gx, gy, gz = self.data
            z2    = az**2
            ay2z2 = math.sqrt(ay**2+z2)
            ax2z2 = math.sqrt(ax**2+z2)
            roll  = 0 if not ay2z2 else math.atan(ax/ay2z2)*_R2D
            pitch = 0 if not ax2z2 else math.atan(ay/ax2z2)*_R2D
            if self.__aftype & ANGLE_KAL:
                roll  = self.__fil_r.kalman(roll)
                pitch = self.__fil_p.kalman(pitch)
            if self.__aftype & ANGLE_COMP:
                roll  = self.__fil_r.complementary(gx, roll )
                pitch = self.__fil_p.complementary(gy, pitch)
            fx[s], fy[s] = roll, pitch
            utime.sleep_us(100)
        roll  = sum(fx)/smps
        pitch = sum(fy)/smps
        
        if asint:
            return _A(int(roll), int(pitch))
        
        return _A(roll, pitch)
             
    def __enable_sleep(self, e:bool) -> None:
        self.__writeBit(0x6b, 0x6, e)
       
    def __enable_interrupts(self, e:bool) -> None:
        self.__writeByte(0x38, (0x11 if e else 0x00))
        
    #SELF-TEST____________>
    def __enable_tests(self, e:bool) -> None:
        #accelerometer test
        self.__set_accel(ACCEL_FS_8 if e else self.__accfs)
        self.__writeBit(0x1c, 0x7, e)
        self.__writeBit(0x1c, 0x6, e)
        self.__writeBit(0x1c, 0x5, e)
        #gyroscope test
        self.__set_gyro(GYRO_FS_500 if e else self.__gyrofs)
        self.__writeBit(0x1b, 0x7, e)
        self.__writeBit(0x1b, 0x6, e)
        self.__writeBit(0x1b, 0x5, e)
       
    def __test(self, st_data:tuple, trim:tuple) -> bool:
        spec = range(-14, 15) #factory min/max specs
        return sum([True if not t else ((s-t)//t in spec) for t, s in zip(trim, st_data)]) == 3   
           
    #FIFO_________________>
    def __reset_fifo(self) -> None:
        self.__writeBit(0x6a, 0x6, False) #disable FIFO
        self.__writeBit(0x6a, 0x2, True)  #reset   FIFO
        self.__writeBit(0x6a, 0x6, True)  #enable  FIFO
		  
    def __enable_fifo(self, e:bool) -> None:
        self.__writeBit(0x6a, 0x6, e)                   #enable FIFO
        self.__writeByte(0x23, (0x78 if e else 0x00))   #enable Gyro and Accel registers
    
    def __fifo_bytes(self, length:int) -> bytearray:
        #read from FIFO buffer
        return self.__readBytes(0x74, length) if length > 0 else None
 
    def __get_fifo_packet(self, length:int=12) -> bytearray:
        fifoC = self.__fifo_count
        extra = fifoC%length
        self.__fifo_bytes(extra)
        fifoC = self.__fifo_count
        return None if fifoC < length else struct.unpack(">hhhhhh", self.__fifo_bytes(fifoC)[-length:])

    #DMP__________________>
    def __reset_dmp(self) -> None:
        self.__writeBit(0x6a, 0x3, True)
    
    def __enable_dmp(self, e:bool) -> None:
        self.__writeBit(0x6a, 0x7, e)

    #CALIBRATE____________>
    def __calibrate(self, loops:int) -> None:
        x = (100 - int((loops - 1) * (0 - 20) / (5 - 1) + 20)) * .01
        kP, kI = 0.3*x, 90*x
        self.__pid(0x43,  kP, kI, loops) #calibrate gyroscope
        kP, kI = 0.3*x, 20*x
        self.__pid(0x3b,  kP, kI, loops) #calibrate accelerometer
        self.print_offsets()

    def __pid(self, readaddr:int, kP:float, kI:float, loops:int) -> None:
        saveaddr = 0x06 if readaddr == 0x3B else 0x13
        data , reading, bitzero = 0  , 0.0, [0]*3
        shift, esample, esum    = 2  , 0  , 0
        error, pterm  , iterm   = 0.0, 0.0, [0.0]*3
        
        for i in range(3):
            data = self.__readWords(saveaddr + (i * shift), 1)[0]
            reading = data
            if not (saveaddr == 0x13):
                bitzero[i] = data & 1
                iterm[i]   = reading * 8
            else:
                iterm[i]   = reading * 4
        
        for L in range(loops):
            esample = 0
            for c in range(100):
                esum = 0
                for i in range(3):
                    data      = self.__readWords(readaddr + (i * 2), 1)[0]
                    reading   = data
                    
                    if ((readaddr == 0x3B) and (i == 2)): reading -= 16384
                        
                    error     = -reading
                    esum     += abs(reading)
                    pterm     = kP * error
                    iterm[i] += (error * 0.001) * kI
                    data      = (round((pterm + iterm[i]) / 8) & 0xFFFE) | bitzero[i] if saveaddr != 0x13 else round((pterm + iterm[i]) / 4)
                        
                    self.__writeWords(saveaddr + (i * shift), 1, data)
                
                c = 0 if (c == 99) and (esum > 1000) else c
                
                if (esum * (.05 if readaddr == 0x3B else 1)) < 5: esample+= 1	
                if (esum < 100) and (c > 10) and (esample >= 10): break		
                utime.sleep_ms(1)
            
            kP *= .75
            kI *= .75
            for i in range(3):
                data = (round(iterm[i] / 8) & 0xFFFE) | bitzero[i] if not (saveaddr == 0x13) else round(iterm[i] / 4)
                self.__writeWords(saveaddr + (i * shift), 1, data)
            
        self.__reset_fifo()
        self.__reset_dmp()
     
    def __set_offsets(self, ax:int, ay:int, az:int, gx:int, gy:int, gz:int) -> None:
        self.__writeWords(0x6 , 1, ax)
        self.__writeWords(0x8 , 1, ay)
        self.__writeWords(0xa , 1, az)
        self.__writeWords(0x13, 1, gx)
        self.__writeWords(0x15, 1, gy)
        self.__writeWords(0x17, 1, gz)

