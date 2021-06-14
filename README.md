# upy-motion


A simple MPU6050 driver written in micropython. This driver should be compatible with any micropython device. This driver does not support quaternion, but it does have a Kalman filter built in. The Kalman filter is automatically applied to the angle property, but (through a constructor argument) it can be applied to the raw accelerometer and gyroscope data, as well.


### Community:

_To officially file a bug report or feature request you can use these templates:_   [bug report](https://github.com/OneMadGypsy/upy-motion/blob/main/.github/ISSUE_TEMPLATE/bug_report.md) | [feature request](https://github.com/OneMadGypsy/upy-motion/blob/main/.github/ISSUE_TEMPLATE/feature_request.md)

_To discus features, bugs or share your own project that utilize code in this repo:_   [join the discussion](https://github.com/OneMadGypsy/upy-motion/discussions/1)

<br />

-------

<br />

## Ports:

### MPU6050.py
>This can be uploaded directly to the board, but is intended to be used as a frozen module. For information regarding how to setup the sdk and freeze a module you can refer to [this post](https://www.raspberrypi.org/forums/viewtopic.php?f=146&t=306449#p1862108) on the Raspberry Pi forum.


### MPU6050.mpy
>This is a cross-compiled version of `MPU6050.py`. It is intended to be uploaded to your board as you would any normal `.py` script.


<br />

-------

<br />

## Docs:


**MPU6050(`bus`, `sda`, `scl`, `ofs`, `intr`, `callback`, `gyro`, `accel`, `rate`, `dlpf`, `filtered`, `anglefilter`, `R`, `Q`, `A`, `angles`, `addr`, `freq`)**
> Main MPU6050 interface. It is only necessary to provide an interrupt pin and callback if you intend to use FIFO. If you do not provide an `ofs` argument the device will auto calibrate. The `angles` argument is used to tell the script to provide angles in the FIFO callback instead of axis data.


Arg             | Type       | Description                                    | Default
----------------|------------|------------------------------------------------|-------------
**bus**         | int        | I2C bus id                                     | **REQUIRED**  
**sda**         | int or Pin | data pin                                       | **REQUIRED**
**scl**         | int or Pin | clock pin                                      | **REQUIRED**
**ofs**         | tuple      | axis offsets                                   | None
**intr**        | int or Pin | interrupt pin                                  | None
**callback**    | function   | function to call on interrupt                  | None
**gyro**        | int        | gyroscope full scale range                     | GYRO_FS_500
**accel**       | int        | accelerometer full scale range                 | ACCEL_FS_2
**rate**        | int        | sample rate                                    | 4
**dlpf**        | int        | digital low-pass filter                        | DLPF_BW_188
**filtered**    | int        | which properties to filter                     | NONE
**anglefilter** | int        | which filters to apply to angles               | NONE
**R**           | float      | Kalman filter measure                          | 0.003
**Q**           | float      | Kalman filter bias                             | 0.001
**A**           | float      | complementary filter alpha                     | .8
**angles**      | int        | return `angles` instead of `data` (FIFO only)  | False
**addr**        | int        | device I2C address                             | 0x68
**freq**        | int        | I2C frequency                                  | 400000

<br />

### Properties:

<br />

**.device_id**
>The id of the device

<br />

**.connected**
>True or False device is connected

<br />

**.data**
>Returns gyroscope and accelerometer data. This data may be filtered with a Kalman filter if the appropriate flag is supplied to the `filtered` argument in the constructor. The [filters](https://github.com/OneMadGypsy/upy-motion/blob/main/README.md#filters) section contains more information on how to use filters. This is a `namedtuple` with the following fields

Field       | Type  |  Description
------------|-------|-----------------
**.acc_x**  | float | accelerometer x
**.acc_y**  | float | accelerometer y
**.acc_z**  | float | accelerometer z
**.gyro_x** | float | gyroscope x
**.gyro_y** | float | gyroscope y
**.gyro_z** | float | gyroscope z

<br />

**.angles**
>Returns angles concocted from accelerometer data. These angles mey be filtered (with Kalman, complementar or both) according to the flag supplied for the `anglefilter` argument in the constructor. The [filters](https://github.com/OneMadGypsy/upy-motion/blob/main/README.md#filters) section contains more information on how to use filters. This is a `namedtuple` with the following fields

Field       | Type  |  Description
------------|-------|-----------------
**.roll**   | float | roll angle
**.pitch**  | float | pitch angle

<br />

**.passed_self_test**
>True or False passed system self-test

<br />

**.celsius**
>Returns the temperature in Celsius

<br />

**.fahrenheit**
>Returns the temperature in Fahrenheit

<br />

### Methods:

<br />

**.start()**
>If an interrupt pin and callback were supplied to the constructor, this will start FIFO interrupts

<br />

**.stop()**
>If an interrupt pin and callback were supplied to the constructor, this will stop FIFO interrupts

<br />

**.set_dlpf(`dlpf:int`)**
>Sets the digital low-pass filter. Possible values include the following

 Const          | Value
----------------|-------
**DLPF_BW_256** | 0x00
**DLPF_BW_188** | 0x01
**DLPF_BW_98**  | 0x02
**DLPF_BW_42**  | 0x03
**DLPF_BW_20**  | 0x04
**DLPF_BW_10**  | 0x05
**DLPF_BW_5**   | 0x06

<br />

**.set_gyro(`rng:int`)**
>Sets the gyroscope fullscale range. Possible values include the following

 Const          | Value
----------------|-------
**GYRO_FS_250** | 0x00
**GYRO_FS_500** | 0x01
**GYRO_FS_1000**| 0x02
**GYRO_FS_2000**| 0x03

<br />

**.set_accel(`rng:int`)**
>Sets the accelerometer fullscale range. Possible values include the following

 Const          | Value
----------------|-------
**ACCEL_FS_2**  | 0x00
**ACCEL_FS_4**  | 0x01
**ACCEL_FS_8**  | 0x02
**ACCEL_FS_16** | 0x03

<br />

**.set_clock(`rng:int`)**
>Sets the clock. The default clock is `CLK_PLL_XGYRO`. The documents recommend that you use one of the gyro clocks. Possible values include the following

 Const            | Value
------------------|-------
**CLK_INTERNAL**  | 0x00
**CLK_PLL_XGYRO** | 0x01
**CLK_PLL_YGYRO** | 0x02
**CLK_PLL_ZGYRO** | 0x03
**CLK_PLL_EXT32K**| 0x04
**CLK_PLL_EXT19M**| 0x05
**CLK_KEEP_RESET**| 0x07

<br />

**.set_rate(`rate:int`)**
>Sets the sample rate. The argument can be between 1 and 255. The default is 4. If a complementary filter is applied to angles, the amount of samples that are taken will be half of this number, and the returned value will be an average of those samples.

<br />

**.print_data()**
>Prints the gyroscope and accelerometer data with any flagged filters automatically applied

<br />

**.print_from_data(`data:tuple`)**
>Prints the gyroscope and accelerometer data that was passed to it

<br />

**.print_angles()**
>Prints the roll and pitch angles with any flagged filters automatically applied

<br />

**.print_from_angles(`angles:tuple`)**
>Prints the angle data that was passed to it

<br />

**.print_celsius()**
>Prints the temperature in celsius

<br />

**.print_fahrenheit()**
>Prints the temperature in fahrenheit

<br />

**.print_offsets()**
>Prints the offsets as a line of code to be used as the `ofs` argument when instantiating `MPU6050`

<br />

**.print_all()**
>Prints everything except offsets

<br />

-------

<br />

## Usage

All the below examples can be copy/pasted, but you must make sure to provide the `bus`, `sda` and `scl` pins (or pin numbers) that apply to your wiring scheme. If an interrupt pin or calibration offsets are used in an example, those too must be replaced with the data that applies to you.

<br />

#### calibration

>The very first thing you should do is run the below script, When it completes it will print a small line of code that you need to copy and paste for use with the `ofs` argument. Failure to provide an `ofs` argument will result in your device auto-calibrating every time you instance it. Make sure your device is as flat and level as you can get it before running calibration. Only run calibration from a fresh power-up of the device. If you do a good job calibrating, the numbers this returns can be used constantly, and you should not need to calibrate again.

```python
from mpu6050 import MPU6050

MPU6050(1, 6, 7)
```
<br />

#### FIFO

>Supplying an interrupt pin and a callback will trigger the driver to use FIFO automatically. You must also call `start()` for interrupts to begin. The `data` argument in the handler will contain all of the accelerometer and gyroscope data, unless you set the `angles` constructor argument to `True`, in which case `data` will then contain angles values. 

```python
from mpu6050 import MPU6050

def handler(data:tuple):
    if 'mpu' in globals():
        print('[{:<16}] {:<10.2f}'.format('TEMPERATURE', mpu.fahrenheit))
        mpu.print_from_data(data)

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51), 2, handler)
if mpu.passed_self_test:
    mpu.start()
```

<br />

#### polling
>If an interrupt pin and callback are not supplied the driver assumes you want to manage your own polling
```python
from mpu6050 import MPU6050
import utime

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51))

if mpu.passed_self_test:
    while True:
        print('[{:<16}] {:<10.2f}'.format('TEMPERATURE', mpu.fahrenheit))
        mpu.print_data()
        utime.sleep_ms(100)
```

<br />

#### accessing data
>data is a `namedtuple` and can be used like any other `namedtuple`. You can either unpack the properties or use them directly. The below examples illustrate both of these methods.

```python
from mpu6050 import MPU6050
import utime

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51))

if mpu.passed_self_test:
    while True:
        ax, ay, az, gx, gy, gz = mpu.data
```


_or_


```python
from mpu6050 import MPU6050

def handler(data:tuple):
    if 'mpu' in globals():
        asum = data.acc_x  + data.acc_y  + data.acc_z
        gsum = data.gyro_x + data.gyro_y + data.gyro_z

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51), 2, handler)
if mpu.passed_self_test:
    mpu.start()
```

<br />

#### angles
>`angles` are handled no different than `data`

```python
from mpu6050 import MPU6050

def handler(data:tuple):
    if 'mpu' in globals():
        roll, pitch = mpu.angles

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51), 2, handler)
if mpu.passed_self_test:
    mpu.start()
```


_just like `data`, `angles` has its own print method, as well_


```python
from mpu6050 import MPU6050
import utime

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51))

if mpu.passed_self_test:
    while True:
        mpu.print_angles()
        utime.sleep_ms(100)
```

_when using fifo you can tell the script to send `angles` instead of axis `data` to the handler callback by setting the `angles` constructor argument to `True`_


```python
from mpu6050 import MPU6050

def handler(data:tuple):
    if 'mpu' in globals():
        roll, pitch = data
        mpu.print_from_angles(data)

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51), 2, handler, angles=True)
if mpu.passed_self_test:
    mpu.start()

```

<br />

#### filters
>This driver supports 2 different types of filters (Kalman and complementary). Complementary filters can only be applied to angles. If a complementary filter is flagged on angles it will return the average of all the samples taken. The amount of samples that are taken will be half of the `rate` argument that was supplied to the constructor.

<br />

>Applying filters to various pieces of data can be done with the below flags

Flag             | Value
-----------------|-------
**NONE**         | 0x00
**FILTER_ACCEL** | 0x01
**FILTER_GYRO**  | 0x02
**FILTER_ANGLES**| 0x04
**FILTER_ALL**   | 0x07


<br />

>Determining which filters are applied to angles can be done with the following flags

 
Flag             | Value
-----------------|-------
**NONE**         | 0x00
**ANGLE_KAL**    | 0x01
**ANGLE_COMP**   | 0x02
**ANGLE_BOTH**   | 0x03

<br />

**filter example**

```python
from mpu6050 import MPU6050, FILTER_GYRO, FILTER_ANGLES, ANGLE_COMP

def handler(data:tuple):
    if 'mpu' in globals():
        mpu.print_from_angles(data)
        
cfg = dict(
    rate        = 20,                          #MPU6050_SPLRTDIV ~ comp filter samples at half of this number
    filtered    = FILTER_GYRO | FILTER_ANGLES, #wont filter accelerometer raw readings
    anglefilter = ANGLE_COMP,                  #apply only complementary filter to angles
    angles      = True                         #send data to handler as angles
)
mpu = MPU6050(1, 6, 7, (1368, -1684, 416, 20, -6, 49), 2, handler, **cfg)

if mpu.passed_self_test:
    mpu.start()

```
