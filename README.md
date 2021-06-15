# upy-motion


An MPU6050 driver written in micropython. This driver should be compatible with any micropython device. This driver does not support quaternion

### Features:

1) Auto-calibration if the `ofs` argument is omitted
2) After auto-calibration the `ofs` argument is supplied to you
3) Automatic FIFO if an interrupt pin and callback are supplied
4) Kalman and complimentary filters are built in and automatically applied to data based on flags you set
5) Data can be retrieved as raw gyroscope and accelerometer data or as angles (roll, pitch only)
6) Temperature can be retrieved as Celsius or Fahrenheit
7) Numerous print options available that format the data into a very neat and easy-to-read display
8) Self-test is built in and using just one property will tell you if your device is functioning properly
9) Everything you can do is well-documented below


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


**MPU6050(`bus`, `sda`, `scl`, `ofs`, `intr`, `callback`, `angles`, `clock`, `gyro`, `accel`, `dlpf`, `rate`, `filtered`, `anglefilter`, `R`, `Q`, `A`, `addr`, `freq`)**

>The argument for this constructor may seem daunting, but it can be broken up into sections that make it far easier to manage. From `bus` through `scl` sets up I2C connection with the device. `ofs` is the configuration offsets that help your device function accurately. Examples are provided later in this document that explain how to get this value. From `intr` through `angles` are interrupt related and only used if you want to use FIFO. From `clock` through `rate` are device specific settings. From `filtered` through `A` are filter specific settings. `addr` and `freq` are the device address and the frequency it should run at. There is very little reason why you should ever have to change these, which is why they are the very last arguments.


>Most things are evidenced later in this document, but there are a couple of things that are easier to simply explain right here. Providing an interrupt pin and a callback will automatically trigger the script to use FIFO. The `rate` argument has 2 purposes. Whatever value you supply will be the divisor for gyroscope clock output, which is it's main intended purpose. However, my driver also uses half of this number to determine how many samples to average when using a complementary filter.


Arg             | Type       | Description                                    | Default
----------------|------------|------------------------------------------------|-------------
**bus**         | int        | I2C bus id                                     | **REQUIRED**  
**sda**         | int or Pin | data pin                                       | **REQUIRED**
**scl**         | int or Pin | clock pin                                      | **REQUIRED**
**ofs**         | tuple      | axis offsets                                   | None
**intr**        | int or Pin | interrupt pin (FIFO only)                      | None
**callback**    | function   | function to call on interrupt (FIFO only)      | None
**angles**      | int        | return `angles` instead of `data` (FIFO only)  | False
**clock**       | int        | clock source to use                            | CLK_PLL_XGYRO
**gyro**        | int        | gyroscope full scale range                     | GYRO_FS_500
**accel**       | int        | accelerometer full scale range                 | ACCEL_FS_2
**dlpf**        | int        | digital low-pass filter                        | DLPF_BW_188
**rate**        | int        | sample rate                                    | 4
**filtered**    | int        | which properties to filter                     | NONE
**anglefilter** | int        | which filters to apply to angles               | NONE
**R**           | float      | Kalman filter measure                          | 0.003
**Q**           | float      | Kalman filter bias                             | 0.001
**A**           | float      | complementary filter alpha                     | .8
**addr**        | int        | device I2C address                             | 0x68
**freq**        | int        | I2C frequency                                  | 400000

<br />

-------

<br />

## Constants:

<br />

#### clock
>Possible values for the `clock` constructor argument are the following. The default clock is `CLK_PLL_XGYRO`. The documents recommend that you use one of the gyro clocks. All clocks (except external) have their typical frequency listed. Actual frequency may vary +/- 3 Khz.

 Const            | Value | Frequency
------------------|-------|--------
**CLK_INTERNAL**  | 0x00  | 8 Mhz
**CLK_PLL_XGYRO** | 0x01  | 33 Khz
**CLK_PLL_YGYRO** | 0x02  | 30 Khz
**CLK_PLL_ZGYRO** | 0x03  | 27 Khz
**CLK_PLL_EXT32K**| 0x04  | 32.768 Khz
**CLK_PLL_EXT19M**| 0x05  | 19.2 Mhz
**CLK_KEEP_RESET**| 0x07  | 0

<br />

#### gyro
>Possible values for the `gyro` constructor argument are the following. The default gyro is `GYRO_FS_500`.

 Const          | Value 
----------------|-------
**GYRO_FS_250** | 0x00  
**GYRO_FS_500** | 0x01  
**GYRO_FS_1000**| 0x02  
**GYRO_FS_2000**| 0x03  

<br />

#### accel
>Possible values for the `accel` constructor argument are the following. The default accel is `ACCEL_FS_2`

 Const          | Value
----------------|-------
**ACCEL_FS_2**  | 0x00
**ACCEL_FS_4**  | 0x01
**ACCEL_FS_8**  | 0x02
**ACCEL_FS_16** | 0x03

<br />

#### dlpf
>Possible values for the `dlpf` constructor argument include the following. The default dlpf is `DLPF_BW_188`. Headers marked **ms** below represent the milliseconds of delay a DLPF will create.

Const           | Value | Accel(ms) | Gyro(ms) | FS (Khz)
----------------|-------|-----------|----------|---------
**DLPF_BW_256** | 0x00  | 0         |  0.98    | 8
**DLPF_BW_188** | 0x01  | 2.0       |  1.9     | 1
**DLPF_BW_98**  | 0x02  | 3.0       |  2.8     | 1
**DLPF_BW_42**  | 0x03  | 4.9       |  4.8     | 1
**DLPF_BW_20**  | 0x04  | 8.5       |  8.3     | 1
**DLPF_BW_10**  | 0x05  | 13.8      | 13.4     | 1
**DLPF_BW_5**   | 0x06  | 19.0      | 18.6     | 1

<br />

#### filtered
>Possible values for the `filtered` constructor argument include the following. The default filter is `NONE`. Applying one or more of these flags tells the driver which data to filter. For accel and gyro Kalman filters will be applied. For angles another flag must be used to determine which filters you want applied.

Flag             | Value
-----------------|-------
**NONE**         | 0x00
**FILTER_ACCEL** | 0x01
**FILTER_GYRO**  | 0x02
**FILTER_ANGLES**| 0x04
**FILTER_ALL**   | 0x07

<br />

#### anglefilter
>Possible values for the `anglefilter` constructor argument include the following. The default anglefilter is `NONE`. 

Flag             | Value
-----------------|-------
**NONE**         | 0x00
**ANGLE_KAL**    | 0x01
**ANGLE_COMP**   | 0x02
**ANGLE_BOTH**   | 0x03

<br />

-----------------

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

-----------------

<br />

### Methods:

<br />

**.start()**
>If an interrupt pin and callback were supplied to the constructor, this will start FIFO interrupts

<br />

**.stop()**
>If an interrupt pin and callback were supplied to the constructor, this will stop FIFO interrupts

<br />

**.print_data()**
>Prints the gyroscope and accelerometer data with any flagged filters automatically applied. The [filters](https://github.com/OneMadGypsy/upy-motion/blob/main/README.md#filters) section contains more information on how to use filters.

<br />

**.print_from_data(`data:tuple`)**
>Prints the gyroscope and accelerometer data that was passed to it

<br />

**.print_angles()**
>Prints the roll and pitch angles with any flagged filters automatically applied. The [filters](https://github.com/OneMadGypsy/upy-motion/blob/main/README.md#filters) section contains more information on how to use filters.

<br />

**.print_from_angles(`angles:tuple`)**
>Prints the angle data that was passed to it

<br />

**.print_celsius()**
>Prints the temperature in Celsius

<br />

**.print_fahrenheit()**
>Prints the temperature in Fahrenheit

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

>The very first thing you should do is calibrate the device. When it completes it will print a small line of code that you need to copy and paste for use with the `ofs` argument. Failure to provide an `ofs` argument will result in your device auto-calibrating every time you instance it. Make sure your device is as flat and level as you can get it before running calibration. Only run calibration from a fresh power-up of the device. If you do a good job calibrating, the numbers this returns can be used constantly, and you should not need to calibrate again unless you customize any of the device configuration arguments (ie. `clock`, `dlpf`, `rate`, `gyro` or `accel`). If you do intend to change any of the configuration arguments, you should add those changes to the script below before running it.

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

mpu = MPU6050(1, 6, 7, (1314, -1629, 410, 28, -17, 51), 2, handler, True)
if mpu.passed_self_test:
    mpu.start()

```

<br />

#### filters

>This driver supports 2 different types of filters (Kalman and complementary). Complementary filters can only be applied to angles. If a complementary filter is flagged on angles it will return the average of all the samples taken. The amount of samples that are taken will be half of the `rate` argument that was supplied to the constructor.


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
