import spidev
import RPi.GPIO as GPIO
import time
import statistics

GPIO0=11
# 使用標準的物理針位編號
GPIO.setmode(GPIO.BOARD)
# 設定 Pin 12 為 PWM 輸出
# GPIO.setup(0, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GPIO0, GPIO.OUT)
# 將 Pin 11 設定為 LOW
GPIO.output(GPIO0, GPIO.LOW)
# time.sleep(1)
GPIO.output(GPIO0, GPIO.HIGH)
time.sleep(1)

# 建立 SPI 物件
spi = spidev.SpiDev()
# 開啟 SPI 連接，0 表示 SPI port，1 表示 device (CS pin)
spi.open(0, 1)
# 設定 SPI 速度和模式
spi.max_speed_hz = 10000000
spi.mode = 0

def read_reg(addr):
    return spi.xfer2([addr, 0, 0,0,0,0,0])
def write_reg(addr, data1,data2,data3):
    return spi.xfer2([addr | 0x40, data1,data2,data3])


# 設定 TDC7200 的設定值

def getTime1():
    a = write_reg(0,1,0,0)
    while read_reg(0)[1] != 0:
        pass
    c = read_reg(0x10)
    return c[1] *65536 + c[2] * 256 + c[3]
    # print(data)
n=0


d_list = []
while True:
    d = getTime1()
    d_list.append(d)
    n = n + 1

    if n % 1000 == 0:
        average = statistics.mean(d_list)
        standard_deviation = statistics.stdev(d_list)
        print("Average:", average)
        print("Standard Deviation:", standard_deviation)
        n = 1
        d_list = []
    
    # time.sleep(0.01)
# 關閉 SPI 連接
spi.close()
GPIO.cleanup()
