from smbus import SMBus
import time

stm_main = 0x29
i2cbus = SMBus(1)
stm_sleep_time = 1

while True:
    i2cbus.write_byte(stm_main,20)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,30)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,40)
    time.sleep(stm_sleep_time)
    i2cbus.write_byte(stm_main,50)

    time.sleep(1)