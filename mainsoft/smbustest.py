import smbus
import time

#SMBusの引数に1を指定する。Raspberry Piのi2cバスの番号
i2c = smbus.SMBus(1)
#デバイスのアドレス 0x68
addr = 0x29

#1バイト データの書き込み
#コマンドフォーマット　アドレス　書き込みたいデータのアドレス　書き込むデータ
i2c.write_byte_data(addr, 0x06, 0xF0)

#複数バイト　データ書き込み
#コマンドフォーマット　アドレス　書き込みたいデータのアドレス　書き込むデータ(配列)
#i2c.write_i2c_block_data(addr, 0x07, [0x02, 0x01])

#1バイト　データ読み込み　
#コマンドフォーマット　アドレス　読み込みたいデータのアドレス
#data = i2x.read_byte_data(addr, 0x05)
#print(data)

#複数バイト　データ読み込み
#コマンドフォーマット　アドレス　読み込みたいデータのアドレス　データ数
#data = i2c.read_i2c_block_data(addr, 0x01, 1)
#print(data)