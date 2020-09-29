# Packages needed for MiCS-2714
import spidev
import os

# Packages needed for PMS5003
import serial
from collections import OrderedDict

# Packages needed for BME280
import board
import busio
import adafruit_bme280

# Genearl packages
import csv
import time


def ReadChannel(channel, spi):
    """A/D Converter, to channel 0?."""
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data


def ConvertVolts(data, places):
    volts = (data * 3.3) / float(1023)
    volts = round(volts, places)
    return volts


def ConvertNO2(data, places):
    NO2 = ((data * 10.05) / float(1023)) + 0.05
    NO2 = round(NO2, places)
    return NO2


class PMS5003Sensor:
    """Read from the PMS5003 Sensor."""

    def __init__(self, tty="/dev/ttyS0"):
        self.tty = tty

    def open(self):
        self.serial = serial.Serial(self.tty, baudrate=9600)

    def close(self):
        self.serial.close()

    def read_bytes(self, data, idx, size=2):
        return int("".join(data[idx : idx + size]), 16)

    def read(self):
        data = self.serial.read(32)
        data = ["{:02X}".format(d) for d in data]

        if data[0] != "42" or data[1] != "4D":
            return None

        res = OrderedDict()
        res["pm1_cf"] = self.read_bytes(data, 4)
        res["pm25_cf"] = self.read_bytes(data, 6)
        res["pm10_cf"] = self.read_bytes(data, 8)
        res["pm1"] = self.read_bytes(data, 10)
        res["pm25"] = self.read_bytes(data, 12)
        res["pm10"] = self.read_bytes(data, 14)

        return res


def write_to_csv(NO2, data, bme280):
    with open("/home/pi/airquality_data.csv", mode="a") as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                NO2,
                data,
                bme280.temperature,
                bme280.humidity,
                bme280.pressure,
                bme280.altitude,
            ]
        )

    with open("/home/pi/airquality_data.csv", mode="r") as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            print(row)


if __name__ == "__main__":

    # MiCS-2714 Code (I think this for the ADC not the 2714
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1000000

    # BME280 Code
    i2c = busio.I2C(board.SCL, board.SDA)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    bme280.sea_level_pressure = 1013.25

    # PMS5003 Object
    pms5003 = PMS5003Sensor()

    NO2_CHANNEL = 0

    while True:

        # MiCS-2714 output
        NO2_level = ReadChannel(NO2_CHANNEL, spi)
        NO2_volts = ConvertVolts(NO2_level, 2)
        NO2 = ConvertNO2(NO2_level, 2)

        # Read from PMS5003
        pms5003.open()
        pms5003_data = pms5003.read()
        pms5003.close()

        # Write to csv, sleep for 10 seconds
        write_to_csv(NO2, pms5003_data, bme280)
        time.sleep(10)
