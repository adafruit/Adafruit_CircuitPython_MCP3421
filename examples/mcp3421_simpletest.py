# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_mcp3421.mcp3421 as ADC
from adafruit_mcp3421.analog_in import AnalogIn

i2c = board.I2C()

adc = ADC.MCP3421(i2c)
adc_channel = AnalogIn(adc)

while True:
    print(adc_channel.value)
    time.sleep(0.01)
