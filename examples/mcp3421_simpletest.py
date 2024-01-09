# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

import time
import board
import adafruit_mcp3421

i2c = board.I2C()

adc = adafruit_mcp3421.Adafruit_MCP3421(i2c)

while True:
    print(adc.value)
    time.sleep(1)
