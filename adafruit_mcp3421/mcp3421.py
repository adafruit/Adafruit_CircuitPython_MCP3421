# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_mcp3421`
================================================================================

CircuitPython driver for the MCP3421 analog to digital converter


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit MCP3421 18-Bit ADC: <https://adafruit.com/product/5870>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MCP3421.git"


class MCP3421:
    """Adafruit MCP3421 ADC driver"""

    MCP3421_GAIN = {
        1: 0b00,  # Gain set to 1X
        2: 0b01,  # Gain set to 2X
        4: 0b10,  # Gain set to 4X
        8: 0b11,  # Gain set to 8X
    }

    MCP3421_RESOLUTION = {
        12: 0b00,  # Resolution set to 12-bit (240 SPS)
        14: 0b01,  # Resolution set to 14-bit (60 SPS)
        16: 0b10,  # Resolution set to 16-bit (15 SPS)
        18: 0b11,  # Resolution set to 18-bit (3.75 SPS)
    }
    _gain = 1
    _resolution = 14
    _unused = 0b00  # Unused bits, defaulting to 0
    _ready = 0b0  # Ready bit, defaulting to 0

    # pylint: disable=too-many-arguments
    def __init__(
        self, i2c, address=0x68, gain=None, resolution=None, continuous_mode=True
    ) -> None:
        """Initialization over I2C

        :param int address: I2C address (default 0x68)
        :param int gain: Select 1X, 2X, 4X or 8X gain (defaults to 1X)
        :param int resolution: Select 12, 14, 16 or 18 bit resolution (defaults to 12-bit)
        :param bool continuous_mode: Select continuous sampling or one shot sampling
        """
        self.i2c_device = I2CDevice(i2c, address)
        if gain is not None:
            self._gain = gain

        if resolution is not None:
            self._resolution = resolution

        self._mode = continuous_mode

        self.gain = self._gain
        self.resolution = self._resolution
        self.mode = self._mode

        self.adc_data = bytearray(4)

    def _read_data(self):
        buffer = bytearray(4)  # Buffer to read data
        with self.i2c_device as device:
            try:
                device.readinto(buffer)
            except Exception as error:
                raise OSError(f"{error}") from error

            # Extract ADC data
            self.adc_data = buffer[:3]

            # Update the configuration from the fourth byte for 18-bit resolution
            if (buffer[3] & 0b11) == self.MCP3421_RESOLUTION[18]:
                self._update_config(buffer[3])
            else:  # For 12, 14, or 16-bit resolutions, the config byte is the third byte
                self._update_config(buffer[2])

    def _update_config(self, config_byte):
        self._gain = (config_byte >> 6) & 0b11
        self._resolution = (config_byte >> 4) & 0b11
        self._mode = (config_byte >> 3) & 0b1
        self._unused = (config_byte >> 1) & 0b11
        self._ready = config_byte & 0b1

    @property
    def register_value(self) -> int:
        """
        Combine all fields into a single byte

        :rtype: int
        """
        return (
            (self._gain << 6)
            | (self._resolution << 4)
            | (self._mode << 3)
            | (self._unused << 1)
            | self._ready
        )

    @property
    def gain(self) -> int:
        """
        The current gain setting from the device

        :rtype: int
        """
        try:
            self._read_data()  # Update data and check if read was successful
            return self._gain
        except Exception as error:
            raise OSError(f"Failed to read gain from device: {error}") from error

    @gain.setter
    def gain(self, value: int) -> None:
        if value not in self.MCP3421_GAIN:
            raise ValueError("Invalid gain value")
        gain_value = self.MCP3421_GAIN[value]
        self._gain = gain_value
        config_byte = self.register_value
        with self.i2c_device as device:
            device.write(bytes([config_byte]))

    @property
    def resolution(self) -> int:
        """
        The current resolution setting from the device

        :rtype: int
        """
        try:
            self._read_data()  # Update data and check if read was successful
            return self._resolution
        except Exception as error:
            raise OSError(f"Failed to read resolution from device: {error}") from error

    @resolution.setter
    def resolution(self, value: int) -> None:
        if value not in self.MCP3421_RESOLUTION:
            raise ValueError("Invalid resolution value")
        resolution_value = self.MCP3421_RESOLUTION[value]
        self._resolution = resolution_value
        config_byte = self.register_value
        with self.i2c_device as device:
            device.write(bytes([config_byte]))

    @property
    def mode(self) -> bool:
        """
        Current mode setting from the device

        :rtype: bool
        """
        try:
            self._read_data()  # Update data and check if read was successful
            return self._mode
        except Exception as error:
            raise OSError(f"Failed to read mode from device: {error}") from error

    @mode.setter
    def mode(self, value: bool) -> None:
        self._mode = value
        config_byte = self.register_value
        with self.i2c_device as device:
            device.write(bytes([config_byte]))

    def read(self) -> int:
        """ADC value

        :return: ADC value
        :rtype: int
        """
        try:
            self._read_data()
        except Exception as error:
            raise OSError(f"Failed to read from device: {error}") from error

        adc_value = 0
        if self._resolution in [
            self.MCP3421_RESOLUTION[12],
            self.MCP3421_RESOLUTION[14],
            self.MCP3421_RESOLUTION[16],
        ]:
            # Directly cast the first two bytes to int16_t
            adc_value = (self.adc_data[0] << 8) | self.adc_data[1]
            if (
                self.adc_data[0] & 0x80
            ):  # Check if the top bit is set for sign extension
                adc_value -= 0x10000
        elif self._resolution == self.MCP3421_RESOLUTION[18]:
            # Use all three bytes, considering the differential nature
            adc_value = (
                (self.adc_data[0] << 16) | (self.adc_data[1] << 8) | self.adc_data[2]
            )
        if self.adc_data[0] & 0x02:  # Extend the sign if the top bit is set
            adc_value |= 0xFF000000

        return adc_value
