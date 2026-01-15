Installation
============

Hardware:
---------

Single Board Computer: Raspberry Pi 4B 4 or 8 GB of RAM with recommended Raspberry Pi OS 
SD Card: Minimum 32 GB (better 64 or more)
Audio Interface: `MCHStreamer Kit <https://www.minidsp.com/products/usb-audio-interface/mchstreamer>` or `MCHStreamer Lite <https://www.minidsp.com/products/usb-audio-interface/mchstreamer-lite>`
Robot Platform: `THYMIO II robot <https://www.thymio.org/>`
Digital Microphones: 5 `Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H <https://www.adafruit.com/product/3421?srsltid=AfmBOopF9a1RzkEGnyDwYi1QTJkdfkqmZ2yrh4Vcfi8nLYe32wcLwwz3>`
Breadboard and Jumper Wires for connecting the microphones to the audio interface
Loudspeaker: `CE32A-4 1-1/4" Mini Speaker Driver 4 Ohm <https://www.daytonaudio.com/product/1138/ce32a-4-1-1-4-mini-speaker-driver-4-ohm>` or equivalent
DAC: `Adafruit I2S 3W Class D Amplifier Breakout - MAX98357A <https://www.adafruit.com/product/3006?srsltid=AfmBOoqIih8R59hlDWWSD45eMgPyGc9iYt6CyZhOlmSevV1-N-GBWHgb>`
Powerbank to power the Raspberry Pi: suggested 5000mAh or more min 2A out

Sofware
-------

1. Install Raspberry Pi OS on the SD card using `Raspberry Pi Imager <https://www.raspberrypi.com/software/>`
    - Enable SSH
    - Enable VNC:
        ```sudo raspi-config````
2. Create a virtual environment with Conda or other tool of your choice
3. Install the required python packages:
    ``` bash
        pip install -r requirements.txt ```
4.

.. Logic Overview
.. ----------------

.. By running  `SonoRo_swarm.py` on your SonoRo robots, AudioProcessor class starts streaming Direction of Arrival (DOA) of incoming audio to the robot and dB SPL values.

.. `SonoRo_swarm.py` then passes then to RobotMove class, which moves the robot's wheel accorsingly to perform the selected behaviour.

.. By selecting ``attraction`` behaviour, the robot will move towards the direction of the highest dB SPL value, while by selecting ``repulsion`` behaviour, the robot will move away from it.
.. The ``dynamic_movement`` parameter instead combined the two movements together using the selected dB SPL ``trigger_level`` and ``critical_level`` thresholds. When the dB SPL is above `critical` the threshold, the robot will move away from the sound source, while when it is above the `trigger` threshold, it will move towards it.

