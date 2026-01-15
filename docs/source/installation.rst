Installation
============

Hardware
---------
* Single Board Computer: Raspberry Pi 4B 4 or 8 GB of RAM with recommended Raspberry Pi OS
* SD Card: Minimum 32 GB (better 64 or more)
* Audio Interface: `MCHStreamer Kit <https://www.minidsp.com/products/usb-audio-interface/mchstreamer>`_ or `MCHStreamer Lite <https://www.minidsp.com/products/usb-audio-interface/mchstreamer-lite>`_

* Robot Platform: `THYMIO II robot <https://www.thymio.org/>`_
* Digital Microphones: 5 `Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H <https://www.adafruit.com/product/3421?srsltid=AfmBOopF9a1RzkEGnyDwYi1QTJkdfkqmZ2yrh4Vcfi8nLYe32wcLwwz3>`_
* Breadboard and Jumper Wires for connecting the microphones to the audio interface
* Loudspeaker: `CE32A-4 1-1/4" Mini Speaker Driver 4 Ohm <https://www.daytonaudio.com/product/1138/ce32a-4-1-1-4-mini-speaker-driver-4-ohm>`_ or equivalent
* DAC: `Adafruit I2S 3W Class D Amplifier Breakout - MAX98357A <https://www.adafruit.com/product/3006?srsltid=AfmBOoqIih8R59hlDWWSD45eMgPyGc9iYt6CyZhOlmSevV1-N-GBWHgb>`_ or equivalent `SparkFun I2S Audio Breakout - MAX98357A <https://www.sparkfun.com/sparkfun-i2s-audio-breakout-max98357a.html>`_
* Powerbank to power the Raspberry Pi: suggested 5000mAh or more min 2A out

Sofware
-------
1. Install Raspberry Pi OS (Raspbian 64 bit OS) on the SD card using `Raspberry Pi Imager <https://www.raspberrypi.com/software/>`_:
    - Enable SSH
    - Enable VNC

        ``sudo raspi-config``
    
2. Create a virtual environment with Conda or other tool of your choice:
    - Install miniconda:
        
        ``wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh``

        ``bash Miniforge3-Linux-aarch64.sh``

    - Create conda environment: (if not active run: ``source ~/miniforge/bin/activate``)
        
        ``conda create -n <env_name> python=3.11.2``
3. Install the required python packages:
    
    ``pip install -r requirements.txt``

4. Install specific packages if needed:
    - PortAudio:
        
        ``sudo apt-get install libasound-dev libportaudio2 portaudio19-dev``
    - Thymiodirect (from github):
        
        ``pip install thymiodirect@git+https://github.com/epfl-mobots/thymio-python``
5. Clone the repository:
    
    ``git clone https://github.com/activesensingcollectives/SonoRo_swarm.git``

6. Install and configure the audio interface:
    - Follow the `instructions <https://www.minidsp.com/log-in-register>`_ and set up the soundcard to work with I2S mics.


Assembly and Setup:
-------------------
- **Assembly**:
    1. Connect the Raspberry Pi to the MCHStreamer audio interface via USB.
    2. Connect the MCHStreamer to the Raspberry Pi via USB.
    3. Connect the DAC to the MCHStreamer with jumper wires (I2S connection).
    4. Connect the Loudspeaker to the DAC.
    5. Connect the 5 I2S microphones to the audio interface according to the provided wiring diagram, using the breadboard and jumper wires.
    6. Connect the Thymio robot to the Raspberry Pi via USB.
    7. Insert SD card into the Raspberry Pi.
    8. Power on the Raspberry Pi using the powerbank (All the other components are powered from the Raspberry Pi).
- **Setup**:
    1. Connect to the Pi via SSH or VNC.
    2. Follow the software installation steps above from step 2.
    3. Go to the :doc:`usage` section to learn how to run the robot. 



