Usage
=====

Overview
--------

By running  ``SonoRo_swarm.py`` on your SonoRo robots, ``AudioProcessor.py`` class starts streaming Direction of Arrival (DOA) of incoming audio to the robot and dB SPL values.

``SonoRo_swarm.py`` then passes to ``RobotMove.py`` class, which moves the robot's wheel accordingly to perform the selected behaviour.

Two `queue` objects (``angle_queue`` and ``level_queue``) run in separate threds and provide ``RobotMove`` with the latest DOA and dB SPL values calculated by ``AudioProcessor``.

By selecting ``attraction`` behaviour, the robot will move towards the direction of the highest dB SPL value, while by selecting ``repulsion`` behaviour, the robot will move away from it.
The ``dynamic_movement`` parameter instead combined the two movements together using the selected dB SPL ``trigger_level`` and ``critical_level`` thresholds.
When the dB SPL is above the `critical` threshold, the robot will move away from the sound source, while when it is above the `trigger` threshold, it will move towards it.


Audio Sensing
--------------
Audio sensing is performed by the ``AudioProcessor.py`` class, which uses mainly the ``instructions <>``sounddevice`` and ``soundfile`` libraries to access the sound card input (``AudioProcessor.input_stream``) and output stream (``AudioProcessor.output_stream``) and record raw mic data. The main worflow follows this structure:

- An output stream is opened to play a chirp through the sound card output (``AudioProcessor.output_stream``). 
- When the output is ended, an input strem (``AudioProcessor.output_stream``) id opened and a buffer is capture and recorded from the sound card.
- The buffer is shared into a queue object to allow DOA processing.
- DOA and dB SPL are calculated using ``update_das`` function; dB SPL of the average sound intensity is calculated thanks to the sensitivity characterization of the mics (:download:`sensitivity <../../Knowles_SPH0645LM4H-B_sensitivity.csv>`); DAS is used for the DOA estimation.

   .. automethod:: AudioProcessor.AudioProcessor.update_das

- If the calculated dB SPL is above the ``trigger_level`` or ``critical_level``, the DOA and dB SPL values are pushed into two separate queue objects to be read by the ``RobotMove`` class.

   .. automethod:: AudioProcessor.AudioProcessor.push_to_queue

   Robot Movement
---------------
The robot movement is controlled by the ``RobotMove.py`` class, which reads the continuos stream of DOA and dB SPL values from the ``AudioProcessor.py`` class.

TO BE FINISHED...

RAW FILES:
----------

Click :download:`here <../../SonoRo_swarm.py>` to view the raw Python script.

Click :download:`here <../../AudioProcessor.py>` to view the raw Python script.

Click :download:`here <../../RobotMove.py>` to view the raw Python script.

Click :download:`here <../../utilities.py>` to view the raw Python script.

Click :download:`here <../../utilities.py>` to view the raw Python script.
