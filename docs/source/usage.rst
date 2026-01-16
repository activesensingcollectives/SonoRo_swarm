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

Robot Movement
---------------
The robot movement is controlled by the ``RobotMove.py`` class, which reads the continuos stream of DOA and dB SPL values from the ``AudioProcessor.py`` class.

TO BE FINISHED...

Audio Sensing
--------------
TO BE ADDED...

RAW FILES:
----------
.. literalinclude:: ../../SonoRo_swarm.py
   :language: python
   :linenos:
   :caption: SonoRo_swarm.py

Click :download:`here <../../SonoRo_swarm.py>` to view the raw Python script.

.. literalinclude:: ../../AudioProcessor.py
   :language: python
   :linenos:
   :caption: AudioProcessor.py

Click :download:`here <../../AudioProcessor.py>` to view the raw Python script.

.. literalinclude:: ../../RobotMove.py
   :language: python
   :linenos:
   :caption: RobotMove.py

Click :download:`here <../../RobotMove.py>` to view the raw Python script.

.. literalinclude:: ../../utilities.py
   :language: python
   :linenos:
   :caption: utilities.py

Click :download:`here <../../utilities.py>` to view the raw Python script.