## M109 Integrated Design Project Codebase

Software team: Jeffrey Yeung (shy30), Frank Zhu (tz387)

## Main code

The mission code is in `competition.py`. `main.py` is the power-up entry point; it starts `competition.run()` and waits for the start button.

## Python files

- `main.py` starts the competition run.
- `competition.py` runs the mission: rack survey, package pickup, delivery, and return to the hub.
- `graphs.py` describes the track and plans routes between bays, racks, and the hub.
- `nav.py` handles line following, turns, pickup and placement actions, and route execution.
- `hw.py` controls the motors, servos, lights, buttons, and sensors.
- `config.py` holds the pin assignments and settings for motors, turns, sensors, and the mission.

## Sensor libraries

- `libs/DFRobot_TMF8x01/DFRobot_TMF8x01.py` drives TMF8801 and TMF8701 distance sensors.
- `libs/VL53L0X/VL53L0X.py` drives the distance sensor used for rack slot checks.
- `libs/VNCL4010/VNCL4010.py` drives a VCNL4010 proximity sensor.
- `libs/mfrc522_python/src/mfrc522/MFRC522.py` drives an MFRC522 RFID reader.
- `libs/tcs3472_micropython/tcs3472.py` reads colour from a TCS3472 sensor.
- `libs/tiny_code_reader/tiny_code_reader.py` communicates with the tiny code reader.

## Major coding decisions

- OOP for maintainability, clarity, and a modular approach.
- The map is mapped out as a graph, with nodes and links, so the AGV can go from one spot to another.
- Line following uses simple negative feedback.
- Motor PWMs are not identical, so we use constant trim to minimise drift.
- Turns are separated into advance and turn-on points to make them reproducible.
- Turn and adjustment timings are manually tuned.
- Hall effect and distance sensor values are interpreted using manual tuning.
- Turns need reference timings and a tolerance window because the rack lines are closely spaced, so the robot can only try to stop on the middle line within a certain timeframe. This needs tuning.

