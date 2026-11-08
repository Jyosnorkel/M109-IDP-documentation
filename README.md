## M109 Integrated Design Project Codebase

Software team: Jeffrey Yeung (shy30), Frank Zhu (tz387)

## Main code

The mission code is in `competition.py`. `main.py` is the power-up entry point; it starts `competition.run()` and waits for the start button.

## Python files

- `main.py` starts the competition run.
- `competition.py` runs the mission: rack survey, package pickup, delivery, and return to the hub.
- `config.py` holds the pin assignments and settings for motors, turns, sensors, and the mission.
- `nav.py` handles line following, turns, pickup and placement actions, and route execution.
- `graphs.py` describes the track and plans routes between bays, racks, and the hub.
- `hw.py` controls the motors, servos, lights, buttons, and sensors.

The drivers in `libs/` are for the distance, RFID, colour, proximity, and code-reader sensors. The mission uses `libs/VL53L0X/VL53L0X.py` for rack slot checks.

## To run it

1. Check that the pins and settings in `config.py` match the robot.
2. Copy the project Python files and `libs/` folder to the Pico.
3. Put the robot at the hub facing the ramp and power it on. Press the start button when ready.
4. Check the route, sensors, pickup, and rack placement on the course; adjust `config.py` if needed.
