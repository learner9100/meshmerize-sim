# Robot firmware

This is what runs on the ESP32. The maze brain (`../core`) is the same code
the simulator runs on your PC, so a strategy that wins in `make compare`
is the strategy the robot drives. Everything that depends on *your*
hardware is in four small files you are expected to edit:

| file | what is in it | when you touch it |
|---|---|---|
| `src/config.h` | pins, sensor type, motor wiring, wheel numbers, speeds, PID, policy | always; this is your robot |
| `src/sensors.h` | reading and calibrating the IR array, line position, branch detection | different sensor array |
| `src/motors.h` | TB6612 PWM, encoder interrupts | different motor driver or encoder |
| `src/drive.h` | line following, junction detection, turning | tuning how the robot drives |

You should not need to touch `robot_walker.h` (adapts the above to the
interface the explorer expects), `menu.h` (serial tests) or `main.cpp`
(button, LED, Dry Run, Actual Run), but they are short and readable.

## Setup once

```sh
pip install platformio
cd firmware
pio run                 # compiles; first time downloads the ESP32 toolchain
pio run -t upload       # flash
pio device monitor      # serial at 115200
```

No Arduino libraries to install. WiFi and Bluetooth are switched off at
boot because the rules forbid them.

## Bring-up, one subsystem at a time

Open the serial monitor; the menu appears. Work down the list. Every test
prints what to change in `config.h` if something is off.

1. **`1` IR raw values.** Hold the robot over the floor, then over the
   line. Every sensor must move clearly. One that does not is miswired or
   dead. If values go *up* on white, set `IR_LINE_READS_HIGH = true`.
2. **`2` IR calibrate.** Sweep the sensor row slowly across the line for
   6 s. The min/max per sensor is saved to flash and survives reboots.
   Redo it when the lighting changes (do it at the venue).
3. **`3` IR normalised.** Line should read ~1000, floor ~0, and `pos`
   should go from -1000 (line under the leftmost sensor) to +1000. The `L`
   and `R` flags show branch detection; adjust `NODE_SIDE_SENSORS` and
   `IR_ON_LINE` if they flicker.
4. **`4` motors.** Each wheel runs forward alone, then both. A wheel going
   backwards: flip `M_*_REVERSED`. A count going negative: flip
   `E_*_REVERSED`. With both running, the two counts should be close; if
   not, one motor is weaker (normal) and the line follower compensates.
5. **`5` encoder scale.** Push the robot exactly 1 m. The test prints the
   `COUNTS_PER_WHEEL_REV` to put in `config.h`. Also measure
   `WHEEL_DIAMETER_MM` and `WHEEL_BASE_MM` with a ruler and `IR_TO_AXLE_MM`
   from the sensor row to the wheel axle.
6. **`6` line following.** On a straight line with a T at the end. Tune
   `LINE_KP` (higher = reacts harder, oscillates when too high) and
   `LINE_KD` (damps the oscillation). It stops at the junction and prints
   which exits it saw: this is the single most important thing to get
   right. If it misses branches, lower `NODE_CONFIRM_MM`; if it invents
   them on curves, raise it.
7. **`7` turns.** Left, right, about-face. The robot should end up with
   the line under the centre sensors. Adjust `WHEEL_BASE_MM` first (it
   sets the arc), then `TURN_OVERSHOOT_DEG`.
8. **`8` benchmark.** Runs the explorer and solver on generated mazes on
   the chip and prints microseconds and stack use. Just to know.

Then build a small test maze with tape, `d` for a Dry Run, `a` for the
Actual Run, `m` to print the map the robot built. In the arena, where no
laptop is allowed: a **short press** of the button starts the Dry Run, the
next short press the Actual Run. If the robot had to be reset in between,
a **long press (2 s)** drives the route saved in flash.

## Choosing the strategy

`POLICY` in `config.h` takes the same names as the simulator's strategies.
Run `make compare` on the PC with your measured `robot.cfg`, pick the
winner, set it here. Keep the timing numbers (`DRY_*`) in `config.h` and
`robot.cfg` identical, otherwise the robot's time guard reasons with
different numbers than the ones you tested.

## What the End looks like

Unknown. The firmware assumes a patch where **all sensors read line for
`END_MARK_MM`**. Check the real arena and change `followUntilNode()` in
`drive.h` if the End is marked differently (a cross, a stop box, a
colour). This is the one assumption that can sink the whole run; verify
it first thing at the venue.

## Different hardware

- **Digital IR array**: `IR_ANALOG = false`. Calibration becomes a no-op.
- **Fewer/more sensors**: change `IR_COUNT` and `IR_PIN`. 5 works; 8 is
  more forgiving at junctions.
- **DRV8833 / L298N**: replace `Motors::drive()` in `motors.h`. Keep
  `set(left, right)` with the same meaning and nothing else changes.
- **No encoders**: not supported. The maze brain needs distances between
  junctions to tell places apart. Cheap N20 motors with magnetic encoders
  are enough.

## Honest status

Compiled for `esp32dev` with PlatformIO; **not yet driven on a real
robot**. Expect to tune the numbers in `config.h` and probably the
junction logic in `drive.h` on the first afternoon with hardware. The
maze brain itself is covered by the simulator's tests.
