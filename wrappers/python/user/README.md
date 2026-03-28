# Hackarena3.0 Python API Wrapper

This API wrapper provides a Python interface for the Hackarena racing game. It allows you to write automated bots that play the game by interacting with the server through code.

To write a bot, you need to implement a class with an `on_tick` method. This method is called continuously during the race and is the core of your bot's logic. It receives two crucial objects: `RaceSnapshot` (to read the game state) and `BotContext` (to send commands to the server).

## Core Concepts

### 1. RaceSnapshot (Reading the Game State)
The `RaceSnapshot` object delivers real-time data from the server. It tells you everything about the current state of your car, your opponents, and the race environment.

Key fields inside `RaceSnapshot`:
* `tick` (int): The current simulation tick.
* `server_time_ms` (int): The current server time in milliseconds.
* `car` (CarState): Detailed information about your vehicle. Key properties include:
  * `position` (Vec3): Your car's coordinates (x, y, z) on the track.
  * `speed_mps` (float) / `speed_kmh` (float): Current speed.
  * `gear` (DriveGear): Current gear (e.g., REVERSE, NEUTRAL, FIRST, etc.).
  * `tire_wear`, `tire_temperature_celsius`, `tire_slip`: Dataclasses containing telemetry for each of the four wheels.
  * `ghost_mode` (GhostModeState): Information on whether your car can currently collide with others.
* `opponents` (tuple[OpponentState, ...]): A list containing the ID, position, orientation, and ghost state of other players.

### 2. BotContext (Interacting with the Game)
The `BotContext` object is responsible for communication between your bot and the game server. You use it to retrieve static track information and to issue commands.

Key properties and methods inside `BotContext`:
* **Properties:**
  * `car_id` (int): Your unique vehicle ID.
  * `map_id` (str): The identifier of the current track.
  * `track` (TrackLayout): Static data about the track, including the centerline, lap length, and pitstop layout.
* **Methods:**
  * `set_controls(throttle, brake, steer, gear_shift=GearShift.NONE, ...)`: Sends driving inputs to the server. Values for throttle, brake, and steer are floats.
  * `request_back_to_track()`: Requests the server to reset your car to the track centerline.
  * `request_emergency_pitstop()`: Forces an emergency teleport to the pit lane.
  * `set_next_pit_tire_type(tire_type: TireType)`: Selects the tire compound (HARD, SOFT, WET) for your next pitstop.

## Example Bot

Below is a simple example of a bot demonstrating how to use `RaceSnapshot` and `BotContext` together.

```python
from hackarena3 import BotContext, DriveGear, GearShift, RaceSnapshot, run_bot

class ExampleBot:
    def __init__(self) -> None:
        self.tick = 0

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self.tick += 1

        # Wait for 50 ticks before doing anything
        if self.tick <= 50:
            return

        # Simple logic to alternate between driving forward and reversing
        if (self.tick // 100) % 2:
            # Shift to REVERSE if not already there, apply brakes to stop first
            if snapshot.car.gear != DriveGear.REVERSE:
                ctx.set_controls(
                    throttle=0,
                    brake=0.5,
                    steer=0.0,
                    gear_shift=GearShift.DOWNSHIFT,
                )
                return
        else:
            # Shift up if in REVERSE or NEUTRAL
            if snapshot.car.gear in (DriveGear.REVERSE, DriveGear.NEUTRAL):
                ctx.set_controls(
                    throttle=0,
                    brake=0.5,
                    steer=0.0,
                    gear_shift=GearShift.UPSHIFT,
                )
                return

        # Traction Control: Check if any tire is slipping significantly
        max_slip = max(
            snapshot.car.tire_slip.front_left,
            snapshot.car.tire_slip.front_right,
            snapshot.car.tire_slip.rear_left,
            snapshot.car.tire_slip.rear_right,
        )
        
        if max_slip > 1.0:
            # If slipping, stop accelerating and apply light brakes
            ctx.set_controls(
                throttle=0.0,
                brake=0.1,
                steer=0.0,
            )
            return

        # Default driving behavior: accelerate straight
        ctx.set_controls(
            throttle=0.55,
            brake=0,
            steer=0.0,
        )

if __name__ == "__main__":
    raise SystemExit(run_bot(ExampleBot()))
```

### Explanation of the Example
1. **State Tracking:** The bot keeps track of local ticks using `self.tick`.
2. **Reading Data:** It uses `snapshot.car.gear` to check the current gear and `snapshot.car.tire_slip` to read the physics state of the wheels.
3. **Decision Making:** It implements a basic traction control system. If `max_slip` exceeds a safe threshold, the bot decides to cut the throttle and apply light brakes.
4. **Executing Actions:** The `ctx.set_controls()` method is called to send the calculated inputs (throttle, brake, steering, and gear shifts) back to the server.

## CLI Commands

You manage your bot submissions and API wrapper versions using the provided Hackarena executable.

### Submitting Your Bot
To submit your bot code to the server, run the following command in your terminal:

```bash
hackarena.exe submit --slot 3 -d "My example bot"
```
* `--slot`: Specifies the slot to deploy the bot into. Every team has exactly 3 slots available (values from 1 to 3 inclusive).
* `-d`: Sets the display name for your bot submission.

### Updating the API Wrapper
To ensure you have the latest version of the API wrapper and game definitions, run:

```bash
hackarena.exe update
```

### Getting Help
If you are unsure about the available commands or arguments, you can always check the built-in help menu:

```bash
hackarena.exe --help
```