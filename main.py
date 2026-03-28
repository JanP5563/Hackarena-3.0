HA3_WRAPPER_API_URL="https://ha3-api.hackarena.pl/"

# EXAMPLE BOT

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
