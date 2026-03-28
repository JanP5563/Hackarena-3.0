# main.py — HackArena 3.0 Racing Bot
# Strategia: stabilne okrążenia + traction control + zarządzanie oponami + pit stop decision

from hackarena3 import (
    BotContext,
    DriveGear,
    GearShift,
    RaceSnapshot,
    TireType,
    run_bot,
)
import math


# ─────────────────────────────────────────────
#  STAŁE I PROGI — dostrajaj podczas testów
# ─────────────────────────────────────────────

# Biegi
UPSHIFT_SPEED_KMH = [0, 60, 110, 160, 210, 260, 310, 360, 400, 440]
# Przejedź do następnego biegu dopiero gdy prędkość przekroczy próg dla bieżącego biegu
# np. UPSHIFT_SPEED_KMH[1] = 60 → z biegu 1 na 2 przy 60 km/h

# Traction control
MAX_SLIP_THROTTLE = 0.6   # próg poślizgu przy dodawaniu gazu (zrzuć gaz)
MAX_SLIP_BRAKE    = 0.5   # próg poślizgu przy hamowaniu (zmniejsz hamowanie)

# Progi temperatury opon — jeśli poza oknem, dostosuj styl jazdy
TEMP_COLD_HARD  = 85    # poniżej → opona twarda jak plastik, ogranicz agresję
TEMP_HOT_HARD   = 120   # powyżej → przegrzanie, odpuść gaz
TEMP_COLD_SOFT  = 65
TEMP_HOT_SOFT   = 100
TEMP_COLD_WET   = 50
TEMP_HOT_WET    = 80

# Zużycie opon — próg decyzji o pit stop (0.0 = nowe, 1.0 = strzępy)
TIRE_WEAR_PIT_THRESHOLD = 0.72   # wjedź do pitu gdy zużycie > 72%
TIRE_WEAR_CRITICAL      = 0.88   # krytyczne — wjedź natychmiast

# Sterowanie
BASE_THROTTLE       = 0.82    # gaz na prostej w optymalnych warunkach
THROTTLE_COLD_TIRE  = 0.55    # ograniczony gaz gdy opony za zimne
THROTTLE_HOT_TIRE   = 0.60    # ograniczony gaz gdy opony przegrzane
THROTTLE_HIGH_SLIP  = 0.0     # gaz gdy poślizg
BRAKE_SLIP_RELEASE  = 0.25    # hamowanie gdy koła blokują się (mniejsze)

# Back-to-track
STUCK_SPEED_KMH    = 3.0   # km/h — jeśli wolniej przez X ticków = stoisz/utknąłeś
STUCK_TICKS_LIMIT  = 180   # ~3 sekundy przy 60 tick/s


class RacingBot:
    """
    Strategia:
    1. Automatyczna skrzynia biegów na podstawie prędkości.
    2. Traction control — kontrola poślizgu na każdym kole.
    3. Monitor temperatury opon — adaptacja gazu/hamowania do okna pracy.
    4. Pit stop decision engine — wjedź do pitu gdy zużycie przekroczy próg.
    5. Anti-stuck handler — wykryj utkniecie i wywołaj back_to_track.
    6. Pierwsza runda w Ghost Mode — jedź spokojniej, nie marnuj opon.
    """

    def __init__(self) -> None:
        self.tick = 0

        # Pit stop
        self.pit_requested       = False
        self.current_tire_type   = TireType.HARD   # zakładamy start na Hard
        self.next_tire_type      = TireType.HARD

        # Anti-stuck
        self.stuck_ticks         = 0
        self.back_to_track_cd    = 0   # cooldown 30s × 60tick/s = 1800 ticków

        # Debug / metryki
        self.lap_start_tick      = 0
        self.best_lap_ticks      = None

    # ──────────────────────────────────────────
    #  HELPERS
    # ──────────────────────────────────────────

    def _avg_tire_wear(self, car) -> float:
        w = car.tire_wear
        return (w.front_left + w.front_right + w.rear_left + w.rear_right) / 4.0

    def _avg_tire_temp(self, car) -> float:
        t = car.tire_temperature_celsius
        return (t.front_left + t.front_right + t.rear_left + t.rear_right) / 4.0

    def _max_slip(self, car) -> float:
        s = car.tire_slip
        return max(s.front_left, s.front_right, s.rear_left, s.rear_right)

    def _is_tire_cold(self, avg_temp: float) -> bool:
        if self.current_tire_type == TireType.HARD:
            return avg_temp < TEMP_COLD_HARD
        if self.current_tire_type == TireType.SOFT:
            return avg_temp < TEMP_COLD_SOFT
        return avg_temp < TEMP_COLD_WET

    def _is_tire_hot(self, avg_temp: float) -> bool:
        if self.current_tire_type == TireType.HARD:
            return avg_temp > TEMP_HOT_HARD
        if self.current_tire_type == TireType.SOFT:
            return avg_temp > TEMP_HOT_SOFT
        return avg_temp > TEMP_HOT_WET

    def _target_gear(self, speed_kmh: float, current_gear: DriveGear) -> GearShift:
        """Prosta automatyczna skrzynia — upshift/downshift na podstawie prędkości."""
        gear_idx = current_gear.value if hasattr(current_gear, 'value') else 1

        # Upshift — jeśli prędkość przekracza próg dla bieżącego biegu
        if gear_idx < len(UPSHIFT_SPEED_KMH) and speed_kmh > UPSHIFT_SPEED_KMH[gear_idx]:
            return GearShift.UPSHIFT

        # Downshift — jeśli prędkość spada poniżej progu poprzedniego biegu - bufor
        if gear_idx > 1 and speed_kmh < UPSHIFT_SPEED_KMH[gear_idx - 1] * 0.80:
            return GearShift.DOWNSHIFT

        return GearShift.NONE

    def _decide_throttle(self, avg_temp: float, max_slip: float) -> float:
        """Zwróć docelowy gaz uwzględniając temperaturę opon i poślizg."""
        if max_slip > MAX_SLIP_THROTTLE:
            return THROTTLE_HIGH_SLIP       # traction control — odetnij gaz

        if self._is_tire_cold(avg_temp):
            return THROTTLE_COLD_TIRE       # zimne opony — delikatnie

        if self._is_tire_hot(avg_temp):
            return THROTTLE_HOT_TIRE        # przegrzane — odpuść trochę

        return BASE_THROTTLE                # optymalne warunki — pełna moc

    def _decide_brake(self, max_slip: float) -> float:
        """Zmniejsz hamowanie gdy koła blokują."""
        if max_slip > MAX_SLIP_BRAKE:
            return BRAKE_SLIP_RELEASE
        return 0.0

    # ──────────────────────────────────────────
    #  PIT STOP DECISION ENGINE
    # ──────────────────────────────────────────

    def _check_pit_stop(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        """
        Decyduj o typie opon i sygnalizuj chęć wjazdu do pitu.
        Pit stop fizycznie odbywa się przez wjechanie w strefę pit.
        Bot musi SAM dojechać do wjazdu — to zadanie dla sterownika.
        """
        avg_wear = self._avg_tire_wear(snapshot.car)

        # Krytyczne zużycie — natychmiast do pitu (emergency tylko w ostateczności)
        if avg_wear > TIRE_WEAR_CRITICAL and not self.pit_requested:
            self.pit_requested = True
            # Wybierz typ opon na kolejny pit: pogoda decyduje
            # Na razie zakładamy brak deszczu — Hard dla trwałości
            self.next_tire_type = TireType.HARD
            ctx.set_next_pit_tire_type(self.next_tire_type)

        # Normalne zużycie — zaplanuj pit
        elif avg_wear > TIRE_WEAR_PIT_THRESHOLD and not self.pit_requested:
            self.pit_requested = True
            self.next_tire_type = TireType.HARD
            ctx.set_next_pit_tire_type(self.next_tire_type)

        # Po pit stopie (zużycie opon wróciło blisko 0) — reset flagi
        if avg_wear < 0.1 and self.pit_requested:
            self.pit_requested = False
            self.current_tire_type = self.next_tire_type

    # ──────────────────────────────────────────
    #  ANTI-STUCK HANDLER
    # ──────────────────────────────────────────

    def _handle_stuck(self, snapshot: RaceSnapshot, ctx: BotContext) -> bool:
        """
        Wykryj utkniecie (bardzo niska prędkość przez wiele ticków).
        Wywołaj back_to_track z uwzględnieniem cooldownu 30s.
        Zwraca True jeśli bot jest aktualnie stuck i obsługuje sytuację.
        """
        self.back_to_track_cd = max(0, self.back_to_track_cd - 1)

        if snapshot.car.speed_kmh < STUCK_SPEED_KMH:
            self.stuck_ticks += 1
        else:
            self.stuck_ticks = 0

        if self.stuck_ticks >= STUCK_TICKS_LIMIT:
            if self.back_to_track_cd == 0:
                ctx.request_back_to_track()
                self.back_to_track_cd = 1800  # 30s cooldown
                self.stuck_ticks = 0
                return True
            else:
                # Cooldown aktywny — spróbuj wyjechać wstecz
                ctx.set_controls(
                    throttle=0.0,
                    brake=0.5,
                    steer=0.0,
                    gear_shift=GearShift.DOWNSHIFT,
                )
                return True

        return False

    # ──────────────────────────────────────────
    #  GŁÓWNA PĘTLA
    # ──────────────────────────────────────────

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self.tick += 1
        car = snapshot.car

        # ── 1. Pierwsze ticki — czekaj na stabilizację ─────────────────────
        if self.tick <= 30:
            ctx.set_controls(throttle=0, brake=0, steer=0)
            return

        # ── 2. Pierwsze okrążenie w Ghost Mode — jedź ostrożniej ───────────
        in_ghost_mode = (car.ghost_mode is not None and
                         str(car.ghost_mode).lower() not in ("inactive", "ghost_mode.inactive"))

        # ── 3. Anti-stuck ───────────────────────────────────────────────────
        if self._handle_stuck(snapshot, ctx):
            return

        # ── 4. Dane telemetryczne ───────────────────────────────────────────
        avg_temp  = self._avg_tire_temp(car)
        max_slip  = self._max_slip(car)
        avg_wear  = self._avg_tire_wear(car)
        speed_kmh = car.speed_kmh

        # ── 5. Pit stop decision ────────────────────────────────────────────
        self._check_pit_stop(snapshot, ctx)

        # ── 6. Automatyczna skrzynia biegów ────────────────────────────────
        # Upewnij się że jesteśmy na biegu > 0 (nie Reverse/Neutral)
        if car.gear in (DriveGear.REVERSE, DriveGear.NEUTRAL):
            ctx.set_controls(
                throttle=0,
                brake=0.3,
                steer=0.0,
                gear_shift=GearShift.UPSHIFT,
            )
            return

        gear_shift = self._target_gear(speed_kmh, car.gear)

        # ── 7. Oblicz throttle i hamowanie ─────────────────────────────────
        throttle = self._decide_throttle(avg_temp, max_slip)
        brake    = self._decide_brake(max_slip)

        # W Ghost Mode (pierwsze okrążenie) — jedź spokojniej żeby opony
        # weszły w okno pracy zanim kolizje się włączą
        if in_ghost_mode:
            throttle = min(throttle, 0.60)

        # ── 8. Wyślij sterowanie ────────────────────────────────────────────
        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=0.0,      # TODO: zastąpić racing line followerem gdy znasz geometrię toru
            gear_shift=gear_shift,
        )

class SimpleBot:
    def __init__(self):
        self.tick = 0
    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext):
        print(f"Działam! Tick: {self.tick}")
        self.tick += 1

if __name__ == "__main__":
    print("Próbuję wystartować serwer...")
    run_bot(SimpleBot())