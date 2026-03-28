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
#  STAŁE
# ─────────────────────────────────────────────

# Biegi: upshift gdy speed_kmh > próg dla bieżącego biegu
UPSHIFT_KMH = [0, 55, 100, 150, 200, 250, 300, 350, 400, 440]

# Traction control (histereza)
SLIP_CUT     = 1.8   # wejście TC
SLIP_RECOVER = 1.2   # wyjście TC

# Steering
STEER_GAIN      = 2.5   # mnożnik błędu bocznego → skręt
STEER_HEADING   = 1.8   # mnożnik błędu kąta → skręt
STEER_MAX       = 1.0   # max wartość steer

# Krzywizna → hamowanie
# |curvature_1pm| > próg = zakręt, redukuj gaz
CURVE_LIGHT  = 0.005   # lekki zakręt
CURVE_MEDIUM = 0.012   # średni zakręt
CURVE_HARD   = 0.025   # ciasny zakręt

THROTTLE_STRAIGHT = 0.85
THROTTLE_CURVE_L  = 0.70
THROTTLE_CURVE_M  = 0.50
THROTTLE_CURVE_H  = 0.30
THROTTLE_TC       = 0.25   # traction control aktywny

BRAKE_CURVE_H     = 0.20   # hamowanie w ciasnym zakręcie

# Pit stop
WEAR_PIT = 0.70
WEAR_CRIT = 0.85

# Anti-stuck
STUCK_SPEED   = 4.0
STUCK_LIMIT   = 250
BTT_COOLDOWN  = 1800   # 30s × 60 tick/s


class RacingBot2:
    """
    Bot z pełnym racing line followerem opartym na CenterlinePoint.

    Kluczowe dane z ctx.track.centerline (CenterlinePoint):
      - position   : Vec3 — pozycja punktu w 3D
      - tangent    : Vec3 — kierunek jazdy w tym punkcie
      - right      : Vec3 — wektor w prawo od osi toru
      - curvature_1pm : float — krzywizna (1/m), im większa |wartość| tym ciasniejszy zakręt
      - s_m        : float — pozycja wzdłuż toru [m]
      - left_width_m / right_width_m : szerokość toru

    Algorytm:
    1. Znajdź najbliższy punkt centerline do pozycji auta.
    2. Oblicz błąd boczny (cross-track error) przez rzut na wektor right.
    3. Oblicz błąd kursu (heading error) przez iloczyn skalarny z tangent.
    4. Skręt = gain_lateral * cross_track_error + gain_heading * heading_error.
    5. Gaz/hamowanie na podstawie krzywizny PRZYSZŁYCH punktów (look-ahead).
    """

    def __init__(self) -> None:
        self.tick = 0
        self.has_moved    = False
        self.tc_active    = False
        self.stuck_ticks  = 0
        self.btt_cooldown = 0

        # Pit stop
        self.pit_requested     = False
        self.current_tire_type = TireType.HARD
        self.next_tire_type    = TireType.HARD

        # Cache centerline (pobieramy raz)
        self._centerline       = None
        self._centerline_len   = 0
        self._last_closest_idx = 0   # indeks poprzednio znalezionego punktu

    # ─────────────────────────────
    #  INIT CENTERLINE
    # ─────────────────────────────

    def _init_track(self, ctx: BotContext) -> None:
        if self._centerline is not None:
            return
        try:
            self._centerline     = ctx.track.centerline
            self._centerline_len = len(self._centerline)
        except Exception:
            self._centerline     = []
            self._centerline_len = 0

    # ─────────────────────────────
    #  GEOMETRIA
    # ─────────────────────────────

    @staticmethod
    def _dist2(p, cx, cy, cz) -> float:
        """Kwadrat odległości między Vec3 p a punktem (cx,cy,cz)."""
        dx = p.x - cx
        dy = p.y - cy
        dz = p.z - cz
        return dx*dx + dy*dy + dz*dz

    def _find_closest(self, car_pos) -> int:
        """
        Znajdź indeks najbliższego punktu centerline.
        Przeszukuje okno ±50 punktów wokół ostatnio znalezionego indeksu,
        żeby nie skanować całego toru co tick.
        """
        if self._centerline_len == 0:
            return 0

        n      = self._centerline_len
        start  = self._last_closest_idx
        window = 60   # liczba punktów do przeszukania w przód i tył

        best_idx  = start
        best_dist = float('inf')

        for offset in range(-10, window):
            idx = (start + offset) % n
            cp  = self._centerline[idx]
            d   = self._dist2(car_pos, cp.position.x, cp.position.y, cp.position.z)
            if d < best_dist:
                best_dist = d
                best_idx  = idx

        self._last_closest_idx = best_idx
        return best_idx

    def _max_curvature_lookahead(self, start_idx: int, steps: int = 20) -> float:
        """
        Zwróć maksymalną |krzywizną| na kolejnych `steps` punktach.
        Używamy tego do decyzji o hamowaniu zanim wejdziemy w zakręt.
        """
        n       = self._centerline_len
        max_curv = 0.0
        for i in range(1, steps + 1):
            idx  = (start_idx + i) % n
            curv = abs(self._centerline[idx].curvature_1pm)
            if curv > max_curv:
                max_curv = curv
        return max_curv

    def _compute_steering(self, car_pos, car_forward, closest_idx: int) -> float:
        """
        Oblicz wartość skrętu [-1, 1].

        car_forward: wektor kierunku jazdy auta (z orientacji/snapshotu).
        Używamy tangent z centerline jako docelowego kierunku.

        Cross-track error: rzut wektora (car_pos → cp.position) na cp.right.
          > 0 → auto jest po lewej stronie → skręć w prawo (steer > 0)
          < 0 → auto jest po prawej stronie → skręć w lewo  (steer < 0)

        Heading error: iloczyn wektorowy tangent z forward → kąt błędu kursu.
        """
        if self._centerline_len == 0:
            return 0.0

        n   = self._centerline_len
        # Patrzymy odrobinę do przodu żeby bot nie gonił punktu pod sobą
        lookahead_idx = (closest_idx + 3) % n
        cp  = self._centerline[lookahead_idx]

        # ── Cross-track error (błąd boczny) ──────────────────────────────
        # Wektor od punktu centerline do pozycji auta
        dx = car_pos.x - cp.position.x
        dy = car_pos.y - cp.position.y
        dz = car_pos.z - cp.position.z

        # Rzut na wektor cp.right (boczne odchylenie od osi toru)
        # Dodatni = auto po lewej od centerline → skręć w prawo
        cross_err = (dx * cp.right.x + dy * cp.right.y + dz * cp.right.z)

        # ── Heading error (błąd kierunku) ────────────────────────────────
        # Iloczyn wektorowy tangent × car_forward daje składową "w górę"
        # która mówi czy jedziemy za bardzo w lewo/prawo względem toru.
        # Uproszczenie: używamy tylko składowej y (zakładamy płaski tor).
        tx, ty, tz = cp.tangent.x, cp.tangent.y, cp.tangent.z
        fx, fy, fz = car_forward.x, car_forward.y, car_forward.z

        # Cross product Y-składowa: tx*fz - tz*fx (obrót w płaszczyźnie XZ)
        heading_err = tx * fz - tz * fx

        # ── Połącz błędy ─────────────────────────────────────────────────
        steer = STEER_GAIN * cross_err - STEER_HEADING * heading_err

        return max(-STEER_MAX, min(STEER_MAX, steer))

    # ─────────────────────────────
    #  HELPERS
    # ─────────────────────────────

    def _max_slip(self, car) -> float:
        s = car.tire_slip
        return max(s.front_left, s.front_right, s.rear_left, s.rear_right)

    def _avg_wear(self, car) -> float:
        w = car.tire_wear
        return (w.front_left + w.front_right + w.rear_left + w.rear_right) / 4.0

    def _gear_index(self, gear) -> int:
        try:
            v = int(gear.value)
            return max(0, v)
        except Exception:
            name = str(gear).upper()
            for k, v in [("FIRST",1),("SECOND",2),("THIRD",3),("FOURTH",4),
                          ("FIFTH",5),("SIXTH",6),("SEVENTH",7),("EIGHTH",8),
                          ("NINTH",9),("TENTH",10)]:
                if k in name:
                    return v
            return 0

    def _decide_gear(self, speed_kmh: float, gear) -> GearShift:
        idx = self._gear_index(gear)
        if idx <= 0:
            return GearShift.UPSHIFT
        if idx < len(UPSHIFT_KMH) and speed_kmh > UPSHIFT_KMH[idx]:
            return GearShift.UPSHIFT
        if idx >= 2 and speed_kmh < UPSHIFT_KMH[idx - 1] * 0.75:
            return GearShift.DOWNSHIFT
        return GearShift.NONE

    # ─────────────────────────────
    #  PIT STOP
    # ─────────────────────────────

    def _check_pit(self, car, ctx: BotContext) -> None:
        wear = self._avg_wear(car)
        if wear > WEAR_PIT and not self.pit_requested:
            self.pit_requested = True
            ctx.set_next_pit_tire_type(TireType.HARD)
        if self.pit_requested and wear < 0.05:
            self.pit_requested = False
            self.current_tire_type = self.next_tire_type

    # ─────────────────────────────
    #  ANTI-STUCK
    # ─────────────────────────────

    def _handle_stuck(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self.btt_cooldown = max(0, self.btt_cooldown - 1)
        speed = snapshot.car.speed_kmh
        if speed > 10:
            self.has_moved = True
        if not self.has_moved:
            return
        if speed < STUCK_SPEED:
            self.stuck_ticks += 1
        else:
            self.stuck_ticks = 0
        if self.stuck_ticks >= STUCK_LIMIT:
            self.stuck_ticks = 0
            if self.btt_cooldown == 0:
                ctx.request_back_to_track()
                self.btt_cooldown = BTT_COOLDOWN

    # ─────────────────────────────
    #  GŁÓWNA PĘTLA
    # ─────────────────────────────

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self.tick += 1
        car = snapshot.car

        # ── Inicjalizacja toru ─────────────────────────────────────────────
        self._init_track(ctx)

        # ── Faza startowa ──────────────────────────────────────────────────
        if self.tick <= 20:
            ctx.set_controls(throttle=0.0, brake=0.0, steer=0.0)
            return

        # ── Pit stop + anti-stuck ──────────────────────────────────────────
        self._check_pit(car, ctx)
        self._handle_stuck(snapshot, ctx)

        # ── Neutral / Reverse → wbij 1. bieg ──────────────────────────────
        if self._gear_index(car.gear) <= 0:
            ctx.set_controls(
                throttle=0.3, brake=0.0, steer=0.0,
                gear_shift=GearShift.UPSHIFT,
            )
            return

        # ── Traction control (histereza) ───────────────────────────────────
        slip = self._max_slip(car)
        if slip > SLIP_CUT:
            self.tc_active = True
        elif slip < SLIP_RECOVER:
            self.tc_active = False

        # ── Znajdź pozycję na torze ────────────────────────────────────────
        closest_idx = self._find_closest(car.position)

        # ── Steering z centerline ──────────────────────────────────────────
        # Potrzebujemy wektora "do przodu" auta. Używamy tangent do pozycji
        # albo orientacji z snapshotu jeśli jest dostępna.
        try:
            car_forward = car.forward   # Vec3 jeśli API udostępnia
        except AttributeError:
            # Fallback: weź tangent najbliższego punktu jako aproksymację
            if self._centerline_len > 0:
                car_forward = self._centerline[closest_idx].tangent
            else:
                car_forward = type('V', (), {'x': 1.0, 'y': 0.0, 'z': 0.0})()

        steer = self._compute_steering(car.position, car_forward, closest_idx)

        # ── Gaz / hamowanie na podstawie krzywizny look-ahead ─────────────
        if self.tc_active:
            throttle = THROTTLE_TC
            brake    = 0.0
        elif self._centerline_len > 0:
            max_curv = self._max_curvature_lookahead(closest_idx, steps=18)

            if max_curv > CURVE_HARD:
                throttle = THROTTLE_CURVE_H
                brake    = BRAKE_CURVE_H
            elif max_curv > CURVE_MEDIUM:
                throttle = THROTTLE_CURVE_M
                brake    = 0.0
            elif max_curv > CURVE_LIGHT:
                throttle = THROTTLE_CURVE_L
                brake    = 0.0
            else:
                throttle = THROTTLE_STRAIGHT
                brake    = 0.0
        else:
            # Brak danych o torze — jedź prosto
            throttle = 0.80
            brake    = 0.0

        # ── Skrzynia biegów ────────────────────────────────────────────────
        gear_shift = self._decide_gear(car.speed_kmh, car.gear)

        # ── Wyślij sterowanie ──────────────────────────────────────────────
        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=steer,
            gear_shift=gear_shift,
        )


if __name__ == "__main__":
    raise SystemExit(run_bot(RacingBot2()))