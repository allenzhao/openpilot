"""The EPS swap: a 2022+ "steer-to-zero" EPS fitted to an older gen1 Mazda (values.STEER_TO_ZERO_EPS_FW).

The car keeps its own platform (its other ECUs are its year's) and fingerprints exactly, the 45 kph floor follows the
EPS firmware, and on that EPS the first activation of the drive in its standby from a stop is held at zero on the
EPS's own channel only: the Torque Interceptor's command is never held, and the EPS's channel climbs back from zero
inside the panda's limits."""
import pytest

from cereal import car, custom
from opendbc.can.packer import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaSafetyFlags
from openpilot.common.params import Params

Ecu = structs.CarParams.Ecu
DT = 0.01
STOCK_LKAS, TI_LKAS = 0x243, 0x249
LANEINFO = ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM")
FLOOR = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

# A 2017-21 CX-5 with a 2022 CX-5 EPS (commaCarSegments, dongle 9f46a57f430101ea): every ECU but the EPS is the body's.
SWAPPED_CX5_FW = {
  (Ecu.engine, 0x7e0): b'PX2H-188K2-E',
  (Ecu.fwdRadar, 0x764): b'K131-67XK2-E',
  (Ecu.transmission, 0x7e1): b'PYB2-21PS1-G',
  (Ecu.fwdCamera, 0x706): b'GSH7-67XK2-N',
  (Ecu.eps, 0x730): b'KSD5-3210X-C-00',
  (Ecu.abs, 0x760): b'KBJ5-437K2-B',
}


class Toggles:
  def __getattr__(self, name):
    return False


def car_fw(versions: dict) -> list[structs.CarParams.CarFw]:
  return [structs.CarParams.CarFw(ecu=ecu, fwVersion=fw.ljust(24, b'\x00'), address=addr,
                                  subAddress=0, brand='mazda') for (ecu, addr), fw in versions.items()]


def eps_only(fw: bytes | None) -> list[structs.CarParams.CarFw]:
  return [] if fw is None else car_fw({(Ecu.eps, 0x730): fw})


@pytest.fixture
def torque_interceptor():
  """Sets the TorqueInterceptorEnabled toggle for the test and puts it back after."""
  params = Params()
  prev = params.get_bool("TorqueInterceptorEnabled")

  def set_ti(on: bool):
    params.put_bool("TorqueInterceptorEnabled", on)
  set_ti(False)
  yield set_ti
  params.put_bool("TorqueInterceptorEnabled", prev)


def get_params(candidate, fw):
  return CarInterface.get_params(candidate, gen_empty_fingerprint(), fw, False, False, False, None)


def test_an_eps_swapped_cx5_fingerprints_exactly_as_its_own_year():
  exact, matches = match_fw_to_car(car_fw(SWAPPED_CX5_FW), "", log=False)
  assert exact and matches == {CAR.MAZDA_CX5}


@pytest.mark.parametrize("candidate, eps_fw, steer_to_zero", [
  (CAR.MAZDA_CX5, b'KSD5-3210X-C-00', True),     # the swap
  (CAR.MAZDA_CX5, b'KBST-3210X-A-00', True),
  (CAR.MAZDA_CX5, b'KJ01-3210X-M-00', False),    # the 2017-21 EPS
  (CAR.MAZDA_CX5, None, False),                  # an EPS that did not answer: the platform's own
  (CAR.MAZDA_CX5_2022, b'KSD5-3210X-C-00', True),
  (CAR.MAZDA_CX5_2022, b'K319-3210X-A-00', False),
  (CAR.MAZDA_CX5_2022, None, True),
  (CAR.MAZDA_CX9_2021, b'TC3M-3210X-A-00', False),
])
def test_the_floor_follows_the_eps_firmware(torque_interceptor, candidate, eps_fw, steer_to_zero):
  CP = get_params(candidate, eps_only(eps_fw))
  assert bool(CP.flags & MazdaSafetyFlags.STEER_TO_ZERO_EPS) == steer_to_zero
  assert CP.minSteerSpeed == (0.0 if steer_to_zero else pytest.approx(FLOOR))
  assert not CP.steerAtStandstill
  assert not CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.STEER_TO_ZERO_EPS   # never a panda flag


def test_gen2_has_no_floor_and_no_eps_flag(torque_interceptor):
  CP = get_params(CAR.MAZDA_3_2019, [])
  assert CP.minSteerSpeed == 0.0 and not CP.flags & MazdaSafetyFlags.STEER_TO_ZERO_EPS


@pytest.mark.parametrize("eps_fw", [b'KSD5-3210X-C-00', b'KJ01-3210X-M-00'])
def test_the_torque_interceptor_steers_to_a_stop_on_any_eps(torque_interceptor, eps_fw):
  torque_interceptor(True)
  CP = get_params(CAR.MAZDA_CX5, eps_only(eps_fw))
  assert CP.minSteerSpeed == 0.0 and CP.steerAtStandstill
  assert CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR
  assert bool(CP.flags & MazdaSafetyFlags.STEER_TO_ZERO_EPS) == (eps_fw == b'KSD5-3210X-C-00')
  assert CP.safetyConfigs[0].safetyParam == MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR


# --- car state: the first-activation hold ---------------------------------------------------------------------------

class Car:
  """A CX-5 car state fed one frame set per 10 ms step; messages not mentioned keep their last frame."""
  def __init__(self, steer_to_zero=True):
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5)
    CP.flags = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR |
                   (MazdaSafetyFlags.STEER_TO_ZERO_EPS if steer_to_zero else 0))
    self.CS = CarInterface(CP, custom.StarPilotCarParams.new_message()).CS
    self.parsers = CarState.get_can_parsers(CP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0
    self.CS.update(self.parsers, Toggles())

  def step(self, kph=0.0, block=0, track=0, effective=0, **msgs):
    self.t += 10_000_000
    msgs["STEER_RATE"] = {"LKAS_BLOCK": block, "LKAS_TRACK_STATE": track, "LKAS_EFFECTIVE": effective}
    msgs["WHEEL_SPEEDS"] = dict.fromkeys(("FL", "FR", "RL", "RR"), kph)
    frames = [self.packer.make_can_msg(name, 0, values) for name, values in msgs.items()]
    for cp in self.parsers.values():
      cp.update([(self.t, frames)])
    return self.CS.update(self.parsers, Toggles())


def test_the_hold_covers_the_standby_from_a_stop_until_the_eps_has_delivered():
  car = Car()
  car.step(block=1, track=1)
  assert car.CS.eps_first_engage_hold
  car.step(block=1, track=1, effective=2)         # the two lowest bits are not torque
  assert car.CS.eps_first_engage_hold
  car.step(block=0, effective=-60)                # delivered once
  assert not car.CS.eps_first_engage_hold
  car.step(block=1, track=1)                      # a later standby is not the first activation
  assert not car.CS.eps_first_engage_hold


def test_the_hold_ends_when_the_standby_lifts_or_the_car_rolls():
  car = Car()
  car.step(block=1, track=1)
  assert car.CS.eps_first_engage_hold
  car.step(block=1, track=0)                      # 0x04: blocked, but not the standby
  assert not car.CS.eps_first_engage_hold
  car.step(block=1, track=1, kph=5.0)             # rolling at 1.4 m/s
  assert not car.CS.eps_first_engage_hold
  car.step(block=1, track=1, kph=2.0)             # a crawl again, still before any delivery
  assert car.CS.eps_first_engage_hold


def test_a_legacy_eps_is_never_held():
  car = Car(steer_to_zero=False)
  car.step(block=1, track=1)
  assert not car.CS.eps_first_engage_hold


def test_the_cx5s_idle_horn_bits_are_not_the_horn():
  car = Car()
  _, fp = car.step(TURN_SWITCH={"HORN": 13})      # what a 2017-21 CX-5 sends at all times (byte 6 0x5a)
  assert not fp.hornPressed
  _, fp = car.step(TURN_SWITCH={"HORN": 15})      # the CX-9's press
  assert fp.hornPressed


# --- car controller: the EPS's own channel --------------------------------------------------------------------------

def panda_refusals(torques: list[int], driver: int = 0) -> list[int]:
  """Frames of a 0x243 stream the panda would refuse with controls allowed: mazda.h MAZDA_STEERING_LIMITS through
  lateral.h's driver_limit_check and real-time check. A refusal resets its references to zero, as there."""
  max_torque, up, down, allowance, multiplier, rt_delta = 800, 10, 25, 15, 1, 300
  last = rt_last = 0
  refused = []
  for i, t in enumerate(torques):
    driver_max = max_torque + (allowance + driver) * multiplier
    driver_min = -max_torque + (-allowance + driver) * multiplier
    highest = min(max(last, 0) + up, max(last - down, max(driver_max, 0)))
    lowest = max(min(last, 0) - up, min(last + down, min(driver_min, 0)))
    bad = abs(t) > max_torque or not lowest <= t <= highest or not min(rt_last, 0) - rt_delta <= t <= max(rt_last, 0) + rt_delta
    last = t
    if i % 25 == 24:
      rt_last = t
    if bad:
      refused.append(i)
      last = rt_last = 0
  return refused


def controller(torque_interceptor=True, steer_to_zero=True):
  CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5)
  CP.flags = int(MazdaSafetyFlags.GEN1 | (MazdaSafetyFlags.TORQUE_INTERCEPTOR if torque_interceptor else 0) |
                 (MazdaSafetyFlags.STEER_TO_ZERO_EPS if steer_to_zero else 0))
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  CS = CI.CS
  CS.cam_lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
  CS.cam_laneinfo = dict.fromkeys(LANEINFO, 0)
  CS.lkas_allowed_speed = True
  CS.accel_button = 0
  CS.ti_lkas_allowed = True
  CS.crz_btns_counter = 0
  CS.fsc_settled_frames = 10**6
  CI.CC.radar_suppress_failed = True   # stock longitudinal: the steering path alone
  return CI


def steer(CI, torque: float, hold: list[bool], v_ego: float = 0.0, lat_active: bool = True):
  """Drives the controller one frame per entry of `hold` (carstate's first-activation hold) with a constant request;
  returns (EPS commands, TI commands, reported torques)."""
  eps, ti, reported = [], [], []
  for f, held in enumerate(hold):
    CI.CS.eps_first_engage_hold = held
    cs = car.CarState.new_message(vEgo=v_ego, canValid=True, standstill=v_ego == 0.0)
    cs.cruiseState.enabled = cs.cruiseState.available = True
    CI.CS.out = cs
    cc = car.CarControl.new_message(enabled=True, latActive=lat_active)
    cc.actuators.torque = torque
    act, sends = CI.apply(cc.as_reader(), int(f * DT * 1e9), Toggles())
    for addr, dat, bus in sends:
      value = (((dat[0] & 0x0F) << 8) | dat[1]) - 2048
      if addr == STOCK_LKAS:
        eps.append(value)
      elif addr == TI_LKAS and bus == 1:
        ti.append(value)
    reported.append(act.torque)
  return eps, ti, reported


@pytest.mark.parametrize("sign", [1, -1])
def test_the_held_eps_channel_waits_while_the_ti_steers(sign):
  CI = controller()
  # 0.6 s held at a stop, then 1 s more: short of the 1.2 s after which the TI guard dips a command left unchanged
  eps, ti, reported = steer(CI, sign * 0.75, [True] * 60 + [False] * 100)
  assert eps[:60] == [0] * 60
  assert ti[59] == sign * 600                                          # the TI ramped at 10 a frame meanwhile
  assert reported[59] == pytest.approx(sign * 600 / 800)               # what the TI was sent
  climb = eps[60:]
  assert climb[0] == sign * 10 and all(abs(b - a) <= 10 for a, b in zip(climb, climb[1:], strict=False))
  assert climb[59] == sign * 600                                       # back from zero at the EPS's up rate
  assert eps[-40:] == ti[-40:] == [sign * 600] * 40                    # one command on both channels again
  assert all(abs(e) <= abs(t) for e, t in zip(eps, ti, strict=True))
  assert panda_refusals(eps) == []


def test_without_the_ti_the_held_channel_is_reported_as_sent():
  CI = controller(torque_interceptor=False)
  eps, ti, reported = steer(CI, 0.5, [True] * 30 + [False] * 60)
  assert ti == [] and eps[:30] == [0] * 30 and reported[:30] == [0.0] * 30
  assert eps[-1] == 400 and reported[-1] == pytest.approx(0.5)
  assert panda_refusals(eps) == []


def test_a_legacy_eps_mirrors_the_ti_as_before():
  CI = controller(steer_to_zero=False)
  eps, ti, _ = steer(CI, 0.5, [True] * 30 + [False] * 30)   # the hold is the steer-to-zero EPS's only
  assert eps == ti and eps[-1] == 400


def test_the_eps_channel_follows_the_shared_command_down_and_is_cut_with_it():
  CI = controller()
  eps, ti, _ = steer(CI, 0.5, [False] * 60)
  assert eps == ti and eps[-1] == 400
  down, ti_down, _ = steer(CI, 0.0, [False] * 8)          # the request let go: down at 25 a frame on both
  assert down == ti_down == [400 - 25 * (i + 1) for i in range(8)]
  cut, _, _ = steer(CI, 0.5, [False], lat_active=False)   # lateral off: a cut, which the panda allows
  assert cut == [0]
  assert panda_refusals(eps + down + cut) == []
