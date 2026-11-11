"""
Mazda's Torque Interceptor in this tree: the toggle's way to the car's flags, what card
publishes of the interceptor, the alert that names it, the learned torque caches and the
low-speed gain. The car side and the panda's are tested in opendbc
(opendbc/car/mazda/tests/test_mazda_torque_interceptor.py, opendbc/safety/tests/test_mazda.py).
"""
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaFlags, MazdaSafetyFlags, TorqueInterceptorState
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import setup_interfaces as opendbc_setup_interfaces
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.lib import latcontrol_torque as latcontrol_torque_v1
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.car.card_ext import fill_torque_interceptor
from openpilot.sunnypilot.selfdrive.car.interfaces import _reset_torque_caches_on_actuator_change, initialize_params
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP, TUNE_SWAP_INACTIVE_FRAMES, \
  ControlsLateralZP
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_torque_v0
from openpilot.sunnypilot.selfdrive.controls.lib.tests.torque_harness import DT, LAF, make_ci
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
from openpilot.sunnypilot.selfdrive.selfdrived.events_zp import EVENTS_ZP, torque_interceptor_not_ready_alert

EventName = log.OnroadEvent.EventName
EventNameZP = custom.OnroadEventZP.EventName
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

LEGACY = MazdaFlags.GEN1 | MazdaFlags.LEGACY_FW_EPS
STEER_TO_ZERO = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS
TI = MazdaFlags.TORQUE_INTERCEPTOR


def mazda_params(params_list):
  """CarParams as card's get_car builds them from the list initialize_params hands it."""
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  opendbc_setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CP, CP_SP


class TestToggle:
  @pytest.mark.parametrize("enabled", [False, True])
  def test_it_reaches_the_car_as_a_flag_and_a_safety_bit(self, enabled):
    with OpenpilotPrefix():
      params = Params()
      params.put_bool("TorqueInterceptorEnabled", enabled, block=True)
      params_list = initialize_params(params)
      assert {"TorqueInterceptorEnabled": enabled} in params_list
      CP, _ = mazda_params(params_list)
      assert bool(CP.flags & TI) == enabled
      assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TORQUE_INTERCEPTOR) == enabled

  def test_unset_is_off_not_none(self):
    # opendbc reads it with int(); a key without a declared default would hand it None
    with OpenpilotPrefix():
      params = Params()
      params.remove("TorqueInterceptorEnabled")
      params_list = initialize_params(params)
      assert {"TorqueInterceptorEnabled": False} in params_list
      CP, _ = mazda_params(params_list)
      assert not CP.flags & TI


class TestCardPublishes:
  def test_what_the_interceptor_says_of_itself(self):
    msg = messaging.new_message('carStateSP').carStateSP
    car_state = SimpleNamespace(ti_enabled=True, ti_not_ready=True, ti_fresh=True, ti_ready=False,
                                ti_state=int(TorqueInterceptorState.OFF), ti_violation=0x11, ti_error=0, ti_version=0x10)
    fill_torque_interceptor(msg, car_state)
    ti = msg.zoompilot.torqueInterceptor
    assert (ti.notReady, ti.heard, ti.ready, ti.state, ti.violation, ti.error, ti.version) == (True, True, False, 1, 0x11, 0, 0x10)

  def test_nothing_on_a_car_without_one(self):
    # not even the empty struct: carStateSP is logged for every car
    untouched = len(messaging.new_message('carStateSP').to_bytes())
    for car_state in (SimpleNamespace(), SimpleNamespace(ti_enabled=False, ti_not_ready=True)):
      event = messaging.new_message('carStateSP')
      fill_torque_interceptor(event.carStateSP, car_state)
      assert len(event.to_bytes()) == untouched
      assert not event.as_reader().carStateSP.zoompilot.torqueInterceptor.notReady

  @pytest.mark.parametrize("enabled", [False, True])
  def test_the_real_carstate_carries_every_field_it_reads(self, enabled):
    # the contract between the two repos: card reads these attributes off opendbc's CarState
    CP, CP_SP = mazda_params([{"TorqueInterceptorEnabled": enabled}])
    CS = CarInterface(CP, CP_SP).CS
    assert CS.ti_enabled == enabled
    msg = messaging.new_message('carStateSP').carStateSP
    fill_torque_interceptor(msg, CS)
    ti = msg.zoompilot.torqueInterceptor
    assert (ti.notReady, ti.heard, ti.ready, ti.state) == (False, False, False, int(TorqueInterceptorState.DISCOVER))


def car_events(flags) -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = 'mazda'
  CP.flags = int(flags)
  return CarSpecificEventsSP(CP, structs.CarParamsSP(), params=SimpleNamespace(get_bool=lambda key: False))


def car_state_sp(not_ready: bool):
  cs_sp = messaging.new_message('carStateSP').carStateSP
  cs_sp.zoompilot.torqueInterceptor.notReady = not_ready
  return cs_sp


def events(*names: int) -> Events:
  ret = Events()
  for name in names:
    ret.add(name)
  return ret


class TestEvents:
  @pytest.mark.parametrize("flags", [LEGACY | TI, STEER_TO_ZERO | TI])
  @pytest.mark.parametrize("mads", [False, True])
  def test_not_ready_is_named_and_refuses_lateral_entry_only(self, flags, mads):
    events_sp = car_events(flags).update(structs.CarState(), events(), car_state_sp(True), mads, mads)
    assert events_sp.zp.has(EventNameZP.torqueInterceptorNotReady)
    assert events_sp.contains(ET.PERMANENT), "named whether or not anything is engaged"
    assert events_sp.contains(ET.NO_ENTRY)
    # a ZP soft disable drops MADS lateral in one frame with no alert while cruise is engaged;
    # carstate's steerFaultTemporary is what takes lateral, through upstream's path
    for disabling in (ET.SOFT_DISABLE, ET.IMMEDIATE_DISABLE, ET.USER_DISABLE):
      assert not events_sp.contains(disabling)

  def test_ready_raises_nothing(self):
    events_sp = car_events(LEGACY | TI).update(structs.CarState(), events(), car_state_sp(False), True, True)
    assert not events_sp.zp.has(EventNameZP.torqueInterceptorNotReady)

  def test_a_car_without_one_raises_nothing(self):
    events_sp = car_events(LEGACY).update(structs.CarState(), events(), car_state_sp(True), True, True)
    assert not events_sp.zp.has(EventNameZP.torqueInterceptorNotReady)

  @pytest.mark.parametrize("flags", [LEGACY | TI, STEER_TO_ZERO | TI])
  def test_its_steer_fault_keeps_upstreams_escalation(self, flags):
    # on the steer-to-zero EPS alone the fault is the non-delivery latch and is downgraded;
    # with an interceptor it is the interceptor gone with the car moving
    ev = events(EventName.steerTempUnavailable)
    car_events(flags).update(structs.CarState(), ev, car_state_sp(True))
    assert ev.has(EventName.steerTempUnavailable)
    assert not ev.has(EventName.steerTempUnavailableSilent)
    assert ev.contains(ET.SOFT_DISABLE)

  def test_without_one_the_steer_to_zero_downgrade_stands(self):
    ev = events(EventName.steerTempUnavailable)
    car_events(STEER_TO_ZERO).update(structs.CarState(), ev, car_state_sp(False))
    assert ev.has(EventName.steerTempUnavailableSilent)

  def test_every_zp_event_has_an_entry(self):
    # selfdrived indexes EVENTS_ZP and the counters built from it by name: a missing one is a KeyError
    events_sp = EventsSP()
    for name, value in EventNameZP.schema.enumerants.items():
      assert value in EVENTS_ZP, name
      assert value in events_sp.zp.event_counters, name


class TestAlert:
  @pytest.mark.parametrize("fields, line", [
    ({}, "No signal: check its cables and OBD power"),
    ({"heard": True, "state": 1, "violation": 0x11}, "Fault code 11"),
    ({"heard": True, "state": 1, "violation": 0x12}, "Fault code 12"),
    ({"heard": True, "state": 1, "error": 0x2A}, "Error code 2A"),
    ({"heard": True, "state": 1}, "Starting up"),
    ({"heard": True, "state": 2}, "Starting up"),
    ({"heard": True, "state": 3}, "Not taking requests"),
  ])
  def test_it_says_what_the_interceptor_last_said(self, fields, line):
    msg = messaging.new_message('carStateSP')
    ti = msg.carStateSP.zoompilot.torqueInterceptor
    ti.notReady = True
    for key, value in fields.items():
      setattr(ti, key, value)
    sm = {'carStateSP': msg.carStateSP}
    alert = torque_interceptor_not_ready_alert(None, None, sm, False, 0, None)  # ty: ignore[invalid-argument-type]
    assert alert.alert_text_1 == "Torque Interceptor Not Ready"
    assert alert.alert_text_2 == line
    # heard once when it comes up: the steer fault beside it is silent for a driver holding the wheel
    assert alert.audible_alert == AudibleAlert.prompt

  def test_selfdrived_builds_it_through_the_event(self):
    events_sp = EventsSP()
    events_sp.zp.add(EventNameZP.torqueInterceptorNotReady)
    msg = messaging.new_message('carStateSP')
    alerts = events_sp.create_alerts([ET.PERMANENT, ET.NO_ENTRY], [None, None, {'carStateSP': msg.carStateSP}, False, 0, None])
    assert [a.alert_type for a in alerts] == ['torqueInterceptorNotReady/permanent', 'torqueInterceptorNotReady/noEntry']


def car_params(flags, brand='mazda'):
  CP = car.CarParams.new_message()
  CP.brand = brand
  CP.flags = int(flags)
  return CP


class TestLearnedTorqueCaches:
  """torqued restores its cache on a key of fingerprint and offline tune, which a Torque
  Interceptor leaves alone: card clears what was learned on the other actuator."""

  CACHES = ("LiveTorqueParameters", "LiveTorqueParametersSP")

  @pytest.mark.parametrize("prev, now, cleared", [
    (LEGACY, LEGACY | TI, True),
    (LEGACY | TI, LEGACY, True),
    (LEGACY, LEGACY, False),
    (LEGACY | TI, LEGACY | TI, False),
    (STEER_TO_ZERO | MazdaFlags.HAS_BSM, STEER_TO_ZERO, False),
    (None, LEGACY | TI, False),
  ])
  def test_cleared_when_the_interceptor_came_or_went(self, prev, now, cleared):
    with OpenpilotPrefix():
      params = Params()
      if prev is not None:
        params.put("CarParamsPersistent", car_params(prev).to_bytes(), block=True)
      for key in self.CACHES:
        params.put(key, b"learned", block=True)
      _reset_torque_caches_on_actuator_change(car_params(now), params)
      for key in self.CACHES:
        assert (params.get(key) is None) == cleared, key

  def test_other_brands_are_left_alone(self):
    with OpenpilotPrefix():
      params = Params()
      params.put("CarParamsPersistent", car_params(0, 'honda').to_bytes(), block=True)
      for key in self.CACHES:
        params.put(key, b"learned", block=True)
      # the bit means something else, or nothing, in another brand's flags
      _reset_torque_caches_on_actuator_change(car_params(TI, 'honda'), params)
      _reset_torque_caches_on_actuator_change(car_params(LEGACY | TI), params)
      for key in self.CACHES:
        assert params.get(key) == b"learned", key

  def test_an_unreadable_last_drive_clears_nothing(self):
    with OpenpilotPrefix():
      params = Params()
      params.put("CarParamsPersistent", b"\x00not capnp", block=True)
      params.put("LiveTorqueParameters", b"learned", block=True)
      _reset_torque_caches_on_actuator_change(car_params(LEGACY | TI), params)
      assert params.get("LiveTorqueParameters") == b"learned"


def torque_cp(flags):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  CP.brand = 'mazda'
  CP.carFingerprint = 'MAZDA_CX5'
  CP.flags = int(flags)
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  return CP.as_reader()


def build_controllers(flags, tune=2.0):
  """controlsd's startup: upstream's controller, then the fork's one per model size."""
  with OpenpilotPrefix():
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put("TorqueControlTune", tune, block=True)
    params.put("TorqueControlTuneBig", 1.0, block=True)
    CP = torque_cp(flags)
    CP_SP = custom.CarParamsSP.new_message().as_reader()
    CI = make_ci()
    upstream = latcontrol_torque_v1.LatControlTorque(CP, CP_SP, CI, DT)
    controls = SimpleNamespace(params=params, CP=CP, CP_SP=CP_SP, _inactive_frames=TUNE_SWAP_INACTIVE_FRAMES)
    ControlsLateralZP.initialize_lateral_control(controls, upstream, CI, DT)  # ty: ignore[invalid-argument-type]
    return controls._lac_by_size


class TestLowSpeedGain:
  """The tunes' proportional ladder under 10 m/s was never delivered by a Mazda EPS. The
  interceptor delivers it, so its cars run the ladder held to a cap, on every tune."""

  # the module whose ladder each tune version runs: v2 is v0's with a derivative schedule
  LADDERS = {0.0: latcontrol_torque_v0, 1.0: latcontrol_torque_v1, 2.0: latcontrol_torque_v0}

  @pytest.mark.parametrize("flags", [LEGACY | TI, STEER_TO_ZERO | TI])
  @pytest.mark.parametrize("tune", [0.0, 2.0])
  def test_capped_on_both_model_sizes_controllers(self, flags, tune):
    by_size = build_controllers(flags, tune)
    assert by_size[False] is not by_size[True]
    for big, version in ((False, tune), (True, 1.0)):
      module = self.LADDERS[version]
      speeds, gains = by_size[big].pid._k_p
      assert list(speeds) == module.INTERP_SPEEDS
      assert gains == [min(k, TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP) for k in module.KP_INTERP]
      assert max(gains) == TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP
      # from 10 m/s up nothing changed
      for v in (10., 12.5, 15., 22., 30., 40.):
        assert np.interp(v, speeds, gains) == np.interp(v, module.INTERP_SPEEDS, module.KP_INTERP), v
      # below, the gain the controller reads is the cap
      for v in (0., 1., 3., 5., 7.5):
        by_size[big].pid.speed = v
        assert by_size[big].pid.k_p == TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP, v

  @pytest.mark.parametrize("flags", [LEGACY, STEER_TO_ZERO])
  def test_a_car_without_one_keeps_the_ladder(self, flags):
    for lac in build_controllers(flags).values():
      assert max(lac.pid._k_p[1]) == 250

  def test_the_shared_tables_are_not_edited(self):
    before = {version: list(module.KP_INTERP) for version, module in self.LADDERS.items()}
    build_controllers(LEGACY | TI)
    build_controllers(LEGACY)
    assert {version: list(module.KP_INTERP) for version, module in self.LADDERS.items()} == before
    assert before[0.0][0] == 250
