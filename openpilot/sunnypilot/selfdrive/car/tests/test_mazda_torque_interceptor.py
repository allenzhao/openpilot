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
from opendbc.car.mazda.values import CAR, MazdaFlags, MazdaSafetyFlags, TorqueInterceptorCause, TorqueInterceptorState
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import setup_interfaces as opendbc_setup_interfaces
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import latcontrol_torque as latcontrol_torque_v1
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.car.card_ext import CardExt, fill_torque_interceptor
from openpilot.sunnypilot.selfdrive.car.interfaces import _reset_torque_caches_on_actuator_change, initialize_params
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP, TUNE_SWAP_INACTIVE_FRAMES, \
  ControlsLateralZP
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_torque_v0
from openpilot.sunnypilot.selfdrive.controls.lib.tests.torque_harness import DT, LAF, make_ci
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
from openpilot.sunnypilot.selfdrive.selfdrived.events_zp import EVENTS_ZP, TORQUE_INTERCEPTOR_CHIME_T, torque_interceptor_not_ready_alert

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
                                ti_state=int(TorqueInterceptorState.OFF), ti_violation=0x11, ti_error=0, ti_version=0x10,
                                ti_unfit_cause=TorqueInterceptorCause.VIOLATION, ti_unfit_code=0x11, ti_dropping_out=True,
                                eps_undelivered=False)
    fill_torque_interceptor(msg, car_state)
    ti = msg.zoompilot.torqueInterceptor
    assert (ti.notReady, ti.heard, ti.ready, ti.state, ti.violation, ti.error, ti.version) == (True, True, False, 1, 0x11, 0, 0x10)
    assert (ti.cause, ti.causeCode, ti.droppingOut, ti.epsUndelivered) == ("violation", 0x11, True, False)
    # and the other way about: each flag from its own attribute
    car_state.ti_dropping_out, car_state.eps_undelivered = False, True
    fill_torque_interceptor(msg, car_state)
    assert (ti.droppingOut, ti.epsUndelivered) == (False, True)

  def test_the_cause_is_opendbcs_in_its_order(self):
    # card writes the number; a reordered enum on either side would name the wrong reason
    schema = custom.CarStateZP.TorqueInterceptor.Cause.schema.enumerants
    assert {name.lower(): value for name, value in schema.items()} == \
           {cause.name.replace("_", "").lower(): cause.value for cause in TorqueInterceptorCause}

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
    # values no default would give, so a fill that reads nothing shows
    CS.ti_not_ready = CS.ti_fresh = CS.ti_ready = CS.ti_dropping_out = CS.eps_undelivered = True
    CS.ti_state, CS.ti_violation, CS.ti_error, CS.ti_version = int(TorqueInterceptorState.RUN), 0x33, 7, 0x10
    CS.ti_unfit_cause, CS.ti_unfit_code = TorqueInterceptorCause.RAMP_DOWN, 5
    event = messaging.new_message('carStateSP')
    fill_torque_interceptor(event.carStateSP, CS)
    ti = event.as_reader().carStateSP.zoompilot.torqueInterceptor
    read = (ti.notReady, ti.heard, ti.ready, ti.droppingOut, ti.epsUndelivered, ti.state, ti.violation, ti.error, ti.version,
            ti.cause.raw, ti.causeCode)
    assert read == ((True, True, True, True, True, 3, 0x33, 7, 0x10, int(TorqueInterceptorCause.RAMP_DOWN), 5) if enabled else
                    (False, False, False, False, False, 0, 0, 0, 0, 0, 0))

  @pytest.mark.parametrize("enabled", [False, True])
  def test_card_fills_it_every_frame_through_its_own_hook(self, enabled):
    # the one call site: a wrong argument there is an exception in card at 100 Hz on
    # interceptor cars only
    CP, CP_SP = mazda_params([{"TorqueInterceptorEnabled": enabled}])
    CI = CarInterface(CP, CP_SP)
    CI.CS.ti_not_ready, CI.CS.ti_state = True, int(TorqueInterceptorState.OFF)
    helper = SimpleNamespace(update_plan_regime=lambda plan, cc: None, reconcile_setpoint_with_dash=lambda cs: None,
                             cruise_arbiter=SimpleNamespace(fill_msg=lambda cs_sp: None), icbm_latch=SimpleNamespace(activation="passive"))
    card_ext = CardExt.__new__(CardExt)
    card_ext.sm = {'longitudinalPlanSP': None, 'carControlSP': None}
    card_ext.v_cruise_helper, card_ext.controller, card_ext.car_state = helper, CI.CC, CI.CS
    event = messaging.new_message('carStateSP')
    card_ext.update_v_cruise_post(structs.CarState(), event.carStateSP)
    ti = event.as_reader().carStateSP.zoompilot.torqueInterceptor
    assert (ti.notReady, ti.state) == ((True, 1) if enabled else (False, 0))


def car_events(flags) -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = 'mazda'
  CP.flags = int(flags)
  return CarSpecificEventsSP(CP, structs.CarParamsSP(), params=SimpleNamespace(get_bool=lambda key: False))


def car_state_sp(not_ready: bool, eps_undelivered: bool = False):
  cs_sp = messaging.new_message('carStateSP').carStateSP
  cs_sp.zoompilot.torqueInterceptor.notReady = not_ready
  cs_sp.zoompilot.torqueInterceptor.epsUndelivered = eps_undelivered
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

  def test_beside_one_the_eps_not_delivering_alone_is_downgraded_as_without_it(self):
    # steer-to-zero firmware: the fault is the EPS's non-delivery, the interceptor fit and
    # steering on. Shown as the same car shows it without the interceptor
    ev = events(EventName.steerTempUnavailable)
    car_events(STEER_TO_ZERO | TI).update(structs.CarState(), ev, car_state_sp(False, eps_undelivered=True))
    assert ev.has(EventName.steerTempUnavailableSilent)
    assert not ev.has(EventName.steerTempUnavailable)
    assert not ev.contains(ET.SOFT_DISABLE)

  def test_on_legacy_firmware_nothing_downgrades_it(self):
    ev = events(EventName.steerTempUnavailable)
    car_events(LEGACY | TI).update(structs.CarState(), ev, car_state_sp(False, eps_undelivered=True))
    assert ev.has(EventName.steerTempUnavailable) and ev.contains(ET.SOFT_DISABLE)

  def test_every_zp_event_has_an_entry(self):
    # selfdrived indexes EVENTS_ZP and the counters built from it by name: a missing one is a KeyError
    events_sp = EventsSP()
    for name, value in EventNameZP.schema.enumerants.items():
      assert value in EVENTS_ZP, name
      assert value in events_sp.zp.event_counters, name


class TestAlert:
  @pytest.mark.parametrize("fields, line", [
    ({"cause": "silent"}, "No signal: check its cables and OBD power"),
    ({"cause": "malformed"}, "Unreadable reply"),
    ({"cause": "violation", "causeCode": 0x11}, "Fault code 11"),
    ({"cause": "violation", "causeCode": 0x12}, "Fault code 12"),
    ({"cause": "error", "causeCode": 0x2A}, "Error code 2A"),
    ({"cause": "notRunning", "causeCode": 0}, "Starting up"),
    ({"cause": "notRunning", "causeCode": 1}, "Starting up"),
    ({"cause": "notRunning", "causeCode": 2}, "Stood aside for the driver and not back"),
    ({"cause": "rampDown"}, "Ramping its torque down"),
    ({}, "Not taking requests"),
    # the reason it was last not fit, whatever the frame of the moment says: a healthy one
    # between dropouts and for the second after a latch clears
    ({"cause": "violation", "causeCode": 0x11, "heard": True, "ready": True, "state": 3}, "Fault code 11"),
    ({"cause": "silent", "heard": True, "ready": True, "state": 3}, "No signal: check its cables and OBD power"),
    # dropouts that keep coming
    ({"cause": "rampDown", "droppingOut": True, "heard": True, "ready": True, "state": 3}, "Keeps dropping out: ramping its torque down"),
    ({"cause": "silent", "droppingOut": True}, "Keeps dropping out: no signal"),
    ({"cause": "violation", "causeCode": 0x33, "droppingOut": True}, "Keeps dropping out: fault code 33"),
  ])
  def test_it_says_why_the_interceptor_was_last_not_fit(self, fields, line):
    msg = messaging.new_message('carStateSP')
    ti = msg.carStateSP.zoompilot.torqueInterceptor
    ti.notReady = True
    for key, value in fields.items():
      setattr(ti, key, value)
    for sm in ({'carStateSP': msg.carStateSP}, {'carStateSP': msg.as_reader().carStateSP}):
      alert = torque_interceptor_not_ready_alert(None, None, sm, False, 0, None)  # ty: ignore[invalid-argument-type]
      assert alert.alert_text_1 == "Torque Interceptor Not Ready"
      assert alert.alert_text_2 == line
      # heard once when it comes up: the steer fault beside it is silent for a driver holding the wheel
      assert alert.audible_alert == AudibleAlert.prompt

  def test_it_chimes_when_it_comes_up_and_not_for_as_long_as_it_stands(self):
    # soundd starts a sound when the alert's sound changes: carried on every frame, it would
    # silence a lane departure that takes the screen and replay each time the screen came back
    events_sp = EventsSP()
    msg = messaging.new_message('carStateSP')
    args = [None, None, {'carStateSP': msg.carStateSP}, False, 0, None]
    chime = round(TORQUE_INTERCEPTOR_CHIME_T / DT_CTRL)
    heard = []
    for raised in [True] * 300 + [False] + [True] * 150:
      events_sp.clear()
      if raised:
        events_sp.zp.add(EventNameZP.torqueInterceptorNotReady)
      alerts = events_sp.create_alerts([ET.PERMANENT], args)
      heard.append(alerts[0].audible_alert if alerts else None)
    assert heard[:chime] == [AudibleAlert.prompt] * chime
    assert set(heard[chime:300]) == {AudibleAlert.none}
    assert heard[300] is None
    # a new alert is heard again
    assert heard[301:301 + chime] == [AudibleAlert.prompt] * chime
    assert set(heard[301 + chime:]) == {AudibleAlert.none}

  def test_the_refusal_is_the_one_read(self):
    # carstate raises upstream's carNotReady beside it, for the selfdrive machine; both refuse
    # on the same frame and the driver should read which part of the car is not ready
    events_sp = EventsSP()
    events_sp.zp.add(EventNameZP.torqueInterceptorNotReady)
    upstream = events(EventName.carNotReady)
    msg = messaging.new_message('carStateSP')
    args = [None, None, {'carStateSP': msg.carStateSP}, False, 0, None]
    refusals = upstream.create_alerts([ET.NO_ENTRY], args) + events_sp.create_alerts([ET.NO_ENTRY], args)
    assert sorted(a.alert_text_2 for a in refusals) == ["Car Not Ready", "Torque Interceptor Not Ready"]
    assert max(refusals, key=lambda a: a.priority).alert_text_2 == "Torque Interceptor Not Ready"
    assert upstream.contains(ET.NO_ENTRY) and not upstream.contains(ET.SOFT_DISABLE)

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
