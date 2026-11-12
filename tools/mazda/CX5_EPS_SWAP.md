# 2017-21 CX-5 with a swapped 2022 EPS and the Torque Interceptor (TI1)

This branch is TheDeliSandwich's `mazda-long-ti1-testing` (ladelisandwich/openpilot @ `8b02b535`, his CX-9 GEN1 + TI1
work on StarPilot) plus the changes below, for a 2019 CX-5 whose EPS has been replaced by a 2022 CX-5 unit and which
has a MoreTorque Torque Interceptor fitted.

**Status: tested on a PC only.** Nothing here has been driven or run on a comma device. The checks are listed under
"How it was checked", the open questions under "Not verified".

## What already applied to a CX-5

The Torque Interceptor support is enabled by the **Enable Torque Interceptor** toggle (`TorqueInterceptorEnabled`), not by
the CX-9 platform: the controller, the 700-count ceiling and its stuck-command guard, the panda's TI safety mode and the
OBD CAN2 mode that puts the TI's bus on bus 1 (`panda/board/main.c`) all act on any GEN1 Mazda. What is CX-9 about the
branch is its calibration (below), not its code paths.

The car state now reads body signals decoded on the CX-9 (lane-keep button, lamps, reverse lamp, hazards, stalk
blinkers, parking brake and auto hold, TCS off, liftgate, wipers, horn). A message the parser subscribes to that never
arrives would hold `canValid` false and block engagement, so they were checked on comma's public 2017-21 CX-5 segments
(commaCarSegments, dongle `9f46a57f430101ea`, which happens to be an EPS-swapped CX-5 too): every one is on the bus,
`canValid` held for each whole minute after start-up, and parking brake, doors, liftgate and TCS off all read 0 while
driving. One did not match: the horn (below).

## What this port changes

1. **Fingerprint.** The 2022 EPS firmware (`KSD5-3210X-C-00`, `KBST-3210X-A-00`) is listed under `MAZDA_CX5`, with the
   public swapped car's engine (`PX2H-188K2-E`). Before, a swapped CX-5 matched neither CX-5 platform exactly: the EPS
   is a 2022 part and everything else is the body's year. The public car only matched by fuzzy fingerprinting, and a car
   whose other ECUs are less common would not have matched at all and come up as an unrecognized car (dashcam). It now
   matches `MAZDA_CX5` exactly.
2. **The low-speed floor follows the EPS firmware** (`values.steer_to_zero_eps`, flag
   `MazdaSafetyFlags.STEER_TO_ZERO_EPS`, CarParams only, never in the panda's safety param). Every gen1 EPS is the same
   hardware and the 45 kph floor is the firmware's, so a 2022 EPS steers to a stop in any body. The floor used to be set
   by platform name, which put it back on a swapped CX-5 whenever the TI toggle was off. The TI lifts the floor on any
   EPS, as before. An EPS that did not answer the firmware query falls back to the platform (`MAZDA_CX5_2022` only).
3. **The 2022 EPS's first activation is held** (steer-to-zero EPS only). zoompilot's logs of that EPS (docs in
   zoompilot `docs/zoompilot/mazda-lateral.md`, "LKAS_FAULT" and "The first-activation hold") show its first LKAS
   activation of a drive can raise `STEER_RATE.LKAS_FAULT` when it is made in the EPS's standby from a stop, at a crawl,
   while it delivers nothing. The bit never clears, and about five seconds later the front camera latches its own fault
   ("Front Camera Sensor System Malfunction" on the cluster) until the car is restarted. The EPS's share of the
   steering is then gone for the rest of the drive and the TI carries on alone (this branch ignores the camera's fault
   on TI cars). With the TI the controller steers at a standstill (`steerAtStandstill`), so that activation would be
   routine. carstate holds
   `eps_first_engage_hold` while the EPS has not delivered torque yet this drive, reports `LKAS_BLOCK` and
   `LKAS_TRACK_STATE` (byte 6 `0x14`), and the car is under 1 m/s; the controller sends the EPS zero meanwhile and
   climbs back from zero at the EPS's rates once it lifts. Only the EPS's channel (0x243) waits: **the TI's command
   (0x249) and what is reported to the lateral controller are unchanged**, so steering is not interrupted. On the CX-9's
   legacy EPS nothing changes.
4. **Horn.** The CX-9 sends 0 in `TURN_SWITCH.HORN` and 15 while pressed; the CX-5 sends 13 all the time (byte 6 is
   `0x5a`), which read as a horn held for the whole drive. Only 15 counts now. It matters only with **Horn Tap
   Bookmarks** on.

Without the TI toggle a swapped CX-5 now drives like a `MAZDA_CX5_2022` (no floor), which is what forcing the 2022
fingerprint used to be for. zoompilot's other 2022-EPS protection, the latch that stops requesting torque after 200 ms
of zero delivery, was not ported: on this EPS a block also means "no request arriving", and without zoompilot's other
changes the latch may never release.

## What is still the CX-9's calibration

- **TI ceiling 700 and the stuck-command guard** (`torque_interceptor.py`): the 0x11 `LKAS_STUK` rule is the TI's own
  firmware and applies to any car. The 650-count point where the EPS stops answering more was measured on the CX-9's EPS.
- **The TI's speed shaping of the torque controller** (`latcontrol_vehicle_tunes.py`, `MAZDA_TI_*`): fitted on CX-9
  drives, where the EPS's own LKAS channel is off below 45 kph and only the TI steers. On a swapped EPS both channels act
  at every speed, so below 45 kph the car will likely respond more strongly than the tune assumes. If low-speed turns
  feel twitchy or oscillate, `MAZDA_TI_ERROR_SCALE_V` is the first number to lower. Its author notes that zoompilot's
  learned table for the 2022 CX-5 EPS has the same shape.
- **Torque base:** the CX-5 uses the CX-9 2021 torque values (upstream's substitution). Turn on **Force Auto-Tune On**
  (Lateral settings): Mazda is not in torqued's allowed cars, so the learned values only apply through that toggle. On
  the CX-9 they differed from the fixed ones by about 20 % (`HANDOFF.md`).
- **Radar emulation and hybrid longitudinal** (optional toggles): they record this car's own radar frames at start-up;
  the fallback templates are the CX-9's.

## Installing

The branch ships prebuilt device binaries (the `prebuilt` file). This port changes Python only, so nothing needs
rebuilding, and the panda firmware is unchanged: `panda/board/obj` was built from `5b79ed8c`, which already has the
Mazda TI safety mode. The comma installer's short URL always clones a repository named `openpilot`
(`installer.comma.ai/<user>/<branch>`), or install over SSH:

```
ssh comma@<device ip>
cd /data && rm -rf openpilot
git clone --depth 1 -b <branch> https://github.com/<user>/openpilot.git openpilot
sudo reboot
```

## Settings

- Vehicle: **Enable Torque Interceptor** on. Leave **Enable Radar Interceptor** off; radar emulation and hybrid
  longitudinal are optional.
- Lateral: **Force Auto-Tune On**.
- If the car still comes up unrecognized (an ECU version not listed): Vehicle, **Disable Fingerprinting** with
  **Car Model** `MAZDA_CX5`; the EPS firmware is still read, so the swap is still detected. Then send the firmware list
  (first-drive check 1) so it can be added.

## First drive

1. Parked, after a start: check the fingerprint and the flags.

   ```
   cd /data/openpilot && source launch_env.sh && PYTHONPATH=/data/openpilot python3 -c "
   from cereal import car
   CP = car.CarParams.from_bytes(open('/data/params/d/CarParams', 'rb').read())
   print(CP.carFingerprint, 'fuzzy', CP.fuzzyFingerprint, 'flags', CP.flags, 'minSteerSpeed', CP.minSteerSpeed)
   for fw in CP.carFw: print(fw.ecu, fw.fwVersion)"
   ```

   Expected: `MAZDA_CX5`, fuzzy `False`, flags `1033` with the TI toggle on (GEN1 1 + TI 8 + steer-to-zero 1024; `1025`
   with it off), `minSteerSpeed` 0. Flags without 1024 means the EPS firmware is not in `STEER_TO_ZERO_EPS_FW`: note
   the eps line.
2. The TI: `tools/mazda/ti_steer_report.py` and `ti_fault_dump.py` (see `HANDOFF.md`) read its state and violation
   codes from the logs.
3. Engage first on an empty road above 60 kph, then in a town loop. Stop and pull away a few times early in the drive:
   that is where the first-activation hold acts. Afterwards look for `STEER_RATE.LKAS_FAULT` (byte 6 bit 5) and the
   camera's `CAM_LKAS.ERR_BIT_1` in the logs. If they ever appear, note the route and time.

## How it was checked

- 21 new tests (`opendbc/car/mazda/tests/test_eps_swap.py`): the public swapped car's firmware matches `MAZDA_CX5`
  exactly; the floor follows the EPS firmware; the hold's conditions; the CX-5's horn bits; and the controller holding
  only the EPS channel and climbing back inside the panda's limits. The existing Mazda tests (73) and the lateral
  tests (223) pass unchanged. The fingerprint and docs test files fail as on the unmodified branch (Ford, Mazda
  `shiftByWire` and query timing; doc names): none of those failures is about this change.
- The controller's real 0x243 frames, through the compiled panda Mazda safety model (param GEN1 + TI): no refusal over
  a hold at a stop, its release, a reversal, a driver pushing against the command, a let-go and a lateral cut, on the
  swapped EPS with and without the TI and on the legacy EPS. A deliberate 30 to 500 count jump is refused, so the check
  can fail.
- The public CX-5 logs replayed through the car state: exact fingerprint, steer-to-zero flag, no floor; the hold comes
  on at each stop in the EPS's standby and goes off when the standby lifts or the car rolls past 1 m/s.

## Not verified

- Nothing has been driven, and nothing has run on a device.
- This car's own firmware versions: the public car is a different CX-5. Whether its EPS reports one of the listed 2022
  versions is check 1 above.
- That the hold is enough on this EPS with a TI fitted. zoompilot's evidence comes from the 2022 EPS without a TI, and
  with a TI the EPS's torque sensor carries the TI's bias as well as the driver's hand.
- How the swapped EPS and the TI add up below 45 kph, and so how the CX-9's low-speed shaping feels on this car.
- The TI's torque reading on this car, and its violation codes, beyond what the CX-9 showed.
