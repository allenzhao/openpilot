# Mazda Torque Interceptor

Support for the MoreTorque Torque Interceptor (TI, the original "TI1") on gen1 Mazdas: the cars
whose EPS firmware steers only above 45 kph, where below that the TI steers alone, and those on
steer-to-zero firmware (a 2022-on EPS, or an older body with one swapped in), where the EPS's
own command steers beside it at every speed. The code is in
`opendbc/car/mazda/{values,mazdacan,carcontroller,carstate}.py`, the panda's side in
`opendbc/safety/modes/mazda.h` and `panda/board/main.c`, and this tree's side in
`openpilot/sunnypilot/selfdrive/car/{interfaces,card_ext,car_specific}.py`,
`openpilot/sunnypilot/selfdrive/selfdrived/events_zp.py` and
`openpilot/sunnypilot/selfdrive/controls/controls_lateral_zp.py`.

**Status: not driven, and not yet run on a device.** Everything below is built from the
interceptor's published documentation, its author's openpilot fork (MoreTore/openpilot),
pdscomp's ports of it to earlier zoompilots (pdscomp/MorePilot, `zoom-more`, one of them on a
CX-5 with a steer-to-zero donor EPS) and the record TheDeliSandwich has kept of a driven CX-9
with a TI1. It is tested on a PC: the real car interface against the compiled panda safety
model, the firmware compiled for the panda's processor, public logs of an EPS-swapped 2017-21
CX-5, no car, no interceptor and no comma device. "Not verified" at the end lists what only
those can answer.

## What it is and where it sits

The TI goes between the steering column's torque sensor and the EPS (`TORQUE_IN`,
`TORQUE_OUT`) and adds a bias to the signal the EPS reads. The EPS then steers with its own
power assist, as it would for a driver's hand, at any speed: the legacy firmware's LKAS floor
(off going down through 45 kph, back on going up through 52) and its hands-off lockout apply
to the LKAS request on CAN, not to the torque sensor. In any state but run, and with no power,
a relay connects the sensor straight to the EPS (bypass) and the car steers as stock.

Its CAN side is daisy-chained into the cable between the OBD-II port and the comma harness
(the "Comma Power" link), so it lives on the OBD-II CAN pair. The panda reaches that pair as
**bus 1**, and only while it multiplexes CAN2 onto the OBD pins, which it otherwise does just
for the fingerprint. `MAZDA_PARAM_TI` in the safety param keeps it there
(`panda/board/main.c`, `set_safety_mode`).

The TI cannot be fingerprinted: it says nothing until it has found its keyed command stream.
The driver declares it with **Settings, Vehicle, Torque Interceptor**
(`TorqueInterceptorEnabled`), between drives. The toggle sets `MazdaFlags.TORQUE_INTERCEPTOR`
(64) and `MazdaSafetyFlags.TORQUE_INTERCEPTOR` (8). It is not in the sunnylink backup and
cannot be written remotely: it changes the panda's safety param.

## Protocol

From the TI's published safety and communication documentation.

**Command, `CAM_LKAS2` 0x249, bus 1, 100 Hz.** Bytes 0-1: the request plus 2048 in the low
12 bits. Bytes 2-3: the same again, in place of a checksum. Bytes 4-6: the discovery key
`C4 61 CE`. Byte 7: the option, `0x60` for normal run. `0x61` is a cold reset that clears
latched violations and `0x62` forces bypass; neither is ever sent and the panda refuses both.
The TI scans 0x200-0x2FF for the key, locks onto that address until its next reset and
answers one address up.

**Feedback, `TI_FEEDBACK` 0x24A, bus 1, 50 Hz.** Byte 0: driver torque plus 127, measured at
the sensor, before the bias. Byte 1: the same again. Byte 2: firmware version (0x01 and 0x10
have been seen). Byte 3: state. Byte 4: violation number. Byte 5: logic error code. Byte 6:
ramp-down, on firmware above version 1: the TI is taking its bias to zero by itself and
ignores the request until it reads zero. Byte 7 is unused (0x30 in captures).

| State | | |
|---|---|---|
| 0 | discover | booted, scanning for the key; bypass |
| 1 | off | start-up or recovery delay, or a latched violation; bypass |
| 2 | driver override | the driver is steering harder than it can pass; bypass |
| 3 | run | applying its bias |

What the TI does with the request:

- It clips it at 600 either way.
- A request that does not change, anywhere but 0 and 600, is `VIOL_LKAS_STUK`: off for 30 s.
  On record it latched after 1.84 to 2.0 s (TheDeliSandwich's CX-9: 22 of 22 at an unbroken
  800, which the TI clips to 600, and a -1 held for 2.0 s at a crawl), cleared in 29.5 s and
  was in run 2 s after that. A 10-count dip seen at 20 Hz restarted its count.
- No command for more than 100 ms is `VIOL_CAN_TMO`: off until 2 s after the stream is back.
- A request that changes faster than it allows, or an unsafe value, makes it ramp its bias to
  zero and wait for a zero request.

Violations by class: class 1 clears with a power cycle (or the reset key), class 2 by itself
30 s after the cause is gone, class 3 after 2 s. Class 2 also clears when the command stream
starts after more than half a second off: that is how a sensor error at key-on goes. The port
keeps the stream up for 5 s after the CAN ignition drops, so a quick restart does not stop it,
and a 31 latched at power-down (seen on TheDeliSandwich's car) can stand for up to 30 s:
"Fault code 31". The violation byte reads in hex as the code
the green LED flashes, which is the number in the alert on the device. Log tools print the
byte in decimal, the last column:

| Code | Name | Class | | In logs |
|---|---|---|---|---|
| 11 | LKAS_STUK | 2 | request did not change, off 0 and 600 | 17 |
| 12 | CAN_TMO | 3 | no command for over 100 ms | 18 |
| 13 | LKAS_ERR | 2 | request out of the window | 19 |
| 14 | LOW_VOLT | 3 | CPU voltage too low | 20 |
| 15 | TMR_ERR | 1 | periodic timer interrupt missing | 21 |
| 16 | MAIN_ERR | 1 | main loop too slow | 22 |
| 17, 18 | TORQUE_H, TORQUE_L | 2 | computed output outside its window | 23, 24 |
| 19 | WDT_ERR | 1 | watchdog | 25 |
| 21 | CPU_REG | 1 | pin or peripheral configuration | 33 |
| 22 | HIGH_TMP | 3 | CPU above 65 C | 34 |
| 23 | DATA_MEM | 1 | EEPROM CRC | 35 |
| 25 | TK_TRNSF | 2 | measured output off the computed one | 37 |
| 26 | ADC_STUK | 1 | ADC channel failed | 38 |
| 31 | HIN_ERR | 2 | sensor input out of its window | 49 |
| 32, 33 | TK_NLOAD, TK_WLOAD | 2 | feedback output out of its window | 50, 51 |
| 34 | CAN_ERR | 3 | any other CAN error | 52 |
| 35 | STCK_ERR | 1 | stack | 53 |
| 36, 37 | PANIC_MON, PANIC | 1 | panic circuit | 54, 55 |
| 41 | LOGIC | 1 | illegal internal state | 65 |
| 43 | CLK_STOP | 1 | oscillator stopped | 67 |

12 during the first seconds of a drive is ordinary: it is how the TI sits before the stream
starts.

## What the port sends

Two commands every 10 ms, from the same controller torque `u`:

- **The EPS's, 0x243**: `u` x 1200, clipped to the measured ceiling, limited against the
  driver, as without the TI. Two things differ on a TI car. `u` stops at the TI's rail
  (below), so the command tops out at 800 counts wherever the ceiling is higher: under about
  13 m/s, which on legacy firmware is only 12.5 to 13 m/s and on steer-to-zero firmware is
  1148 counts without the toggle. And the share it yields to an opposing driver runs over 15
  to 30 counts of the TI's torque reading, not 15 to 50 of the EPS's. On legacy firmware the
  command is zero while that EPS takes no request (below its floor, during its lockout), as
  it always was, and ramps from zero when it does again.
- **The TI's, 0x249**: `u` x 900, clipped to 600, at 6 counts per frame up and 15 down, limited
  against the driver with an allowance of 15 and a multiplier of 40. The clip, the rates and
  the driver limit are the TI author's shipped numbers; the firmware's own limit on the rate
  is not published, and 10/25 and 12/25 have been driven on TI1 units without a recorded
  ramp-down. The 900 is pdscomp's pairing on legacy firmware, an 800 count EPS command beside
  `u` x 600, on this fork's 1200 scale (on his steer-to-zero CX-5 he sent 1200:600 below
  14.2 m/s and 800:600 above 14.5). The author's own fork sends the TI the same count as the
  EPS command, clipped to 600, a third more per unit of controller torque. On legacy firmware
  `STEER_SCALE` 1200 is the second value to try if the TI feels weak, and the rail moves with
  it. On steer-to-zero firmware it would put the rail at 0.5 and stop the EPS command at 600;
  if tight low-speed turns run out there with the request on the rail, the next step is to
  let the EPS command run past the TI's rail to its own ceiling (1148), the TI resting on 600.

The TI is the actuator the controller reports (`actuatorsOutput.torque` = its command / 900),
because it is the one that acts at every speed. The lateral stack reads a rail and a slew of
its own for it (`CarControllerParams.STEER_RAIL_SCHEDULE`, `STEER_SLEW_SCHEDULE`, through
`opendbc/sunnypilot/car/lateral_tune.py`): the rail is 600/900 flat, so a saturated controller
puts exactly 600 on the wire, the one value besides 0 the TI lets a request rest on.

The TI's own rules, as the controller keeps them:

- **Nothing rests.** A request that has stayed inside a 40-count band for 1.2 s, the clip
  included, is held 20 counts nearer zero for 0.2 s, and the up rate brings it back: the shape
  driven on a TI1 (TheDeliSandwich's ceiling guard, 20 counts for 0.2 s every 1.4 s at a 700
  ceiling). A steady hand against the command pins the driver limit, a quiet road the
  controller's output and a tight turn the clip; any would be a stuck request. It does not
  depend on how the TI samples the request or what change it counts: the tests hold it to
  readers at 100, 50 and 20 Hz that count any change or only one of 10 counts, with the clip
  exempt or not, and none sees a request still for 1.84 s. It costs a few counts on average.
- **Nothing below 1 m/s.** A stopped car holds its command still. On legacy firmware lateral
  is off below it too (`minSteerSpeed` 1.0). On steer-to-zero firmware `minSteerSpeed` stays
  0: lateral runs from 0.3 m/s as it does without the TI, the EPS command alone carries it,
  and the reported actuator reads zero there.
- **Every frame, engaged or not**, zero when not steering.
- **A release walks down at 15 a frame** when openpilot lets go while the panda still lets it
  steer: a software pause, the MADS button, slowing through 1 m/s. A cut is a tug at low
  speed. A brake, a cruise cancel or main off reaches the panda first: the frame on its way is
  refused and the next is a zero. That is a cut, as on every openpilot port and in the
  author's fork.
- **Zero at once when the TI is not fit**, and the ramp restarts from zero when it is again:
  the panda's reference is at zero by then too. The panda hears an unfit frame a cycle before
  the controller does, so the one command on its way is refused.
- **A refused frame is answered with a zero** before the ramp restarts. The panda reports each
  0x249 it refuses (src 193); a zero it admits whatever the state (bar a relay malfunction,
  when it sends nothing at all), so even a run of refusals leaves the TI a stream.

Fit means: feedback heard within 70 ms (two lost frames and a cycle of jitter), both torque
bytes equal, a nonzero version, state run, no violation, no error, no ramp-down.

Driver torque is the TI's reading (the EPS's is the biased signal), in carstate and in the
panda. `steeringPressed` is above 6 for five frames, or above 20 at once. When the TI goes
silent its last reading stands for 1.2 s: until its own watchdogs have put it in bypass the
EPS still reports the bias, and that read as a press would soften the alarm for this very
fault. What stands is whether the driver was steering, not the last sample: a sample counts
on the cycle that carries it and the next, as when the TI talks, so one noise peak before a
silence is not a press. After that, and with a TI never heard, the EPS's own reading and
threshold are used. Through the silence the EPS command yields to whichever of the two
readings shows more torque against it: a driver who starts pushing then shows only on the
EPS's own.

The cluster's hands-on-wheel frame mirrors the steer-required alert at any speed, not only
above the EPS's floor.

What the stock port warns of and the TI steers through is off on a legacy-firmware TI car:
the low-speed alert, the car's LKAS setting, the camera's fault as a permanent steer fault.
Above the EPS's floor that is not free. A camera fault takes the EPS's share of the steering
for the rest of the drive, and the car's lane keep switched off takes it until it is switched
back on (the EPS re-arms in about 3 s); the TI carries on alone, with nothing shown.

On steer-to-zero firmware the EPS command is half the steering at every speed, and its loss is
shown as it is without the toggle: the car's lane keep off is "LKAS is off" (MADS paused, or
no entry), the camera's fault is "LKAS Fault: Restart the car", and the road-speed
non-delivery alert is the "Steering Assist Temporarily Unavailable" banner without the
escalation, the TI steering on while the latch holds the EPS command at zero. The EPS's re-arm
grace covers its own block only, never a TI fault. After every stop that EPS sits in its
standby until about 3 s after pulling away, and the TI steers alone from 1 m/s; beside the TI
the low-speed gain is capped, so a crawl's request can stay under the latch's 200 counts, and
in the standby any request counts toward it instead.

## Faults and what the driver sees

- Not fit while moving for 0.3 s: `steerFaultTemporary`. Upstream's take-control alert runs,
  as for any steer fault: lateral drops at once, and three seconds of it disengage cruise as
  well. With a hand on the wheel upstream softens that alert to a banner or drops it, and the
  TI reads a resting hand as one.
- A shorter dropout costs the TI's command for those frames and its ramp back from zero
  (0.9 s from 540 counts, a second from the clip), not the driver's lateral. Unfit frames add
  up until the TI has been fit for that second, and a dropout that comes inside it is charged
  0.1 s on top of its own frames: it restarts a command that had not climbed back. So
  dropouts that keep coming are a fault too (20 ms twice a second: on the fourth), and the
  fault is held a second after the last of them.
- Dropouts more than a second apart never add up to a fault, but each still restarts the
  command. Every one is held against the TI for 3 s; 9 s of that owed, which is more often
  than once in 3 s kept up (every 1.1 s: the fifth), names the TI without a fault, and it
  stays named until all of it is paid off, at most 12 s after the last.
- A malformed frame (the two torque bytes unequal, version zero, a state past run) is a fault
  at once and for a second after: on that frame the panda has dropped cruise and MADS lateral
  alike, and steering comes back only when the driver engages again.
- Driver override is not a fault while the driver holds the wheel, or for a second after on
  legacy firmware, 2.5 s on steer-to-zero firmware, where the EPS steers on meanwhile. It has
  sat in driver override about 1.8 s at start-up with nobody on the wheel (pdscomp's capture);
  how long it takes after a real override is not known.
- On steer-to-zero firmware a TI fault drops lateral although the EPS could steer alone, and
  three seconds later cruise as well. That is chosen for the first drives: simple and loud.
  Letting the EPS carry on, the TI's command at zero and its name on the screen, is the next
  step once drives show how often it drops out.
- At a stop a TI that answers is not counted against: it is sent nothing there. What was
  counted while the car moved is kept, so a latch is still a fault on the first frame the
  car moves again. In stop and go under a latch with MADS on, that is the take-control alarm
  at every pull-away, until MADS is switched off or the car has moved for three seconds.
- Not fit for 2 s in all while moving, silent for 2 s at any speed once the command stream
  has run for 8 s, or dropping out again and again: **Torque Interceptor Not Ready**, engaged
  or not, with a chime when it comes up and the reason the TI was last not fit. Nothing
  engages while it shows, cruise or MADS lateral ("openpilot Unavailable, Torque Interceptor
  Not Ready"); what is engaged is left to the steer fault. It stays through a stop and clears
  a second after the TI is fit:

| Second line | |
|---|---|
| No signal: check its cables and OBD power | no feedback at all. Unplugged, unpowered, or the panda firmware is not this branch's and bus 1 is not on the OBD pins |
| Fault code NN | its violation number, table above |
| Error code NN | its logic error byte |
| Starting up | discover or off with no violation |
| Stood aside for the driver and not back | driver override with nobody on the wheel for over a second |
| Ramping its torque down | it judged the request unsafe and is waiting for a zero |
| Unreadable reply | frames that fail the form check |
| Keeps dropping out: (reason) | fit on most frames, with dropouts that keep coming |

- A TI silent for more than a second also fails the panda's RX checks. That is **Controls
  Mismatch**: an immediate disable of cruise as well as steering when engaged (stock cruise is
  canceled), and a refused entry otherwise. It is on purpose: the panda limits both commands
  against the driver torque the TI reports and has no other source for it. Turn the toggle off
  to drive without the interceptor.

The named alert never disables anything itself. A zoompilot event that soft-disables would
drop MADS lateral in one frame with no alert while cruise is engaged. Its no-entry for cruise
is upstream's own `carNotReady`, which carstate raises beside it.

`carStateSP.zoompilot.torqueInterceptor` logs every frame what the TI said in its last frame
(heard, ready, state, violation, error, version) and what carstate made of it: notReady,
droppingOut, cause with causeCode, the reason it was last not fit, kept until it has been fit
for a second and is no longer named, and epsUndelivered, the steer fault being the EPS's
non-delivery alone.

## The panda

- 0x249 is admitted on bus 1 only, 8 bytes, with clear high nibbles, request equal to its copy
  and key `C4 61 CE 60`.
- Torque needs openpilot steering (the same test as 0x243, MADS included) and a fit TI heard
  within 100 ms. The controller's 70 ms is inside that, so one that has stopped hearing the
  TI stops before it is refused. Staleness latches until the next fit frame. The latch is
  kept by every frame in the RX checks and every frame the host sends, so the microsecond
  timer's wrap, 71 minutes on, does not revive an old one while the car or the host is
  talking.
- Limits 600, 6 up, 15 down, 192 per 250 ms, allowance 15, multiplier 40, on a history of its
  own: `steer_torque_cmd_checks`' globals belong to 0x243. The down rate must not exceed the
  controller's, or every driver-override winddown is refused.
- `TI_FEEDBACK` is an RX check on bus 1. Its quality flag is whether the frame is well formed
  (bytes 0 and 1 equal, version nonzero, state at most 3). Off and driver override are valid
  frames that carry no permission; failing the flag on them would be a controls mismatch at
  every stop. A malformed frame takes MADS lateral as well as `controls_allowed`, until the
  driver engages again: it is the driver torque both commands are limited against.
- Driver torque is sampled from `TI_FEEDBACK`, not 0x240. For the second or two between a TI
  going silent and the RX check's lag, 0x243 is limited against its last readings, the same
  tolerance every car's driver torque message has.
- 0x249 is never forwarded between the car and the camera.
- Firmware: if nothing acknowledges bus 1 (the TI's CAN pair open with power still there) the
  frames for it would queue until the host could write to no bus at all, in about 2.5 s. The
  8 Hz tick drops that queue once more than 50 frames wait in it (`can_tx_limit_backlog`,
  called from `panda/board/main.c`; `tests/usbprotocol/test_comms.py` runs it on the host
  build of the queues).

Without `MAZDA_PARAM_TI` none of this is reachable and the mode is as it was, with one
exception that is not about the TI: while the panda refuses 0x243 because lateral is off, it
now also restarts that command's ramp. Under MADS with cruise engaged the rate checks used to
follow a host's request through the refusals, and the first frame after lateral rose could be
a step.

## Lateral tune

- Tune scale stays 1.5 and the offline tune is the car's own. The platforms' speed-binned
  seeds are not used: they were learned on the EPS command alone, most behind steer-to-zero
  firmware. torqued's own cache is cleared when the toggle changes
  (`_reset_torque_caches_on_actuator_change`).
- Legacy firmware: the default bins on the offline values, under seed version 64. Below the
  floor the TI steers alone, and 2.64 is about what one gives.
- Steer-to-zero firmware: both actuators act from the same `u`, so lateral accel per unit
  torque is the two shares added. The bins start at that sum, under seed version 65, with a
  crawl bin from 2 m/s: the EPS's learned table on this firmware plus the TI's share from a
  CX-9's TI-only fits below 45 kph (2.6 and 2.05 per unit at 800 counts, so 2.9 and 2.3 here,
  less a fifth for the lighter car), falling with speed above that, where it is not on record:

  | Bin, m/s | 3.5 | 6.5 | 10 | 15 | 21 | 26.5 | 32 | 37.5 |
  |---|---|---|---|---|---|---|---|---|
  | lat accel factor | 3.0 | 4.4 | 4.6 | 3.3 | 2.9 | 3.1 | 3.2 | 3.3 |
  | EPS alone, same firmware | 1.8 | 2.08 | 2.3 | 1.8 | 1.75 | 2.0 | 2.1 | 2.3 |
  | friction | 0.075 | 0.075 | 0.075 | 0.075 | 0.07 | 0.065 | 0.06 | 0.06 |

  A seed under the plant's gain makes every term too strong: in a closed-loop simulation of
  this plant the flat 2.64 overshot 18 to 28 % at 8 to 12 m/s and rang at 1.6 Hz, and the
  learner's clip (+-30 % of the seed) could not climb out. The table did not ring there, and
  held if the TI's share is half the estimate (10 to 17 % weak) or a quarter more. Unknown
  where the estimate leans, it leans high: a seed over the gain is only weak at first. The
  bins seed from the table, not from the offline values, so custom offline values do not move
  them; Less Restrict Settings for Self-Tune widens their clip.
- The proportional ladder is held to 4.0 on TI cars
  (`TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP`), on every tune. It climbs to 250 at 1 m/s, where no
  Mazda EPS ever delivered; pdscomp's CX-8 owner logged 150 to 220 command reversals a minute
  under 30 kph with it, and 4.0 was the knee of a replay sweep on that car's drives. From
  10 m/s up it is unchanged.

## Constants

| Name | Value | From |
|---|---|---|
| `STEER_MAX` | 600 | the TI's clip (its documentation) |
| `STEER_DELTA_UP` / `DOWN` | 6 / 15 | TI author's fork |
| `STEER_DRIVER_ALLOWANCE` / `MULTIPLIER` | 15 / 40 | TI author's fork |
| `STEER_SCALE` | 900 | 600 x tune scale 1.5: pdscomp's 800:600 on legacy firmware |
| `STEER_HELD_FRAMES`, `STEER_DIP`, `STEER_DIP_FRAMES` | 1.2 s, 20, 0.2 s | TheDeliSandwich's driven guard; latches on record from 1.84 s |
| `MIN_SPEED` | 1.0 m/s | pdscomp: 5 of 9 latches entered at a stop |
| `STEER_THRESHOLD`, `STEER_OVERRIDE_NOW` | 6, 20 | TI author; pdscomp (noise peaks 10 to 16) |
| `FEEDBACK_TIMEOUT` | 70 ms | two lost 50 Hz frames and a cycle of jitter; panda 100 ms |
| `FAULT_T` | 0.3 s | chosen, over the 242 ms feedback pauses pdscomp logged |
| `REFIT_T` | 1.0 s | the command's climb from zero to its clip, 600 / 6 frames |
| `RESTART_T` | 0.1 s | chosen: three more dropouts inside the refit window are a fault |
| `DROPOUT_T`, `DROPOUT_ALERT_T` | 3 s, 9 s | chosen: named at more than one dropout in 3 s, kept up |
| `BYPASS_T` | 1.2 s | the TI's documented worst case to bypass, 1.09 s |
| `NOT_READY_ALERT_T`, `BOOT_T` | 2 s, 8 s | chosen; on record in run 2 to 3 s after the first frame |
| the alert's chime | 1 s | chosen: heard when it comes up, silent while it stands |
| `DRIVER_OVER_RELEASE_T`, `_STZ_T` | 1 s, 2.5 s | chosen; 1.8 s in driver override at start-up on record |
| low-speed KP cap | 4.0 | pdscomp's replay sweep, one CX-8 |
| steer-to-zero seeds | table above, version 65 | EPS table plus a TI share estimated from a CX-9 |
| bus 1 backlog | 50 frames | chosen; the host's writes stop at 246 |

## Installing on a comma 3X

The branch is source: the device builds it at first boot. The comma installer's short URL
always clones a repository named `openpilot`: a copy of the branch pushed to a fork named
`openpilot` installs from the setup screen (Custom Software, `installer.comma.ai/<you>/<branch>`).
A fork named `zoompilot` is installed over SSH, onto a device that already runs openpilot (it
keeps `/data/continue.sh`, which starts `/data/openpilot`):

```
ssh comma@<device ip>
cd /data && rm -rf openpilot
git clone --depth 1 --recurse-submodules -b zoompilot-ti https://github.com/<you>/zoompilot.git openpilot
sudo reboot
```

The first boot should compile for some minutes and then flash the panda with the firmware
built from this branch's `panda` and `opendbc_repo`; it is that firmware that puts bus 1 on
the OBD pins. If the screen says openpilot failed to build, it did not. The branch expects
AGNOS 19.9 (`launch_env.sh`): on an older OS the updater runs first, and needs a network.
With Quickboot Mode on, delete `/data/openpilot/prebuilt` after updating the branch, or
nothing rebuilds.

The toggle appears in Settings, Vehicle once the car is known: after one drive, or after
picking the car there. Until it is on the TI stays in bypass and the car drives as stock.

## First drive

On steer-to-zero firmware the car is complete without the TI: drive it first with the toggle
off. That drive is the reference, and the toggle off is the way home if the TI misbehaves. If
lateral is refused with "LKAS is off", the car's lane keep is off; switch it on. If it is on
and still refused, the camera reports `LANE_LINES` 0 with it on, which no older camera in the
public logs does: note it, as it stops the toggle-on drive the same way. Do the first
drives with openpilot longitudinal off: with it on, a silent TI leaves the trip with no cruise
of any kind, since the radar is taken over at the first stop and nothing can engage.

1. Parked, engine on, toggle on. Check what the car was taken for:
   `python3 -c 'import openpilot.cereal.messaging as m; from opendbc.car.structs import car; from openpilot.common.params import Params; from opendbc.car.mazda.values import MazdaFlags as F; cp = m.log_from_bytes(Params().get("CarParams"), car.CarParams); print([f.name for f in F if cp.flags & f], cp.minSteerSpeed)'`.
   A swapped steer-to-zero EPS shows `STEER_TO_ZERO_EPS` and 0.0. `LEGACY_FW_EPS` and 1.0
   means its firmware was not read at that start; the car then runs as legacy until the device
   restarts (the parameters are cached), so reboot it. `safetyParam` is 10 either way.
2. `python tools/scripts/car/can_printer.py --bus 1` should show
   0x24A at 50 Hz with byte 3 reaching 03 within a few seconds: the TI only answers a stream
   it has found. On record it goes from off with violation 0x12 through driver override to
   run in about 2.5 s. Note byte 2 (the version, 0x01 or 0x10), byte 6 (0 in run) and byte 7
   (0x30 on one unit). Some 130 ids at full rate besides mean the OBD pair is the vehicle's own
   CAN, as on the 2017-21 CX-5 in the public logs. Our own 0x249 shows at 100 Hz under
   `--bus 129` (what the panda queued for bus 1), and under `--bus 193` only what it refused.
   No 0x24A: the alert's first line of the table above.
3. `python tools/scripts/dump.py carStateSP --values carStateSP.zoompilot.torqueInterceptor`
   shows the decoded state, in decimal: version 16 is 0x10, violation 18 is code 12. Note the
   version. `pandaStates` should show `safetyParam` 10 (11 under openpilot longitudinal).
4. Hands off and then turning the wheel by hand, compare `carState.steeringTorque` (the TI's
   reading) with `STEER_TORQUE_SENSOR` in 0x240 on bus 0. If the two are not in the same
   units, the driver thresholds here (6, 15, 20, 30) need rescaling. With the TI steering,
   note what 0x240 reads hands off: that is the bias (on a CX-9 it rose 0.05 counts per TI
   count, to about 30 at 600). `carState.steeringPressed` must stay false hands off while the
   TI steers; if it chatters, the threshold needs raising.
5. Engage on an empty road above 60 kph first, where the EPS's own LKAS also acts and a stock
   zoompilot drive is the reference, then 30 to 50 kph with hands ready. On steer-to-zero
   firmware a 1.5 to 2 Hz shimmy of the wheel there means the bins' lat accel factor is too
   low for the plant; turn-in that runs wide and catches up late means it is high. Compare
   tight low-speed turns with the toggle off: the EPS command stops at 800 counts with it on.
6. Somewhere safe, steer through it hard enough to override and let go. A take-control alarm
   a second or two after letting go means the TI took longer to return to run than the
   excusal allows (1 s on legacy firmware, 2.5 s on steer-to-zero).
7. After the drive look at `torqueInterceptor.violation` over time. A 17 (code 11) is a
   request that rested somewhere. Count the times `torqueInterceptor.ready` falls while
   lateral is active and the driver's hands are off: each is the command restarting from
   zero, and `cause` says what the TI gave as the reason (6 is a ramp-down: it judged the
   request's rate unsafe).
8. If the learned lat accel factor sits pinned at its limit after a few drives, the
   interceptor's gain is outside what self-tune may learn: turn on Less Restrict Settings for
   Self-Tune (Settings, Steering, Torque). On steer-to-zero firmware compare the learned bins
   (seed version 65) with the table: the factor less the EPS's own is the TI's share by speed.
9. Listen for the cluster's hands-on-wheel chime below 45 kph on legacy firmware: the frame
   was never sent there before.

Switch openpilot longitudinal on only after `TI_FEEDBACK` has been seen reaching run over a
few drives.

## Not verified

- Nothing has been driven. No frame here has been seen by a TI.
- Nothing has run on a device: this branch has not been cloned, built or booted on a comma
  3X, the firmware has been compiled and never flashed, and bus 1 on the OBD pins has not
  been observed.
- Whether the TI's torque reading is in the EPS sensor's units. The two forks' pressed
  thresholds (6 against 15) suggest it may not be.
- What this car's 0x240 reads while the TI injects (on a CX-9's legacy EPS: 0.05 counts per
  TI count up to about 600, then 32 to 34 whatever is sent), and whether legacy firmware minds
  a nonzero 0x243 at the moment its block lifts.
- The TI's gain per count against the EPS's own LKAS, by speed, and so which scale is right:
  900 here and in pdscomp's port, 1200 by the author's fork. On legacy firmware above the
  floor both act and below it only the TI does, so the plant steps at 45/52 kph, inside one
  speed bin. On steer-to-zero firmware the seed table is the EPS's learned share plus an
  estimate of the TI's from another car and EPS; above 15 m/s the TI's share is not on record.
- Whether one count for one frame counts as a change to the TI, and whether a held 600 is
  exempt on a given unit. The guard no longer depends on either.
- When driver override happens on an ordinary push, and how long the TI takes to return to
  run after one. Longer than the excusal (1 s on legacy firmware, 2.5 s on steer-to-zero) and
  each hard override ends in a short steer fault.
- The firmware version of any given unit, and byte 6 on version 1. Any nonzero version is
  accepted, and ramp-down is read only above version 1, as the author's fork does. A
  version 1 unit ramping its bias down gives no sign of it and waits for a zero request.
- What the TI does in its 229 to 242 ms feedback pauses, and how often it ramps down.
- The low-speed gain. 4.0 is one car's replay sweep on a different chassis.
- Whether the car's OBD-II CAN pair is the same bus the camera sits on. On the 2017-21 CX-5
  in the public logs bus 1 carries the vehicle's own frames, so there 0x249 and 0x24A are on
  the vehicle bus too, which the forks that drive TIs have always done, and a TI still in
  discover scans the vehicle's own 0x200-0x2FF frames for its key: the panda does not look
  for the key in anything but 0x249. Whether the gateway passes 0x249 on is not known.
- Openpilot longitudinal beside the TI. A malformed `TI_FEEDBACK` frame drops the panda's
  permission, and with it every engaged radar stand-in frame, for the 2 s until selfdrived's
  mismatch counter disengages; what the body does with its cruise active and no 0x21B for 2 s
  is not known. It needs a faulty unit (none in pdscomp's 11,000 frames).
- The bus 1 backlog bound in the firmware: modeled on the host build of the queue code, not
  run on a panda with an open bus.
- comma's servers. Upstream's device flag `ignore_uploads` ("Uploads ignored due to a
  potential detected safety policy violation") exists; whether a TI trips it is not known.
  Nothing here hides the interceptor from the logs.

## Tried and rejected

- **One count for one frame every 25 frames, the clip left alone**, the first stuck-request
  guard. It held only if the TI compares every frame and counts a one-count change, and its
  nudge always landed on the same frame parity: a 50 Hz reader on the other one never saw it.
- **Masking the EPS's own faults on steer-to-zero firmware as on legacy.** It hid the loss of
  half the steering: with the car's lane keep off the EPS echoes the request and delivers
  nothing, and nothing showed.
- **A banner instead, steering on with the TI alone**, for the car's lane keep off or the
  camera's fault beside a fit TI. A tune for both actuators on one under-steers (a third of
  the request at 21 m/s, simulated), and the lane keep is the driver's to switch on.
- **pdscomp's lane presentation on his donor-EPS CX-5** (0x440 rewritten to lines visible,
  `LANE_LINES` 0 not read as lane keep off). In the public logs four pre-2021 cameras never
  sent `LANE_LINES` 0 with the lane keep on, and the swapped EPS delivered while the 0x440
  shown to it said 1. A camera that does would show on the toggle-off drive.
- **Cutting a class 2 latch short with a half-second pause in the stream.** It clears them,
  and bypasses the unit's own recovery delay.

- **Reporting the EPS command, or the larger of the two.** Below the floor the EPS command is
  zero while the TI steers: the learner and the steer-limit classifier would see torque with
  no command behind it.
- **Commanding the EPS below its floor**, as both other forks do. It delivers nothing there,
  and the 2022 firmware raised `LKAS_FAULT` in a similar state. Zero costs no torque.
- **pdscomp's 12/25 panda backstop over a 6/15 controller.** A panda down rate above the
  controller's refuses every driver-override winddown.
- **Letting the panda admit a shrinking request after its permission falls**, to make the
  release a ramp on a brake as well. It is torque after the driver has braked, the one thing
  the panda refuses unconditionally.
- **Refusing 0x243 once the TI's torque reading is 100 ms old.** The controller keeps the EPS
  command until the steer fault at 0.3 s; the panda would cut holes in the EPS's stream.
- **Clearing the unfit count on the first fit frame.** Unfit 180 ms of every 220 never
  faulted and never alerted while the command averaged 3 counts of 450.
- **Clearing it after 0.5 s of fitness.** The command needs a second to climb back, so
  dropouts 0.5 s apart restarted it for ever (16 % of the clip delivered) without a word.
- **A fault for dropouts more than a second apart.** The command recovers between them; the
  take-control alarm for an interceptor that is steering most of the time is worse than the
  name on the screen.
- **The alert's second line from the frame of the moment.** Between dropouts that frame is a
  healthy one, and the line flipped several times a second.
- **The alert's sound on every frame it stands.** soundd plays a sound when it changes: a
  lane departure under the standing alert was silent, and the chime replayed whenever the
  screen came back.
- **Holding the steer fault through a stop once the TI is named**, to end the alarm at every
  pull-away under a latch. At a standstill upstream swaps the fault for its silent variant
  and keeps it for the rest of the fault: engaged with no steering and no alarm.
- **Reading the EPS's torque sensor the moment the TI goes quiet.** It is the TI's bias until
  the TI is in bypass.
- **A firmware version whitelist (0x01, 0x10).** A unit on any other firmware would fail the
  RX check and take cruise with it, with nothing to learn the byte from but a log.
- **Failing the RX quality flag on off or a violation.** A controls mismatch at every stop
  (pdscomp's first port).
- **Fingerprinting the TI.** It is silent until spoken to.
- **Hiding the toggle from the logs, or switching backends when it is on.** pdscomp's fork
  does both; no source was found for the claim behind it.
