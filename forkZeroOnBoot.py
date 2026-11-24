# -*- coding: utf-8 -*-
# @Time : 2026/09/17
# @File : forkZeroOnBoot.py
# @RBK Version: 3.4.x
# @Version: 1.0.0
# @Description:
#   Fork auto-zero on boot, running as a periodRun background daemon.
#
#   Requirement: after the robot boots (the script is loaded for the first time),
#   if the fork is not at its zero position, move it back to zero automatically.
#
#   "First boot" detection: __init__ sets self.first_boot = True. After zeroing
#   is done (or the fork is confirmed already at zero) it is set to False, so it
#   does not run again during the same power cycle. A script reload or a reboot
#   runs __init__ again, i.e. counts as a new boot.
#
#   Zero check (per axis, zero limit DI first, then position):
#     - zero limit DI configured (> 0) and triggered      -> at zero;
#     - DI not configured (-1) or not triggered, but |pos - zero| <= precision -> at zero;
#     - otherwise                                          -> needs zeroing.
#
#   Zeroing: periodRun only waits for motor feedback, decides whether zeroing is
#   needed, and submits one position Action via r.addMoveTask(). The RBK task
#   engine does the motion planning.
#
#   Safety:
#     1) Task gate: only zero when the dispatcher is not running a task
#        (idle / charging). If a task is running, skip this boot's zeroing.
#     2) If the robot starts moving or an axis fails during zeroing, stop
#        zeroing and raise an alarm.
#     3) Every axis has a timeout. On timeout: resetMotor + stopMotor + setUserError.
#     4) Axes with a zero limit DI: if the DI never changes for too many cycles,
#        treat the limit switch as faulty and warn.
#
#   One periodRun() call must take < 50 ms.

import json
import os
import sys
import time
import traceback

current_file_path = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_path)
sys.path.append(current_folder_path)
sys.path.append(current_folder_path + "/syspy")

from syspy.rbk import MoveStatus, BasicModule, ParamServer
from syspy.rbkSim import SimModule


"""
####BEGIN DEFAULT ARGS####
{}
####END DEFAULT ARGS####
"""


class Module(BasicModule):
    """Fork auto-zero on boot (periodRun background daemon)."""

    SCRIPT_VERSION = "V1.1.0"
    MOTOR_READY_HOLD = 2.0

    # common getMoveStatus()["task_status"] values; any status not listed does not block zeroing
    IDLE_TASK_STATUS = (0, 1, 6)
    ACTIVE_TASK_STATUS = (2, 3, 4, 5)
    CHARGING_TASK_STATUS = (7,)

    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        p = ParamServer(__file__)

        # ---------------- lift axis ----------------
        self.lift_motor = p.loadParam(
            "liftMotor", type="str", default="motor2", group="module-lift",
            comment="Fork lift motor name; None = no such axis")
        self.lift_zero = p.loadParam(
            "liftZero", type="float", default=0.086, minValue=0, maxValue=10,
            unit="m", group="module-lift", comment="Lift zero position")
        self.lift_zero_di = p.loadParam(
            "liftZeroDi", type="int", default=-1, minValue=-1, maxValue=100,
            group="module-lift", comment="Lift zero limit DI; -1 = not configured (position only)")
        self.lift_vel = p.loadParam(
            "liftVel", type="float", default=0.1, minValue=0, maxValue=0.1,
            unit="m/s", group="module-lift",
            comment="Lift zeroing max speed (must not exceed the motor maxSpeed in the robot model)")
        self.lift_precision = p.loadParam(
            "lift_zero_precision", type="float", default=0.005, minValue=0.0,
            maxValue=0.1, unit="m", group="module-lift",
            comment="Lift zero position tolerance (used when there is no zero limit DI)")

        # ---------------- stretch (reach) axis ----------------
        self.stretch_motor = p.loadParam(
            "stretchMotor", type="str", default="None", group="module-stretch",
            comment="Fork stretch motor name; None = no such axis")
        self.stretch_zero = p.loadParam(
            "stretchZero", type="float", default=0.02, minValue=0, maxValue=10,
            unit="m", group="module-stretch", comment="Stretch zero position (retracted)")
        self.stretch_zero_di = p.loadParam(
            "stretchZeroDi", type="int", default=-1, minValue=-1, maxValue=100,
            group="module-stretch", comment="Stretch zero limit DI; -1 = not configured (position only)")
        self.stretch_vel = p.loadParam(
            "stretchVel", type="float", default=0.02, minValue=0, maxValue=1.5,
            unit="m/s", group="module-stretch", comment="Stretch zeroing max speed")
        self.stretch_precision = p.loadParam(
            "stretch_zero_precision", type="float", default=0.005, minValue=0.0,
            maxValue=0.1, unit="m", group="module-stretch",
            comment="Stretch zero position tolerance (used when there is no zero limit DI)")

        # ---------------- lateral (side-shift) axis ----------------
        self.lateral_motor = p.loadParam(
            "lateral_motor", type="str", default="None", group="module-lateral",
            comment="Fork side-shift motor name; None = no such axis")
        self.lateral_zero = p.loadParam(
            "lateral_zero", type="float", default=0.0, minValue=-10, maxValue=10,
            unit="m", group="module-lateral", comment="Side-shift zero position")
        self.lateral_zero_di = p.loadParam(
            "lateral_zero_di", type="int", default=-1, minValue=-1, maxValue=100,
            group="module-lateral", comment="Side-shift zero limit DI; -1 = not configured (position only)")
        self.lateral_vel = p.loadParam(
            "lateral_vel", type="float", default=0.0, minValue=0, maxValue=10,
            unit="m/s", group="module-lateral", comment="Side-shift zeroing max speed")
        self.lateral_precision = p.loadParam(
            "lateral_zero_precision", type="float", default=0.005, minValue=0.0,
            maxValue=0.1, unit="m", group="module-lateral",
            comment="Side-shift zero position tolerance (used when there is no zero limit DI)")

        # ---------------- other linear axes (JSON string) ----------------
        # example:
        # [{"name": "expand1", "zero": 0.1375, "zeroDi": -1, "vel": 0.4, "precision": 0.005}]
        self.extra_motors = p.loadParam(
            "extraMotors", type="str", default="[]", group="module-extra",
            comment="Other linear motor axes to zero (JSON string), "
                    "fields: name/zero/zeroDi/vel/precision/timeout")

        # ---------------- task & safety ----------------
        self.check_task_status = p.loadParam(
            "checkTaskStatus", type="bool", default=True, group="safety",
            comment="Skip zeroing on this boot if the dispatcher is running a task (recommended)")
        self.skip_when_charging = p.loadParam(
            "skipWhenCharging", type="bool", default=True, group="safety",
            comment="Treat charging as idle and allow zeroing on boot")
        self.block_when_moving = p.loadParam(
            "blockWhenMoving", type="bool", default=True, group="safety",
            comment="Stop zeroing and raise an alarm if the robot starts moving during zeroing")
        self.move_speed_threshold = p.loadParam(
            "moveSpeedThreshold", type="float", default=0.01, minValue=0.0,
            maxValue=1.0, unit="m/s", group="safety",
            comment="Travel speed threshold for 'robot is moving'")
        self.zero_timeout = p.loadParam(
            "zeroTimeout", type="float", default=20.0, minValue=0.1,
            maxValue=600.0, unit="s", group="safety",
            comment="Zeroing timeout per axis")
        self.retry_interval = p.loadParam(
            "retryInterval", type="float", default=0.3, minValue=0.0,
            maxValue=10.0, unit="s", group="safety",
            comment="Retry interval after timeout/failure; 0 = no retry")
        self.limit_di_allow_count = p.loadParam(
            "limitDiAllowCount", type="int", default=3, minValue=0, maxValue=100,
            group="safety",
            comment="Cycles the zero limit DI may stay inactive before it is treated as faulty")
        self.log_interval = p.loadParam(
            "logInterval", type="float", default=1.0, minValue=0.0, maxValue=10.0,
            unit="s", group="debug", comment="Log/status report interval; 0 = every cycle")
        self.first_boot_log_once = p.loadParam(
            "firstBootLogOnce", type="bool", default=True, group="debug",
            comment="Report boot info only once (avoid log spam)")

        # ---------------- runtime state ----------------
        self.first_boot = True          # first-boot flag
        self.axis_list = []
        self.zero_axis_list = []
        self.zero_start_time = None
        self.zero_task_sent = False
        self.zero_task_payload = None
        self.motor_ready_since = None
        self.retry_time = None
        self.last_log_time = 0.0
        self.limit_di_off_count = {}
        self.last_axis_state = {}
        self.state = {}
        self.last_error = ""
        self._summary_sent = False
        self._summary_result = ""

        r.logInfo("%s %s loaded, file=%s" % (self.name(), self.SCRIPT_VERSION, os.path.basename(__file__)))

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def name(self):
        return "forkZeroOnBoot"

    def _read_di_map(self, r: SimModule):
        """Read DI once per cycle (each Di() is an IPC round trip)."""
        di_map = {}
        try:
            for node in r.Di().get("node", list()):
                di_map[node.get("id")] = bool(node.get("status", False))
        except Exception:
            return di_map
        return di_map

    def _read_motor_map(self, r: SimModule):
        """Read odo() once per cycle; returns {motor name: {position, speed}}."""
        motor_map = {}
        try:
            for info in r.odo().get("motor_info", list()):
                name = info.get("motor_name")
                if name is None:
                    continue
                motor_map[name] = {
                    "position": self._safe_float(info.get("position"), None),
                    "speed": self._safe_float(info.get("speed"), 0.0),
                }
        except Exception:
            return motor_map
        return motor_map

    @staticmethod
    def _safe_float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _norm_motor_name(name):
        if name is None:
            return "None"
        name = str(name).strip()
        if name == "" or name.lower() == "none":
            return "None"
        return name

    def _task_status(self, r: SimModule):
        """Read the current task status; None if unavailable (treated as idle, zeroing allowed)."""
        try:
            status = r.getMoveStatus()
        except Exception:
            return None
        if not isinstance(status, dict):
            return None
        value = status.get("task_status", None)
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _task_is_active(self, r: SimModule):
        """Whether the dispatcher is running a task. False if unknown (does not block zeroing)."""
        if not self.check_task_status:
            return False
        status = self._task_status(r)
        if status is None:
            return False
        if status in self.ACTIVE_TASK_STATUS:
            return True
        if self.skip_when_charging and status in self.CHARGING_TASK_STATUS:
            return False
        return False

    def _chassis_moving(self, r: SimModule):
        """Whether the robot is moving: use the planned speed from getNextSpeed first,
        fall back to odo vel_x/vel_y/vel_rotate."""
        speed = None
        try:
            value = r.getNextSpeed()
            if isinstance(value, dict) and value:
                speed = value
        except Exception:
            speed = None
        if speed is None:
            try:
                value = r.odo()
                if isinstance(value, dict):
                    speed = value
            except Exception:
                speed = None
        if not isinstance(speed, dict):
            return False
        for key in ("x", "y", "vel_x", "vel_y", "rotate", "vel_rotate"):
            value = self._safe_float(speed.get(key, 0.0), 0.0)
            if abs(value) > self.move_speed_threshold:
                return True
        return False

    def _build_axis_list(self):
        """On first run, parse params and build the axis list (one-time work)."""
        axes = []
        extra_error = ""

        def add_axis(motor, zero_pos, zero_di, vel, precision, timeout):
            motor = self._norm_motor_name(motor)
            if motor == "None":
                return False
            if vel <= 0.0:
                return False
            axes.append(ZeroAxis(motor, float(zero_pos), int(zero_di), False,
                                 float(vel), 0.0, float(precision), float(timeout)))
            return True

        add_axis(self.lift_motor, self.lift_zero, self.lift_zero_di,
                 self.lift_vel, self.lift_precision, self.zero_timeout)
        add_axis(self.stretch_motor, self.stretch_zero, self.stretch_zero_di,
                 self.stretch_vel, self.stretch_precision, self.zero_timeout)
        add_axis(self.lateral_motor, self.lateral_zero, self.lateral_zero_di,
                 self.lateral_vel, self.lateral_precision, self.zero_timeout)

        try:
            extra = json.loads(self.extra_motors) if self.extra_motors else []
        except Exception as e:
            extra = []
            extra_error = "extraMotors is not valid JSON: %s" % e
        if isinstance(extra, list):
            for item in extra:
                if not isinstance(item, dict):
                    continue
                add_axis(item.get("name"),
                         item.get("zero", 0.0),
                         item.get("zeroDi", -1),
                         item.get("vel", 0.0),
                         item.get("precision", 0.005),
                         item.get("timeout", self.zero_timeout))

        # keep one entry per motor name (lift may also be listed in extraMotors)
        unique = {}
        for axis in axes:
            unique[axis.name] = axis
        self.axis_list = list(unique.values())

        # motor name set but speed <= 0: this axis can never move, so warn the site
        unconfigured = []
        for motor in (self.lift_motor, self.stretch_motor, self.lateral_motor):
            motor = self._norm_motor_name(motor)
            if motor == "None":
                continue
            if not any(axis.name == motor for axis in self.axis_list):
                unconfigured.append(motor)
        return extra_error, unconfigured

    # ------------------------------------------------------------------ #
    # zeroing flow
    # ------------------------------------------------------------------ #
    def _scan_axes(self, r: SimModule):
        """Read DI and motor positions once, check whether each axis is at zero."""
        di_map = self._read_di_map(r)
        motor_map = self._read_motor_map(r)
        for axis in self.axis_list:
            axis.di_active = bool(di_map.get(axis.zero_di, False)) if axis.zero_di > 0 else False
            info = motor_map.get(axis.name, None)
            if isinstance(info, dict) and info.get("position") is not None:
                axis.position = info["position"]
        return motor_map

    def _check_axes_need_zero(self, r: SimModule):
        need_zero = []
        for axis in self.axis_list:
            axis.need_zero, reason = axis.check_need_zero()
            if axis.need_zero:
                need_zero.append(axis)
        return need_zero

    def _start_zero(self, r: SimModule, now):
        if self.last_error:
            if r.errorExits(53945):
                r.clearError(53945)
            self.last_error = ""
        self.zero_axis_list = self._check_axes_need_zero(r)
        self.zero_start_time = now
        self.zero_task_sent = False
        self.zero_task_payload = None
        self.limit_di_off_count = {}
        self._summary_sent = False
        for axis in self.zero_axis_list:
            axis.start(now)
        r.setNotice("%s boot: fork not at zero, submitting zeroing Action: %s" % (
            self.name(), ", ".join("%s(pos=%.4f)" % (a.name, a.position) for a in self.zero_axis_list)))
        r.logInfo("%s start zero: %s" % (self.name(), json.dumps(
            [a.state() for a in self.zero_axis_list], ensure_ascii=False)))

    def _feedback_ready(self, motor_map, now):
        """Only submit zeroing after all axes have had real feedback for a while."""
        ready = bool(self.axis_list)
        for axis in self.axis_list:
            info = motor_map.get(axis.name)
            if not isinstance(info, dict) or info.get("position") is None:
                ready = False
                break
        if not ready:
            self.motor_ready_since = None
            return False
        if self.motor_ready_since is None:
            self.motor_ready_since = now
            return False
        return (now - self.motor_ready_since) >= self.MOTOR_READY_HOLD

    def _submit_zero_task(self, r: SimModule, now):
        """Submit once the same 3051 task as a manual RoboShop position Action."""
        actions = []
        for axis in self.zero_axis_list:
            actions.append({
                "maxSpeed": float(axis.vel),
                "mode": "position",
                "name": axis.name,
                "operationTime": float(axis.timeout),
                "position": float(axis.zero_pos),
                "stopDI": int(axis.zero_di if axis.zero_di > 0 else -1),
            })
        payload = {
            "body": {
                "actions": actions,
                "parallel": len(actions) > 1,
                "version": 1,
            }
        }
        try:
            result = r.addMoveTask(json.dumps(payload))
        except Exception as e:
            self._fail_zero(r, "failed to submit zeroing Action: %s" % e)
            return False
        self.zero_task_sent = True
        self.zero_task_payload = payload
        self.zero_start_time = now
        for axis in self.zero_axis_list:
            axis.start(now)
        r.logInfo("%s zero Action submitted: ret=%s payload=%s" % (
            self.name(), result, json.dumps(payload, ensure_ascii=False)))
        return True

    def _run_zero(self, r: SimModule, now):
        """Submit the zeroing Action once, then only monitor feedback."""
        if self.block_when_moving and self._chassis_moving(r):
            self._fail_zero(r, "robot started moving during zeroing, zeroing stopped")
            return "failed"

        if not self.zero_task_sent:
            if not self._submit_zero_task(r, now):
                return "failed"
            return "running"

        all_reached = True
        for axis in self.zero_axis_list:
            reached = axis.in_zero()
            axis.reached = bool(reached)
            if not reached:
                all_reached = False
                self._check_limit_di(r, axis)
            if axis.timed_out(now):
                self._fail_zero(
                    r,
                    "axis %s not zeroed within %.1fs, position %.4f, zero %.4f, zero limit DI %s" % (
                        axis.name, axis.timeout, axis.position, axis.zero_pos,
                        axis.zero_di if axis.zero_di > 0 else "not configured"))
                return "failed"

        if all_reached:
            self.retry_time = None
            self.first_boot = False         # zeroing done, not repeated on this boot
            self.zero_axis_list = []
            self.zero_start_time = None
            self.zero_task_sent = False
            self.zero_task_payload = None
            self.last_error = ""
            if r.errorExits(53945):
                r.clearError(53945)
            if not self._summary_sent:
                self._summary_sent = True
                self._summary_result = "zero done"
                r.setNotice("%s boot: fork zeroing complete" % self.name())
            return "finished"
        return "running"

    def _check_limit_di(self, r: SimModule, axis):
        """Warn when the zero limit DI never changes (so a faulty limit switch is not silently ignored)."""
        if axis.zero_di <= 0:
            return
        count = self.limit_di_off_count.get(axis.name, 0) + 1
        self.limit_di_off_count[axis.name] = count
        if self.limit_di_allow_count > 0 and count == self.limit_di_allow_count + 1:
            r.setUserWarning(
                55900,
                "%s zeroing: axis %s zero limit DI%d not triggered for %d cycles, "
                "check the limit switch/wiring; using position instead (position %.4f, zero %.4f)" % (
                    self.name(), axis.name, axis.zero_di, count, axis.position, axis.zero_pos))
            r.logWarn("%s limit DI didn't trigger: axis=%s di=%d pos=%.4f" % (
                self.name(), axis.name, axis.zero_di, axis.position))

    def _throttle(self, now):
        """Record when zeroing may be tried again (used for failure retry and busy re-check).

        retry_interval=0 means no retry (give up for this boot).
        """
        if self.retry_interval > 0.0:
            self.retry_time = now + self.retry_interval
        else:
            self.retry_time = None

    def _fail_zero(self, r: SimModule, message):
        """Zeroing failed: stop motors and alarm; retries after retry_interval."""
        if self.zero_task_sent:
            try:
                r.stopCurrentBlock()
            except Exception:
                pass
        for axis in self.zero_axis_list:
            r.resetMotor(axis.name)
        r.stopMotor()
        r.setUserError(53945, "%s %s" % (self.name(), message))
        r.logError("%s %s" % (self.name(), message))
        self.last_error = message
        self.zero_axis_list = []
        self.zero_start_time = None
        self.zero_task_sent = False
        self.zero_task_payload = None
        self._throttle(time.time())
        self._summary_sent = False

    def _build_state(self, need_zero_names, phase):
        state = {
            "firstBoot": bool(self.first_boot),
            "phase": phase,
            "needZero": need_zero_names,
            "error": self.last_error,
        }
        axes = []
        for axis in self.axis_list:
            axes.append(axis.state())
        state["axes"] = axes
        if self.zero_axis_list and self.zero_start_time is not None:
            state["zeroElapsed"] = round(time.time() - self.zero_start_time, 2)
            state["zeroTaskSent"] = bool(self.zero_task_sent)
        return state

    def _report(self, r: SimModule, state, force=False):
        now = time.time()
        if not force and self.log_interval > 0.0 and (now - self.last_log_time) < self.log_interval:
            return
        self.last_log_time = now
        text = json.dumps(state, ensure_ascii=False)
        r.setInfo(text)
        r.logDebug("[%s]%s" % (self.name(), text))

    # ------------------------------------------------------------------ #
    # periodRun
    # ------------------------------------------------------------------ #
    def periodRun(self, r: SimModule) -> bool:
        try:
            now = time.time()
            if not self.axis_list and not self._summary_sent:
                # first run: parse params and build axis list
                extra_error, unconfigured = self._build_axis_list()
                if extra_error:
                    r.setUserError(53946, "%s %s" % (self.name(), extra_error))
                    self.last_error = extra_error
                if unconfigured:
                    r.setUserWarning(
                        55901,
                        "%s these motors have a name but no zeroing speed (vel>0), cannot zero: %s" % (
                            self.name(), ", ".join(unconfigured)))

            if not self.axis_list:
                if not self._summary_sent:
                    self._summary_sent = True
                    self._summary_result = "no axis"
                    r.setNotice("%s no fork linear motor configured for zeroing, script idle" % self.name())
                self._report(r, {"firstBoot": bool(self.first_boot), "phase": "disabled",
                                 "error": self.last_error})
                return True

            motor_map = self._scan_axes(r)

            # ---------------- first boot: check and zero ----------------
            if self.first_boot:
                if self.zero_start_time is not None:
                    # phase 2: zeroing in progress
                    result = self._run_zero(r, now)
                    self._report(r, self._build_state(
                        [a.name for a in self.zero_axis_list], "zeroing"),
                        force=(result == "failed"))
                    return True

                # phase 1: zeroing not started
                if not self._feedback_ready(motor_map, now):
                    self._report(r, self._build_state([], "wait-motor-ready"))
                    return True

                need_zero = self._check_axes_need_zero(r)
                if not need_zero:
                    # already at zero (or zeroing done): finish for this boot
                    self.first_boot = False
                    self.retry_time = None
                    self.limit_di_off_count = {}
                    if self.last_error:
                        r.clearError(53945)
                        self.last_error = ""
                    if r.warningExits(55900):
                        r.clearWarning(55900)
                    if not self._summary_sent:
                        self._summary_sent = True
                        self._summary_result = "already in zero"
                        r.setNotice("%s boot: fork already at zero, no zeroing needed" % self.name())
                    self._report(r, self._build_state([], "done"), force=True)
                    return True

                if self.retry_time is not None and now < self.retry_time:
                    # still waiting (retry after failure, or re-check after a busy task);
                    # send no motor command this cycle, only report per log_interval.
                    phase = "retry-wait" if self.last_error else "wait-retry"
                    self._report(r, self._build_state(
                        [a.name for a in need_zero], phase))
                    return True

                if self._task_is_active(r):
                    # dispatcher is running a task: yield, do not interrupt a moving robot;
                    # check again after retry_interval; retry_interval=0 gives up for this boot.
                    self._throttle(now)
                    if self.retry_interval <= 0.0:
                        self.first_boot = False
                    if not self._summary_sent:
                        self._summary_sent = True
                        self._summary_result = "task busy"
                        r.setNotice("%s boot: dispatcher is running a task, skipping fork zeroing on this boot" % self.name())
                    self._report(r, self._build_state(
                        [a.name for a in need_zero], "wait-task"),
                        force=self.first_boot_log_once)
                    return True

                self._start_zero(r, now)
                return True

            # ---------------- after boot: light monitoring only ----------------
            self._report(r, self._build_state([], "monitor"))
            return True

        except Exception as e:
            # daemon must not die: report the exception (with line number), then return False
            r.setNotice("%s periodRun exception: %s\n%s" % (
                self.name(), e, traceback.format_exc()))
            return False


class ZeroAxis:
    """A linear motor axis to be zeroed."""

    def __init__(self, name, zero_pos, zero_di, di_active, vel, position,
                 precision, timeout):
        self.name = name
        self.zero_pos = zero_pos
        self.zero_di = zero_di
        self.di_active = di_active
        self.vel = vel
        self.position = position
        self.precision = precision
        self.timeout = timeout
        self.need_zero = False
        self.start_time = None
        self.reached = False

    def position_at_zero(self):
        return abs(self.position - self.zero_pos) <= self.precision

    def in_zero(self):
        """Zero check: zero limit DI first, then position."""
        if self.zero_di > 0 and self.di_active:
            return True
        return self.position_at_zero()

    def check_need_zero(self):
        """Whether zeroing is needed. Returns (need_zero, reason)."""
        if self.zero_di > 0 and self.di_active:
            return False, "limit-%d on" % self.zero_di
        if self.position_at_zero():
            return False, "pos %.4f" % self.position
        if self.zero_di > 0:
            return True, "limit-%d off, pos %.4f" % (self.zero_di, self.position)
        return True, "pos %.4f" % self.position

    def start(self, now):
        self.need_zero = True
        self.start_time = now
        self.reached = False

    def timed_out(self, now):
        return self.start_time is not None and (now - self.start_time) > self.timeout

    def state(self):
        return {
            "name": self.name,
            "needZero": self.need_zero,
            "zero": self.in_zero(),
            "pos": round(self.position, 4),
            "zeroPos": self.zero_pos,
            "zeroDi": self.zero_di,
            "zeroDiOn": bool(self.di_active),
            "reached": bool(self.reached),
        }


if __name__ == "__main__":
    from syspy.rbkSim import SimModule

    r1 = SimModule()
    args = {}
    m = Module(r1, args)        # empty construction: writes params/<script name>.json
    r1.logInfo("%s" % m)
