# -*- coding: utf-8 -*-
# @File : periodRunAll.py
# @RBK Version: 3.4.x
# @Version: 1.0.0
# @Description:
#   periodRun only accepts ONE script. This entry script loads several
#   periodRun scripts from the same folder and calls each one's periodRun()
#   every cycle, so they can all run at the same time.
#
#   Default: forkZeroOnBoot.py + check_back_lidar.py
#
#   - The original scripts are NOT modified. Each one keeps reading its own
#     params file (params/<script name>.json), exactly as before.
#   - If one script fails to load or raises an exception, the others keep
#     running. Failures are logged and shown as a warning (55902).
#   - Put this file and the scripts it loads in the same folder
#     (scripts/module/userpy/), and set periodRun scriptName to
#     "userpy/periodRunAll.py".

import importlib.util
import os
import sys
import time
import traceback

from syspy.rbk import BasicModule, MoveStatus
from syspy.rbkSim import SimModule

"""
####BEGIN DEFAULT ARGS####
{}
####END DEFAULT ARGS####
"""

# Scripts to run, in this order, every cycle (file names in this folder).
SCRIPTS = [
    "forkZeroOnBoot.py",
    "check_back_lidar.py",
]

WARN_CODE = 55902          # user warning: a sub-script failed to load / raised
SLOW_CYCLE_SEC = 0.05      # log a warning if one full cycle takes longer than this
LOG_THROTTLE_SEC = 10.0    # do not repeat the same exception log more often than this

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)


def _load_module_class(file_name):
    path = os.path.join(HERE, file_name)
    mod_name = "_periodRunAll_" + os.path.splitext(file_name)[0]
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod.Module


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        self.status = MoveStatus.NONE
        self.subs = []            # list of (name, instance)
        self.last_err_log = {}    # name -> time of last exception log
        self.last_slow_log = 0.0
        load_errors = []

        for file_name in SCRIPTS:
            try:
                cls = _load_module_class(file_name)
                inst = cls(r, args)
                self.subs.append((file_name, inst))
                r.logInfo("periodRunAll: loaded %s" % file_name)
            except Exception:
                msg = "periodRunAll: failed to load %s\n%s" % (file_name, traceback.format_exc())
                r.logError(msg)
                load_errors.append(file_name)

        if load_errors:
            r.setUserWarning(WARN_CODE, "periodRunAll: failed to load " + ", ".join(load_errors))
        r.logInfo("periodRunAll: running %s" % ", ".join(n for n, _ in self.subs))

    def periodRun(self, r: SimModule) -> bool:
        t0 = time.time()
        for name, inst in self.subs:
            try:
                inst.periodRun(r)
            except Exception:
                now = time.time()
                if now - self.last_err_log.get(name, 0.0) > LOG_THROTTLE_SEC:
                    self.last_err_log[name] = now
                    r.logError("periodRunAll: %s periodRun exception\n%s" % (name, traceback.format_exc()))
                    r.setUserWarning(WARN_CODE, "periodRunAll: %s periodRun exception" % name)
        dt = time.time() - t0
        if dt > SLOW_CYCLE_SEC and time.time() - self.last_slow_log > LOG_THROTTLE_SEC:
            self.last_slow_log = time.time()
            r.logWarn("periodRunAll: cycle took %.1f ms" % (dt * 1000.0))
        return True

    def run(self, r: SimModule, args):
        self.status = MoveStatus.FINISHED
        return self.status

    def suspend(self, r: SimModule):
        self.status = MoveStatus.SUSPENDED
        return self.status

    def cancel(self, r: SimModule):
        self.status = MoveStatus.NONE
        return self.status


if __name__ == "__main__":
    r1 = SimModule()
    Module(r1, {})
