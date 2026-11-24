# -*- coding: utf-8 -*-

import json
import os
import sys


current_file_path = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_path)
sys.path.append(current_folder_path)
sys.path.append(current_folder_path + "/syspy")

from syspy.rbk import BasicModule, MoveStatus
from syspy.rbkSim import SimModule


ERROR_CODE = 53920
CHECK_EVERY_CYCLES = 100
REQUIRED_DEVICES = {
    "camera": {
        0: "TOP-360",
        4: "Percipio",
        5: "OBS-360",
    },
    "laser": {
        1: "Left",
        2: "Right",
        3: "Back",
    },
}


def _get_param_value(device, target_key):
    for device_param in device.get("deviceParams", []):
        array_param = device_param.get("arrayParam", {})
        for param in array_param.get("params", []):
            if param.get("key") != target_key:
                continue

            for value_key in (
                "uint32Value",
                "int32Value",
                "stringValue",
                "boolValue",
                "doubleValue",
            ):
                if value_key in param:
                    return param[value_key]
    return None


def _inspect_sensors(robot_file):
    result = {
        "model": None,
        "robotFileType": type(robot_file).__name__,
        "deviceTypesFound": False,
        "requiredDevices": {
            device_type: device_names
            for device_type, device_names in REQUIRED_DEVICES.items()
        },
        "deviceTypeFound": {
            device_type: False for device_type in REQUIRED_DEVICES
        },
        "devices": [],
        "missingDevices": [],
        "disabledDevices": [],
        "invalidDevices": [],
        "allEnabled": False,
        "reason": "",
    }

    if not isinstance(robot_file, dict):
        result["reason"] = "getRobotFile result is not dict"
        return result

    result["model"] = robot_file.get("model")
    device_types = robot_file.get("deviceTypes")
    if not isinstance(device_types, list):
        result["reason"] = "deviceTypes is not list"
        return result

    result["deviceTypesFound"] = True

    for device_type_data in device_types:
        if not isinstance(device_type_data, dict):
            continue

        device_type = str(
            device_type_data.get("name", "")
        ).strip().lower()
        if device_type not in REQUIRED_DEVICES:
            continue

        result["deviceTypeFound"][device_type] = True
        devices = device_type_data.get("devices", [])
        if not isinstance(devices, list):
            result["reason"] = device_type + " devices is not list"
            return result

        for device in devices:
            if not isinstance(device, dict):
                continue

            name = str(device.get("name", "")).strip()
            device_id = _get_param_value(device, "id")
            enabled = device.get("isEnabled")
            result["devices"].append({
                "type": device_type,
                "name": name,
                "id": device_id,
                "isEnabled": enabled,
            })

    device_by_key = {}
    for device in result["devices"]:
        device_by_key[(device["type"], device["id"])] = device

    for device_type, device_names in REQUIRED_DEVICES.items():
        for device_id, expected_name in device_names.items():
            device_key = {
                "type": device_type,
                "id": device_id,
                "name": expected_name,
            }
            device = device_by_key.get((device_type, device_id))
            if device is None:
                result["missingDevices"].append(device_key)
                continue

            enabled = device["isEnabled"]
            device_key["name"] = device["name"] or expected_name
            if enabled is False:
                result["disabledDevices"].append(device_key)
            elif enabled is not True:
                result["invalidDevices"].append(device_key)

    result["allEnabled"] = (
        not result["missingDevices"]
        and not result["disabledDevices"]
        and not result["invalidDevices"]
    )

    if result["allEnabled"]:
        result["reason"] = "All 3 cameras and 3 lidars are enabled"
    else:
        reasons = []
        if result["missingDevices"]:
            reasons.append(
                "missing " + str(result["missingDevices"])
            )
        if result["disabledDevices"]:
            reasons.append(
                "disabled " + str(result["disabledDevices"])
            )
        if result["invalidDevices"]:
            reasons.append(
                "invalid isEnabled " + str(result["invalidDevices"])
            )
        result["reason"] = "; ".join(reasons)
    return result


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        self.status = MoveStatus.NONE
        self.cycle_count = CHECK_EVERY_CYCLES - 1
        self.check_count = 0

    def periodRun(self, r: SimModule) -> bool:
        self.cycle_count += 1
        if self.cycle_count < CHECK_EVERY_CYCLES:
            return True

        self.cycle_count = 0
        self.check_count += 1

        try:
            result = _inspect_sensors(r.getRobotFile())
            lidar_ok = result["allEnabled"] is True
            result["exception"] = ""
        except Exception as error:
            lidar_ok = False
            result = {
                "model": None,
                "robotFileType": None,
                "deviceTypesFound": False,
                "requiredDevices": {
                    device_type: device_names
                    for device_type, device_names in REQUIRED_DEVICES.items()
                },
                "deviceTypeFound": {
                    device_type: False for device_type in REQUIRED_DEVICES
                },
                "devices": [],
                "missingDevices": [
                    {
                        "type": device_type,
                        "id": device_id,
                        "name": device_name,
                    }
                    for device_type, device_names in REQUIRED_DEVICES.items()
                    for device_id, device_name in device_names.items()
                ],
                "disabledDevices": [],
                "invalidDevices": [],
                "allEnabled": False,
                "reason": "getRobotFile exception",
                "exception": str(error),
            }

        try:
            error_exists = bool(r.errorExits(ERROR_CODE))
            error_query_exception = ""
        except Exception as error:
            error_exists = None
            error_query_exception = str(error)

        debug_info = {
            "monitor": "camera_lidar_model",
            "cycleCount": self.cycle_count,
            "checkCount": self.check_count,
            "checkEveryCycles": CHECK_EVERY_CYCLES,
            "lidarOk": lidar_ok,
            "errorCode": ERROR_CODE,
            "errorExistsBeforeCheck": error_exists,
            "errorQueryException": error_query_exception,
            "result": result,
        }
        debug_text = json.dumps(debug_info)
        r.setInfo(debug_text)
        r.logInfo(debug_text)

        if lidar_ok:
            if error_exists is True:
                r.clearError(ERROR_CODE)
                r.logInfo("All 3 cameras and 3 lidars are enabled.")
            return True

        # If the error was manually cleared while any required sensor remains
        # disabled or missing, report it again during this check.
        if error_exists is not True:
            issue_messages = []
            for device in result.get("missingDevices", []):
                issue_messages.append(
                    "{} {} (ID {}) missing".format(
                        device["type"].capitalize(),
                        device["name"],
                        device["id"],
                    )
                )
            for device in result.get("disabledDevices", []):
                issue_messages.append(
                    "{} {} (ID {}) disabled".format(
                        device["type"].capitalize(),
                        device["name"],
                        device["id"],
                    )
                )
            for device in result.get("invalidDevices", []):
                issue_messages.append(
                    "{} {} (ID {}) has invalid enable state".format(
                        device["type"].capitalize(),
                        device["name"],
                        device["id"],
                    )
                )

            if issue_messages:
                message = "Sensor check failed: " + "; ".join(issue_messages)
            else:
                message = "Sensor check failed: " + result["reason"]
            if result.get("exception"):
                message += ": " + result["exception"]
            r.setUserError(ERROR_CODE, message)
            r.logError(message)

        return True

    def run(self, r: SimModule, args):
        r.setInfo(
            "Sensor model monitor checks 3 cameras and 3 lidars through "
            "periodRun and getRobotFile every 100 control cycles."
        )
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
    args = {}
    Module(r1, args)
