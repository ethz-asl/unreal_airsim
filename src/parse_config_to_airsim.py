#!/usr/bin/env python

import rospy
import sys
import os
import math
import numpy as np
from scipy.spatial.transform import Rotation
import json
import yaml
from datetime import datetime


class ConfigParser:
    def __init__(self):
        """  Initialize ros node and read params """
        # Params
        default_path = os.path.join(os.path.expanduser("~"), "Documents", "AirSim", "settings.json")
        print(default_path)
        self.target_file_path = rospy.get_param('~target_file_path', "")  # Where to write the target config
        if self.target_file_path is "":
            self.target_file_path = os.path.join(os.path.expanduser("~"), "Documents", "AirSim", "settings.json")
        self.config_file_path = rospy.get_param('~config_file_path', "no_source_path_given")  # Where the source is

        # Initial / minimal config
        self.new_cfg = {"ConfigParserInfo": "This config was auto-created from source '%s' on %s." %
                                            (self.config_file_path, datetime.now().strftime("%d.%m.%Y, %H:%M:%S")),
                        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
                        "SettingsVersion": 1.2,
                        "SimMode": "Multirotor"}
        self.yaml_cfg = None
        self.log_counter = [0, 0, 0]

        # Settings not to parse
        self.np_general = ["SeeDocsAt", "SettingsVersion", "SimMode", "ConfigParserInfo"]
        self.np_camera = ["T_B_S", "X", "Y", "Z", "Pitch", "Roll", "Yaw", "CaptureSettings"]
        self.image_type_map = {"Scene": 0, "DepthPlanner": 1, "DepthPerspective": 2, "DepthVis": 3,
                               "DisparityNormalized": 4, "Segmentation": 5, "SurfaceNormals": 6, "Infrared": 7}
        self.np_lidar = ["T_B_S", "SensorType", "Enabled", "DataFrame"]
        self.np_imu = ["T_B_S", "SensorType", "Enabled"]

    def parse(self):
        """ Execute the settings conversion """
        info = "* Starting config parser for unreal_airsim to AirSim config conversion *"
        self.log("\n" + "*" * len(info) + "\n" + info + "\n" + "*" * len(info))

        self.read_source_config()
        self.setup_target_file()
        self.forward_args(self.yaml_cfg, self.new_cfg, self.np_general)
        sensor_dict, camera_dict = self.parse_sensors()
        vehicle, vehicle_name = self.parse_vehicle()
        vehicle["Cameras"] = camera_dict
        vehicle["Sensors"] = sensor_dict
        self.new_cfg["Vehicles"] = {vehicle_name: vehicle}
        self.write_target_file()
        info = "* Settings parsing finished successfully (%i Warnings, %i Errors)! *" % (
            self.log_counter[1], self.log_counter[2])
        self.log("\n" + "*" * len(info) + "\n" + info + "\n" + "*" * len(info))

    def write_target_file(self):
        j = json.dumps(self.new_cfg, indent=2)
        f = open(self.target_file_path, 'w')
        print >> f, j
        f.close()
        self.log("Wrote config to target file '%s'." % self.target_file_path)

    def parse_vehicle(self):
        """ Read settings for a (the) vehicle """
        # TODO(schmluk): maybe pass on other vehicle settings as well here
        vehicle = {"VehicleType": "SimpleFlight"}
        vehicle_name = rospy.get_param("vehicle_name", "airsim_drone")  # default
        self.log("Using vehicle name '%s'." % vehicle_name)
        return vehicle, vehicle_name

    def parse_sensors(self):
        """ Identify all listed sensors and convert to AirSim settings """
        sensors = self.yaml_cfg["sensors"]
        sensor_dict = {}
        camera_dict = {}
        for name in sensors:
            data = sensors[name]
            out = {}
            if not "sensor_type" in data:
                self.log("Skipping sensor '%s', no 'sensor_type' given!" % name, 2)
                continue
            type = data["sensor_type"]
            if type == "Camera":
                self.log("Found camera '%s'." % name)
                image_type = "Scene"  # default
                if "image_type" in data:
                    image_type = data["image_type"]
                if image_type not in self.image_type_map:
                    log("Unknown image_type '%s', defaulting to 'Scene'!\n(Known types are %s)" % (
                        image_type, ', '.join("%s" % self.image_type_map[key] for key in self.image_type_map)), 1)
                    image_type = "Scene"
                capture_settings = {"ImageType": self.image_type_map[image_type]}
                if "CaptureSettings" in data:
                    capt_in = data["CaptureSettings"]
                    self.forward_args(capt_in, capture_settings, ["ImageType"])
                out["CaptureSettings"] = [capture_settings]
                self.forward_args(data, out, self.np_camera)
                T = None
                if "T_B_S" in data:
                    T = data["T_B_S"]
                self.parse_transformation(T, out)
                camera_dict[name] = out
            elif type == "Lidar":
                self.log("Found lidar '%s'." % name)
                out = {"SensorType": 6, "Enabled": True, "DataFrame": "SensorLocalFrame"}
                self.forward_args(data, out, self.np_lidar)
                T = None
                if "T_B_S" in data:
                    T = data["T_B_S"]
                self.parse_transformation(T, out)
                sensor_dict[name] = out
            elif type == "Imu":
                self.log("Found imu '%s'." % name)
                out = {"SensorType": 2, "Enabled": True}
                self.forward_args(data, out, self.np_imu)
                T = None
                if "T_B_S" in data:
                    T = data["T_B_S"]
                self.parse_transformation(T, out)
                sensor_dict[name] = out
        return sensor_dict, camera_dict

    def read_source_config(self):
        """ Check that source file exists and read yml dict """
        if not os.path.isfile(self.config_file_path):
            self.log("Source file '%s' could not be found!" % self.config_file_path, 2)
            sys.exit()
        try:
            with open(self.config_file_path) as yaml_file:
                self.yaml_cfg = yaml.load(yaml_file, Loader=yaml.Loader)
            self.log("Read source: '%s'" % self.config_file_path)
        except:
            self.log("Source file '%s' is not a valid yaml file and could not be read!" % self.config_file_path, 2)
            sys.exit()

    def setup_target_file(self):
        """ Setup directory and backup existing configs if they have changed """
        backup_old_file = True
        if os.path.isfile(self.target_file_path):
            with open(self.target_file_path) as json_file:
                old_cfg = {}
                try:
                    old_cfg = json.load(json_file)
                except:
                    backup_old_file = True
                    self.log("Existing file '%s' could not be read." % self.target_file_path)
                if "ConfigParserInfo" in old_cfg:
                    time_str = old_cfg["ConfigParserInfo"]
                    time_str = time_str[-21:-1]
                    time = datetime.strptime(time_str, "%d.%m.%Y, %H:%M:%S")
                    change_time = datetime.fromtimestamp(os.path.getmtime(self.target_file_path))
                    if (change_time - time).total_seconds() < 30:
                        # file has not changed, we give 30s leeway for all the parsing
                        backup_old_file = False
                        self.log(
                            "Existing file '%s' was auto-generated and will be overwritten." % self.target_file_path)
                    else:
                        self.log("Existing file '%s' has been changed since auto-generation." % self.target_file_path)
                elif old_cfg is {}:
                    self.log("Existing file '%s' was not auto-generated." % self.target_file_path)

        else:
            self.log("No existing '%s' found." % self.target_file_path)
            backup_old_file = False
            target_dir = os.path.dirname(self.target_file_path)
            if not os.path.isdir(target_dir):
                os.mkdir(target_dir)
                rospy.loginfo("Created target directory '%s'." % target_dir)
        if backup_old_file:
            backup_name = self.target_file_path + datetime.now().strftime(".backup_%d_%m_%Y_%H_%M_%S")
            os.rename(self.target_file_path, backup_name)
            self.log("Created backup of existing settings at '%s'." % backup_name)

    @staticmethod
    def forward_args(in_dict, out_dict, exclusion=[]):
        """ Forward settings from one dict to the other, except exclusions"""
        for key in in_dict:
            if str(key[:1]).isupper():
                # All Airsim settings are CamelCase, unreal_airsim are lower_case
                if key not in exclusion:
                    out_dict[key] = in_dict[key]

    @staticmethod
    def parse_transformation(list_in, dict_out):
        if list_in is None:
            dict_out["X"] = 0
            dict_out["Y"] = 0
            dict_out["Z"] = 0
            dict_out["Roll"] = 0
            dict_out["Pitch"] = 0
            dict_out["Yaw"] = 0
        else:
            if len(list_in) != 4:
                log("Transformations are expected as 4x4 matrices, data will be ignored!", 1)
                self.parse_transformation(None, dict_out)
                return
            else:
                for l in list_in:
                    if len(l) != 4:
                        log("Transformations are expected as 4x4 matrices, data will be ignored!", 1)
                        self.parse_transformation(None, dict_out)
                        return
            dict_out["X"] = list_in[0][3]
            dict_out["Y"] = -list_in[1][3]
            dict_out["Z"] = -list_in[2][3]
            R = [x[:3] for x in list_in[:3]]
            R = Rotation.from_dcm(R)
            euler = R.as_euler('xyz', degrees=True)
            dict_out["Roll"] = round(euler[0], 3)
            dict_out["Pitch"] = round(-euler[1], 3)
            dict_out["Yaw"] = round(-euler[2], 3)

    def log(self, string, severity=0):
        if severity == 0:
            rospy.loginfo(string)
        elif severity == 1:
            rospy.logwarn(string)
        elif severity == 2:
            rospy.logerr(string)
        self.log_counter[severity] = self.log_counter[severity] + 1


if __name__ == '__main__':
    rospy.init_node('config_parser', anonymous=True)
    parser = ConfigParser()
    parser.parse()
