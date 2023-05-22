from utm import from_latlon

from director.thirdparty import transformations
import PythonQt

import numpy as np

from pymap3d import enu2geodetic

class CoordinatesConverter:

    def __init__(self):
        self.t_enu_map = None
        self.lla_ref = None
        #self.gnss_handler = PythonQt.dd.ddGnssHandler()

    def parse_g2o_file(self, geo_filename: str):

        for line in open(geo_filename):
            row = line.split(" ")
            if len(row) > 0:
                if row[0] == "GNSS_LLA_TO_MAP":
                    position = [float(row[1]), float(row[2]), float(row[3])]
                    self.t_enu_map = np.identity(4)
                    self.t_enu_map = transformations.quaternion_matrix([float(row[7]), float(row[4]), float(row[5]), float(row[6])])
                    self.t_enu_map[:3, 3] = position
                elif row[0] == "GNSS_LLA_REF":
                    self.lla_ref = [float(row[1]), float(row[2]), float(row[3])]

    def map_to_utm(self, position: np.ndarray):
        enu = self.map_to_enu(position)
        lat, lon, alt = self.enu_to_latlong(enu)
        easting, northing = self.latlong_to_utm(lat, lon)
        return [easting, northing, alt]

    def map_to_enu(self, position: np.ndarray):
        pose = np.identity(4)
        pose[0, 3] = position[0]
        pose[1, 3] = position[1]
        pose[2, 3] = position[2]
        pose_enu = np.linalg.inv(self.t_enu_map) @ pose
        return pose_enu[0:3, 3]

    def enu_to_latlong(self, position_enu: np.ndarray):
        lat, lon, alt = enu2geodetic(position_enu[0], position_enu[1], position_enu[2],
                                     self.lla_ref[0], self.lla_ref[1], self.lla_ref[2])
        return lat, lon, alt


    def latlong_to_utm(self, lat: float, lon: float):
        easting, northing, zone_num, zone_letter = from_latlon(lat, lon)
        #easting, northing = self.gnss_handler.convertWGS84toEPSG3067(lat, lon)
        return easting, northing


