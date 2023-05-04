from utm import from_latlon

from director.thirdparty import transformations

import numpy as np

from pymap3d import enu2geodetic

class CoordinatesConverter:

    def __init__(self):
        self.t_enu_map = None
        self.lla_ref = None

    def parse_g2o_file(self, geo_filename: str):
        # file_data = np.genfromtxt(geo_filename, delimiter=" ", dtype='<U21', usecols=np.arange(0, 8))
        # row_lla_map = file_data[np.where(file_data[:, 0] == "GNSS_LLA_TO_MAP")[0]][0]
        # position = [row_lla_map[1], row_lla_map[2], row_lla_map[3]]
        # quat = [row_lla_map[4], row_lla_map[5], row_lla_map[6]]
        # self.t_enu_map = transformUtils.transformFromPose(position, quat)
        # self.lla_ref = file_data[np.where(file_data[:, 0] == "GNSS_LLA_REF")[0]][0]

        for line in open(geo_filename):
            row = line.split(" ")
            if len(row) > 0:
                if row[0] == "GNSS_LLA_TO_MAP":
                    position = [float(row[1]), float(row[2]), float(row[3])]
                    quat = [float(row[4]), float(row[5]), float(row[6]), float(row[7])]
                    self.t_enu_map = np.identity(4)
                    self.t_enu_map = transformations.quaternion_matrix(quat)
                    self.t_enu_map[:3, 3] = position
                elif row[0] == "GNSS_LLA_REF":
                    self.lla_ref = [float(row[1]), float(row[2]), float(row[3])]

    def map_to_utm(self, position):
        lat, lon, alt = self.map_to_latlong(position)
        easting, northing = self.latlong_to_utm(lat, lon)
        return easting, northing, alt

    def map_to_latlong(self, position):
        pose = np.identity(4)
        pose[0, 3] = position[0]
        pose[1, 3] = position[1]
        pose[2, 3] = position[2]
        pose_enu = np.linalg.inv(self.t_enu_map) * pose
        lat, lon, alt = enu2geodetic(pose_enu[0, 3], pose_enu[1, 3], pose_enu[2, 3],
                                     self.lla_ref[0], self.lla_ref[1], self.lla_ref[2])
        return lat, lon, alt


    def latlong_to_utm(self, lat, lon):
        easting, northing, zone_num, zone_letter = from_latlon(lat, lon)
        return easting, northing


