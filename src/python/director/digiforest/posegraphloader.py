from director import visualization as vis
from director import vtkNumpy as vnp
from PythonQt import QtCore, QtGui, QtUiTools

import numpy as np
import os
import re

class PoseGraphLoader():
    def __init__(self, data_dir: str, point_clouds_dir_name: str):
        self.point_clouds_dir_name = point_clouds_dir_name
        self.num_experiments = 0
        self.data_dir = data_dir
        self.polydata_payloads = [] # payload of each experiment contained in the pose graph file
        self.polydata_non_payloads = []  # other clouds
        self.trajectories = [] # trajectory for each experiments
        self.file_data = None
        self.median_pose_height = 0
        self.apply_offset = True
        self.offset = None

    def load_g2o_file(self, filename: str) -> bool:
        print("loading", filename)
        self.file_data = np.loadtxt(filename, delimiter=" ", dtype='<U21', usecols=np.arange(0,11))
        #only keep vertex SE3 rows
        self.file_data = np.delete(self.file_data, np.where(
                       (self.file_data[:, 0] == "EDGE_SE3:QUAT"))[0], axis=0)

        #rearrange colums
        self.file_data = np.delete(self.file_data, 0, axis=1)
        sec = self.file_data[:, 8]
        nsec = self.file_data[:, 9]
        # removing sec and nsec columns
        self.file_data = np.delete(self.file_data, 8, axis=1)
        self.file_data = np.delete(self.file_data, 8, axis=1)
        # reinsert them at correct location
        self.file_data = np.insert(self.file_data, 1, sec, axis=1)
        self.file_data = np.insert(self.file_data, 2, nsec, axis=1)

        self.file_data = self.file_data.astype(float, copy=False)
        return self._load_file_data(filename)

    def load_csv_file(self, filename: str) -> str:
        print("loading", filename)
        self.file_data = np.loadtxt(filename, delimiter=",", dtype=np.float64, skiprows=1)
        return self._load_file_data(filename)

    def median_pose_height(self):
        return self.median_pose_height

    def first_node_position(self, exp_num: int):
        return self.offset

    def _load_file_data(self, filename: str) -> bool:
        '''
        Load the pose graph data
        '''
        if not os.path.isdir(self.point_clouds_dir_name):
            print(self.point_clouds_dir_name, "doesn't exist, cannot load pose graph")
            return False



        exp_num = 1
        data = np.array([]) # point coordinates
        timestamps = np.array([], dtype=np.int64) # sec, nsec
        index_row = 1

        for row in self.file_data:
            # assumes that an experiment has less than 10000 elements
            if row[0] > exp_num*10000 or index_row == self.file_data.shape[0]:
                self.num_experiments += 1
                self.trajectories.append(data)
                # finding the payload nodes
                payload_dir = os.path.join(self.data_dir, self.point_clouds_dir_name)

                if os.path.isdir(payload_dir):
                    payload_files = [f for f in os.listdir(payload_dir) if os.path.isfile(os.path.join(payload_dir, f))]
                    points_payload = np.array([])  # point coordinates
                    index_payload = np.array([], dtype=np.int64)
                    for file in payload_files:
                        # ignore files that are not point clouds
                        file_extension = os.path.splitext(file)[1].lower()
                        if file_extension != ".ply" and file_extension != ".pcd":
                            continue

                        split = re.split('\W+|_', file)
                        if len(split) >= 3:
                            sec = int(split[1])
                            nsec= int(split[2])
                            index = np.where(np.logical_and(timestamps[:, 0] == sec, timestamps[:, 1] == nsec))

                            if len(index) >= 1 and index[0].size == 1:
                                index_payload = np.append(index_payload, int(index[0][0]))

                    if index_payload.size > 0:
                        points_payload = data[index_payload, :]
                        data = np.delete(data, index_payload, axis=0)
                        timestamps = np.delete(timestamps, index_payload, axis=0)
                        poly_data = vnp.numpyToPolyData(points_payload)

                        if not poly_data or not poly_data.GetNumberOfPoints():
                            print("Failed to read data from file: ", filename)
                            return False

                        self.polydata_payloads.append(poly_data)

                        zvalues = vnp.getNumpyFromVtk(poly_data, "Points")[:, 2]
                        self.median_pose_height = np.median(zvalues)

                # loading the non-payload nodes

                poly_data = vnp.numpyToPolyData(data)

                if not poly_data or not poly_data.GetNumberOfPoints():
                    print("Failed to read data from file: ", filename)
                    return False

                self.polydata_non_payloads.append(poly_data)

                exp_num += 1
                data = np.array([])
                timestamps = np.array([])
            else:
                if self.offset == None:
                    self.offset = [row[3], row[4], row[5]]

                if self.apply_offset and self.offset is not None:
                    row[3] -= self.offset[0]
                    row[4] -= self.offset[1]
                    row[5] -= self.offset[2]

                position = np.array([row[3], row[4], row[5]])
                timestamp = np.array([row[1], row[2]], dtype=np.int64)
                if data.shape[0] != 0:
                    data = np.vstack((data, position))
                    timestamps = np.vstack((timestamps, timestamp))
                else:
                    data = position
                    timestamps = timestamp
            index_row += 1

        return True

