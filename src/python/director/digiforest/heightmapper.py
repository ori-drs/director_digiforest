from director import visualization as vis
from director.timercallback import TimerCallback
from director import objectmodel as om
from director.debugpolydata import DebugData
from director import ioutils
from director import segmentation
from director import vtkNumpy as vnp
from PythonQt import QtCore, QtGui, QtUiTools
from director.digiforest.utils import convert_heights_mesh

import digiforest_drs as df

import os
import numpy as np
from typing import List
import pcl

class HeightMapper():

    def _show_pclXYZnormal(self, cloud_pc, name: str, visible: bool, parent: str):
        array_xyz = cloud_pc.to_array()[:, 0:3]
        cloud_pc = pcl.PointCloud()
        cloud_pc.from_array(array_xyz)
        cloud_pd = vnp.getVtkPolyDataFromNumpyPoints(cloud_pc.to_array())
        vis.showPolyData(cloud_pd, name, visible=visible, parent=parent)

    def terrain_mapping(self, filename: str, height_map_file: str,
                        median_pose_height: float):
        cloud_pc = pcl.PointCloud_PointNormal()
        ext = os.path.splitext(filename)[1].lower()
        #python-pcl only works with pcd files
        if ext == ".pcd":
            cloud_pc._from_pcd_file(filename.encode('utf-8'))
        else:
            print("Cannot load", filename)
            return

        self._show_pclXYZnormal(cloud_pc, "Cloud Raw", visible=False, parent=os.path.basename(filename))

        # remove non-up points
        cloud = df.filterUpNormal(cloud_pc, 0.95)
        self._show_pclXYZnormal(cloud, "Cloud Up Normals only", visible=False, parent=os.path.basename(filename))

        # drop from xyznormal to xyz
        array_xyz = cloud.to_array()[:, 0:3]
        cloud = pcl.PointCloud()
        cloud.from_array(array_xyz)

        # get the terrain height
        self.heights_array_raw = df.getTerrainHeight(cloud)
        convert_heights_mesh(self.heights_array_raw, height_map_file)
        self.display_height_map_file(height_map_file, parent=os.path.basename(filename),
                                     median_pose_height=median_pose_height)

        self.heights_pd = vnp.getVtkPolyDataFromNumpyPoints(self.heights_array_raw)
        obj = vis.showPolyData(self.heights_pd, 'Heights', color=[0, 1, 0], visible=False,
                               parent=os.path.basename(filename))
        obj.setProperty('Point Size', 10)

        # filter NaNs *** these could have been removed in function ***
        self.heights_array = self.heights_array_raw[self.heights_array_raw[:, 2] < 10]
        self.heights_pd = vnp.getVtkPolyDataFromNumpyPoints(self.heights_array)
        obj = vis.showPolyData(self.heights_pd, 'Heights Filtered', visible=False, color=[0, 0, 1],
                               parent=os.path.basename(filename))
        obj.setProperty('Point Size', 10)

    def display_height_map_file(self, height_map_file: str, parent: str, median_pose_height: float):
        height_mesh = ioutils.readPolyData(height_map_file)
        height_mesh = segmentation.addCoordArraysToPolyDataXYZ(height_mesh)
        vis.showPolyData(height_mesh, 'Height Mesh', 'Color By', 'z',
                         colorByRange=[median_pose_height - 4,
                                       median_pose_height + 4], parent=parent)






