from director import vtkNumpy as vnp
import director.ioutils as io

import pcl
import os
import shutil

import vtk
import numpy as np
from vtk.util.numpy_support import vtk_to_numpy

def convert_poly_data_to_pcd(poly_data, filename: str):
    '''
    Converts ply file to pcd
    '''
    ext = os.path.splitext(filename)[1].lower()
    basename = os.path.splitext(filename)[0]
    if ext == ".pcd":
        return

    if os.path.isfile(basename + ".pcd"):
        # file already exists
        return

    points = vtk_to_numpy(poly_data.GetPoints().GetData())
    normals = vtk_to_numpy(poly_data.GetPointData().GetNormals())
    zeros_array = np.zeros((points.shape[0], 1))

    # Combine points and normals into a single NumPy array
    array = np.concatenate((points, normals, zeros_array), axis=-1)
    array = array.astype(np.float32)

    cloud = pcl.PointCloud_PointNormal()
    cloud.from_array(array)

    #_ = io.writePolyData(poly_data, "/tmp/cloud.ply")
    pcl.save_PointNormal(cloud, "/tmp/cloud.pcd")
    shutil.copyfile("/tmp/cloud.pcd", basename + ".pcd")