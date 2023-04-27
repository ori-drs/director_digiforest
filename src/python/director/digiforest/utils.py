from director import vtkNumpy as vnp
import director.ioutils as io

import pcl
import os
import shutil

import vtk
import numpy as np
from vtk.util.numpy_support import vtk_to_numpy

def convert_poly_data_to_pcd(poly_data, path: str,
                             output_dir: str):
    '''
    Converts ply file to pcd
    '''
    ext = os.path.splitext(path)[1].lower()
    filename_with_extension = os.path.basename(path)
    filename_without_extension = os.path.splitext(filename_with_extension)[0]
    new_path = os.path.join(output_dir, filename_without_extension)+ ".pcd"
    if ext == ".pcd":
        return new_path

    if os.path.isfile(new_path):
        # file already exists
        return new_path

    points = vtk_to_numpy(poly_data.GetPoints().GetData())
    normals = vtk_to_numpy(poly_data.GetPointData().GetNormals())
    zeros_array = np.zeros((points.shape[0], 1))

    # Combine points and normals into a single NumPy array
    array = np.concatenate((points, normals, zeros_array), axis=-1)
    array = array.astype(np.float32)

    cloud = pcl.PointCloud_PointNormal()
    cloud.from_array(array)

    pcl.save_PointNormal(cloud, "/tmp/cloud.pcd")
    shutil.copyfile("/tmp/cloud.pcd", new_path)
    return new_path
