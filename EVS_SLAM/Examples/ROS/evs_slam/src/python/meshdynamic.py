
#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import open3d as o3d
import pymesh
import trimesh

from scipy.interpolate import NearestNDInterpolator
from scipy.interpolate import griddata

def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    #print("generation of "+str(i)+" LoD successful")
    return mesh_lods

#input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/kitti_pointcloud.pcd"
#input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/ciclopista_pointcloud.pcd"
#input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/cio_pointcloud.pcd"
input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/pointcloud.pcd"
output_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/"
# Carga el archivo PCD en un DataFrame de pandas
df = pd.read_csv(input_file, comment='/', delim_whitespace=True, header=None, skiprows=11, names=['x', 'y', 'z', 'dynamic'])

# Separa los puntos en dos DataFrames basándote en el valor de 'dynamic'
df_static = df[df['dynamic'] == 0]
df_dynamic = df[df['dynamic'] == 1]

# Crea nubes de puntos Open3D a partir de los DataFrames
pcd_static = o3d.geometry.PointCloud()
pcd_static.points = o3d.utility.Vector3dVector(df_static[['x', 'y', 'z']].values)

pcd_dynamic = o3d.geometry.PointCloud()
pcd_dynamic.points = o3d.utility.Vector3dVector(df_dynamic[['x', 'y', 'z']].values)

pcd_dynamic.paint_uniform_color([1, 0.5, 0])  

pcd_static.paint_uniform_color([0.2, 0.5, 1])  

#o3d.visualization.draw_geometries([pcd_dynamic,pcd_static])

#profundidad
limitar_radio=True
if limitar_radio is True:

    # Calcula el centro de la nube de puntos estáticos
    center = np.asarray(pcd_static.points).mean(axis=0)
    center_dynamic = np.asarray(pcd_dynamic.points).mean(axis=0)

    # Calcula la distancia de cada punto al centro
    distances = np.sqrt(np.sum((np.asarray(pcd_static.points) - center)**2, axis=1))
    distances_dynamic = np.sqrt(np.sum((np.asarray(pcd_dynamic.points) - center_dynamic)**2, axis=1))

    print("La distancia más lejana es:", np.max(distances))

    # Define el radio
    radio = 550 #550  # Ajusta este valor según tus necesidades
    # Filtra los puntos que están más allá del radio
    pcd_static = pcd_static.select_by_index(np.where(distances <= radio)[0])
    pcd_dynamic = pcd_dynamic.select_by_index(np.where(distances_dynamic <= radio)[0])

    # Visualiza las nubes de puntos
    #o3d.visualization.draw_geometries([pcd_static])

#o3d.visualization.draw_geometries([pcd_dynamic,pcd_static])

# Aplicar el filtro a ambas nubes de puntos
nb_neighbors=18 #18 #30 20
std_ratio=0.05 #0.3 #0.05 2

dynamic=True
if dynamic==True:

    cl, ind = pcd_dynamic.remove_statistical_outlier(nb_neighbors, std_ratio)
    pcd_dynamic = pcd_dynamic.select_by_index(ind)

    # Calcula el centro de la nube de puntos dinámicos
    center_dynamic = np.asarray(pcd_dynamic.points).mean(axis=0)

    # Calcula la distancia de cada punto al centro
    distances_dynamic = np.sqrt(np.sum((np.asarray(pcd_dynamic.points) - center_dynamic) ** 2, axis=1))

    # Filtra los puntos que están más allá del radio
    pcd_dynamic = pcd_dynamic.select_by_index(np.where(distances_dynamic <= radio)[0])

    # Aumenta el tamaño del voxel para reducir la nube de puntos
    pcd_dynamic = pcd_dynamic.voxel_down_sample(voxel_size=0.8)

    points = np.asarray(pcd_dynamic.points)
    interpolator = NearestNDInterpolator(points[:, :2], points[:, 2])


    grid_x, grid_y = np.mgrid[min(points[:, 0]):max(points[:, 0]):100j, min(points[:, 1]):max(points[:, 1]):100j]
    grid_z = griddata(points[:, :2], points[:, 2], (grid_x, grid_y), method='linear')
    
    new_points = np.column_stack((grid_x.flatten(), grid_y.flatten(), grid_z.flatten()))
    pcd_interpolated = o3d.geometry.PointCloud()
    pcd_interpolated.points = o3d.utility.Vector3dVector(new_points)

    pcd_dynamic.paint_uniform_color([1, 0.5, 0])  
    pcd_interpolated.paint_uniform_color([0, 0.5, 1])  
    o3d.visualization.draw_geometries([pcd_dynamic])

nb_neighbors=30 #30 20
std_ratio=0.05 #0.05 2

static=True
if static==True:
    cl, ind = pcd_static.remove_statistical_outlier(nb_neighbors, std_ratio)
    pcd_static = pcd_static.select_by_index(ind)
        

    pcd_static.paint_uniform_color([0.2, 0.5, 1])  

    voxel_size = 0.4  # Ajusta el tamaño del voxel según tus necesidades
    pcd_static = pcd_static.voxel_down_sample(voxel_size)
    #pcd_static = pcd_static.voxel_down_sample(voxel_size)

    o3d.visualization.draw_geometries([pcd_static])


    o3d.visualization.draw_geometries([pcd_static,pcd_dynamic])
    