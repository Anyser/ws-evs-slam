
#!/usr/bin/env python3
import os
import numpy as np
import open3d as o3d


input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/pointcloud.pcd"
output_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/mesh.ply"

# Carga la nube de puntos
pcd = o3d.io.read_point_cloud(input_file)

# Asegúrate de que la nube de puntos no está vacía
if not pcd.is_empty():
    # Estima las normales
    pcd.estimate_normals()
    # Filtra los outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=3.0)
    pcd = pcd.select_by_index(ind)

    # Crea la malla usando el algoritmo de Poisson
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

    # Guarda la malla en un archivo .ply
    is_saved = o3d.io.write_triangle_mesh(output_file, mesh)

    if is_saved:
        print(f"La malla se ha guardado correctamente en: {output_file}")
    else:
        print("Hubo un error al guardar la malla.")
else:
    print("La nube de puntos está vacía.")


"""
#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import open3d as o3d


input_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/kitti_pointcloud.pcd"
output_file = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_mesh.ply"
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


#profundidad
limitar_radio=True
if limitar_radio is True:

    # Calcula el centro de la nube de puntos estáticos
    center = np.asarray(pcd_static.points).mean(axis=0)

    # Calcula la distancia de cada punto al centro
    distances = np.sqrt(np.sum((np.asarray(pcd_static.points) - center)**2, axis=1))
    print("La distancia más lejana es:", np.max(distances))

    # Define el radio
    radio = 550  # Ajusta este valor según tus necesidades
    # Filtra los puntos que están más allá del radio
    pcd_static = pcd_static.select_by_index(np.where(distances <= radio)[0])
    # Visualiza las nubes de puntos
    #o3d.visualization.draw_geometries([pcd_static])


#profundidad
limitar=False
if limitar is True:
    distance_threshold = 1000 #10
    pcd_static = pcd_static.select_by_index(
        [i for i, point in enumerate(pcd_static.points) if np.linalg.norm(point) < distance_threshold]
    )
    pcd_dynamic = pcd_dynamic.select_by_index(
        [i for i, point in enumerate(pcd_dynamic.points) if np.linalg.norm(point) < distance_threshold]
    )

# Filtrado estadístico para eliminar ruido

# Aplicar el filtro a ambas nubes de puntos
nb_neighbors= 30 #20
std_ratio=0.05 #2


cl, ind = pcd_static.remove_statistical_outlier(nb_neighbors, std_ratio)
pcd_static = pcd_static.select_by_index(ind)

pcd_static.paint_uniform_color([0.2, 0.5, 1])  
pcd_dynamic.paint_uniform_color([1, 0.5, 0])  # Verde para puntos dinámicos

o3d.visualization.draw_geometries([pcd_static])
#quitandole vecinos
#output_pcd_static = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_pcd_static_kitti.pcd"
#o3d.io.write_point_cloud(output_pcd_static, pcd_static)

output_pcd_static = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_pcd_static.pcd"
o3d.io.write_point_cloud(output_pcd_static, pcd_static)

output_ply_static = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_static.ply"
o3d.io.write_point_cloud(output_ply_static, pcd_static, write_ascii=True)



cl, ind = pcd_dynamic.remove_statistical_outlier(nb_neighbors, std_ratio)
pcd_dynamic = pcd_dynamic.select_by_index(ind)

# Filtrado de ruido basado en la densidad
pcd_dynamic, ind = pcd_dynamic.remove_radius_outlier(nb_points=50, radius=3)
pcd_static, ind = pcd_static.remove_radius_outlier(nb_points=50, radius=3)
o3d.visualization.draw_geometries([pcd_static])

# Asigna colores a los puntos estáticos y dinámicos
pcd_static.paint_uniform_color([0.2, 0.5, 1])  
pcd_dynamic.paint_uniform_color([1, 0.5, 0])  # Verde para puntos dinámicos

# Visualiza las nubes de puntos
#o3d.visualization.draw_geometries([pcd_static, pcd_dynamic])
#o3d.visualization.draw_geometries([pcd_static])
#output_pcd_static = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_pcd_static.pcd"
#o3d.io.write_point_cloud(output_pcd_static, pcd_static)

#output_ply_static = "/home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/ROS/orb_slam2_wod/src/python/kitti_static.ply"
#o3d.io.write_point_cloud(output_ply_static, pcd_static, write_ascii=True)
"""