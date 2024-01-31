#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

# Rango de altura para los conos
RANGO_ALTURA_CONOS = [0.2, 0.55]

def posibleCono(cluster, rango_altura):

    #Calculo de la Z del cluster

    z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
    altura = z_max - z_min
    return rango_altura[0] <= altura <= rango_altura[1]

def detectar_conos(array, rango_altura):

    puntos = np.column_stack((array['x'], array['y'], array['z']))
    clustering = DBSCAN(eps=0.25, min_samples=50).fit(puntos)
    labels = clustering.labels_

    conos_potenciales = []
    for label in set(labels):
        if label == -1:
            continue

        cluster = puntos[labels == label]
        if posibleCono(cluster, rango_altura):
            conos_potenciales.append(cluster)

    return conos_potenciales

def filter_points(array):
    x, y, z = array['x'], array['y'], array['z']
    cond_x = (x <= 6) & (x > -0.5)
    cond_y = np.abs(y) <= 4
    cond_z = z < 0.5


    distanciaOrigen = np.sqrt(x**2 + y**2 + z**2)
    umbral = 0.9

    filtro = cond_x & cond_y & cond_z & (distanciaOrigen > umbral)
    return array[filtro]

def groundFilter(array):

    x, y, z = array['x'], array['y'], array['z']

    #Combinar x, y en un array 2D
    X = np.column_stack((x, y))
    ransac = RANSACRegressor(min_samples=50, residual_threshold=0.05)
    ransac.fit(X, z)

    #Obtener inliers y seleccionar outliers
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    # Filtrar puntos que no son suelo (outliers)
    return array[outlier_mask]

def lidar_callback(data):
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    modArray = np.copy(array)
    modArray['z'] = -modArray['z']
    modArray['y'] = -modArray['y']

    filtered_array = filter_points(modArray)
    array_sin_suelo = groundFilter(filtered_array)
    conos = detectar_conos(array_sin_suelo, RANGO_ALTURA_CONOS)

    for cono in conos:
        punto_medio = np.mean(cono, axis=0)
        print(f"Cono detectado en: x={punto_medio[0]:.2f}, y={punto_medio[1]:.2f}, z={punto_medio[2]:.2f}")

    corrected = ros_numpy.point_cloud2.array_to_pointcloud2(array_sin_suelo, stamp=data.header.stamp, frame_id=data.header.frame_id)
    pub.publish(corrected)

rospy.init_node('lidar_correction_node')
sub = rospy.Subscriber('/rslidar_points', PointCloud2, lidar_callback)
pub = rospy.Publisher('/corrected_rslidar_points', PointCloud2, queue_size=10)
rospy.spin()
