#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import  PointCloud2


def filter_points(array):

    # Extraer los campos individuales
    x = array['x']
    y = array['y']
    z = array['z']

    # Condiciones de filtrado
    cond_x = (x <= 6) & (x > -0.5)
    cond_y = np.abs(y) <= 4
    cond_z = (z < 0.5) & (z > -0.125)
    distanciaOrigen = np.sqrt(x**2 + y**2 + z**2)
    umbral = 0.9

    # Combinar las condiciones
    filtro = cond_x & cond_y & cond_z & (distanciaOrigen > umbral)

    # Aplicar el filtro y devolver el resultado
    return array[filtro]


def lidar_callback(data):
    # Convertir PointCloud2 a un array de NumPy
    array = ros_numpy.point_cloud2.pointcloud2_to_array(data)

    modArray= np.copy(array)

    modArray['z'] = -modArray['z']
    modArray['y'] = -modArray['y']

    # Filtrar la nube de puntos
    filtered_array = filter_points(modArray)

    # Convertir de nuevo a PointCloud2 y publicar
    corrected = ros_numpy.point_cloud2.array_to_pointcloud2(filtered_array, stamp=data.header.stamp, frame_id=data.header.frame_id)
    pub.publish(corrected)


rospy.init_node('lidar_correction_node')

sub= rospy.Subscriber('/rslidar_points', PointCloud2, lidar_callback)

pub= rospy.Publisher('/corrected_rslidar_points', PointCloud2, queue_size= 10)

rospy.spin()