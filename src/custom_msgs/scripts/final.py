#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.spatial import Delaunay
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from custom_msgs.msg import Map
from std_msgs.msg import ColorRGBA
from sklearn.cluster import DBSCAN

cones = [] 
route_publisher = None
triangles_publisher = None

EPS= 5.
MIN_SAMPLES= 3

def filtrar_aislados(cones):
    pos= np.array([cone['position'] for cone in cones])
    clustering= DBSCAN(eps= EPS, min_samples= MIN_SAMPLES).fit(pos)
    filtrados= [cones[i] for i in range(len(cones)) if clustering.labels_[i] != -1]
    return filtrados

def procesa_ruta():
    if not cones or len(cones) < 3:
        rospy.loginfo("No se han recibido suficientes conos para la triangulación")
        return

    points = np.array([cone['position'] for cone in cones])
    delaunay = Delaunay(points)
    
    route, simplices = calculate_route(cones, delaunay)
    visualize_triangles(points, delaunay.simplices, simplices, triangles_publisher) 
    publish_route(route, route_publisher)

def ordenar_midpoints(midpoints):
    if not midpoints:
        return []

    def distancia(p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    punto_actual = min(midpoints, key=lambda p: distancia(p, (0, 0)))
    midpoints_ordenados = [punto_actual]

    midpoints.remove(punto_actual)

    while midpoints:
        siguiente_punto = min(midpoints, key=lambda p: distancia(p, punto_actual))
        midpoints_ordenados.append(siguiente_punto)
        midpoints.remove(siguiente_punto)
        punto_actual = siguiente_punto

    return midpoints_ordenados



def calculate_route(cones, delaunay):
    route = []
    seen_midpoints = set()
    simplices_validos = set() 

    for simplex_index, simplex in enumerate(delaunay.simplices):
        simplex_cones = [cones[i] for i in simplex]
        aristas = [(simplex_cones[i], simplex_cones[(i + 1) % 3]) for i in range(3)]
        aristas_validas = [arista for arista in aristas if arista[0]['color'] != arista[1]['color']]

        if aristas_validas: 
            simplices_validos.add(simplex_index) 

        for cone1, cone2 in aristas_validas:
            midpoint = ((cone1['position'][0] + cone2['position'][0]) / 2,
                        (cone1['position'][1] + cone2['position'][1]) / 2)
            if midpoint not in seen_midpoints:
                route.append(midpoint)
                seen_midpoints.add(midpoint)

    route= ordenar_midpoints(route)
    print(route)

    return route, simplices_validos

def visualize_triangles(points, simplices, simplices_validos, publisher):
    marker = Marker()
    marker.header.frame_id = "rslidar"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.g = 1.0 

    for simplex_index in simplices_validos:
        simplex = simplices[simplex_index]
        for i in range(3):
            p1 = Point(x=points[simplex[i]][0], y=points[simplex[i]][1], z=0)
            p2 = Point(x=points[simplex[(i + 1) % 3]][0], y=points[simplex[(i + 1) % 3]][1], z=0)
            marker.points.append(p1)
            marker.points.append(p2)

    publisher.publish(marker)

def publish_route(route, publisher):
    marker = Marker()
    marker.header.frame_id = "rslidar"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0

    for p in route:
        point = Point(x=p[0], y=p[1], z=0)
        marker.points.append(point)

    publisher.publish(marker)


def publish_cones(cones, publisher):
    marker = Marker()
    marker.header.frame_id = "rslidar"
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1  
    marker.scale.x = 0.2 
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0

    for cone in cones:
        point = Point(x=cone['position'][0], y=cone['position'][1], z=0)
        marker.points.append(point)
        color = ColorRGBA()  
        if cone['color'] == 'b': 
            color = ColorRGBA(0, 0, 1, 1)  
        elif cone['color'] == 'y': 
            color = ColorRGBA(1, 1, 0, 1)  
        marker.colors.append(color)

    if not marker.points:  
        rospy.logwarn("Intentando publicar un Marker sin puntos.")
        return

    publisher.publish(marker)


def callback(msg):
    global cones
    cones = [{'position': (cone.position.x, cone.position.y), 'color': cone.color} for cone in msg.cones if cone.color != 'o' and 
                                                                cone.confidence > 0.9]
    cones= filtrar_aislados(cones)
    publish_cones(cones, cone_publisher)
    procesa_ruta()

def listener():
    global route_publisher, triangles_publisher, cone_publisher
    rospy.init_node('subscriber_node')

    rospy.Subscriber('perception_map', Map, callback)
    route_publisher = rospy.Publisher('visualization_route', Marker, queue_size=10)
    triangles_publisher = rospy.Publisher('visualization_triangles', Marker, queue_size=10)
    cone_publisher = rospy.Publisher('visualization_cones', Marker, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()