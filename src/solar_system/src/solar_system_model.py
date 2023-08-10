#!/usr/bin/env python3

import os
import yaml
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    
    rospy.init_node('solar_system_model')
    broadcaster = tf2_ros.TransformBroadcaster()

    #Diretório atual do script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    #Caminho para o arquivo YAML
    config_file = os.path.join(script_dir, "../param/solar_params.yaml")

    #Abertura do arquivo e leitura
    with open(config_file) as f:
        config_data = yaml.load(f, Loader=yaml.FullLoader)

    #Parâmetros dos planetas
    planet_params = config_data["planet_params"]

    rate = rospy.Rate(10)

    #Loop principal
    while not rospy.is_shutdown():
        #Tempo atual do ROS
        current_time = rospy.Time.now()

        #Loop de cada planeta
        for planet in planet_params:
            planet_name = planet["name"]
            planet_radius = planet["radius"]
            satellite_radius = planet["satellite_radius"]
            planet_rotation_direction = planet["planet_rotation_direction"]
            satellite_rotation_direction = planet["satellite_rotation_direction"]
            planet_rotation_rate = planet["planet_rotation_rate"]
            satellite_rotation_rate = planet["satellite_rotation_rate"]

            #Ângulo de rotação do planeta
            planet_angle = (planet_rotation_direction * planet_rotation_rate * current_time.to_sec()) % (2.0 * math.pi)
            
            #Transformação do planeta
            planet_transform = TransformStamped()
            planet_transform.header.stamp = current_time
            planet_transform.header.frame_id = "star"
            planet_transform.child_frame_id = planet_name
            planet_transform.transform.translation.x = planet_radius * math.cos(planet_angle)
            planet_transform.transform.translation.y = planet_radius * math.sin(planet_angle)
            planet_transform.transform.translation.z = 0.0
            planet_transform.transform.rotation.x = 0.0
            planet_transform.transform.rotation.y = 0.0
            planet_transform.transform.rotation.z = math.sin(planet_angle / 2)
            planet_transform.transform.rotation.w = math.cos(planet_angle / 2)
            broadcaster.sendTransform(planet_transform)

            #Ângulo de rotação do satélite
            satellite_angle = (satellite_rotation_direction * satellite_rotation_rate * current_time.to_sec()) % (2.0 * math.pi)
            
            #Transformação do satélite
            satellite_transform = TransformStamped()
            satellite_transform.header.stamp = current_time
            satellite_transform.header.frame_id = planet_name
            satellite_transform.child_frame_id = planet_name + "_satellite"
            satellite_transform.transform.translation.x = satellite_radius * math.cos(satellite_angle)
            satellite_transform.transform.translation.y = satellite_radius * math.sin(satellite_angle)
            satellite_transform.transform.translation.z = 0.0
            satellite_transform.transform.rotation.x = 0.0
            satellite_transform.transform.rotation.y = 0.0
            satellite_transform.transform.rotation.z = math.sin(satellite_angle / 2)
            satellite_transform.transform.rotation.w = math.cos(satellite_angle / 2)
            broadcaster.sendTransform(satellite_transform)


        rate.sleep()


if __name__ == "__main__":
    main()
