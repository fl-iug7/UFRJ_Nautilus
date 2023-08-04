#!/usr/bin/python3
import math
import rospy
from geometry_msgs.msg import Wrench


class Listener():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("velocidades", Wrench, self.callback)
        self.pub = rospy.Publisher('resultado', Wrench, queue_size=10)


    def callback(self, data):
        #Calcula o quadrado das componentes da velocidade linear
        componente_x_linear2 = data.force.x * data.force.x
        componente_y_linear2 = data.force.y * data.force.y
        componente_z_linear2 = data.force.z * data.force.z
        
        #Calcula o módulo da velocidade linear
        modulo_velocidade_linear = math.sqrt(componente_x_linear2 + componente_y_linear2 + componente_z_linear2)

        #Calcula o quadrado das componentes da velocidade angular
        componente_x_angular2 = data.torque.x * data.torque.x
        componente_y_angular2 = data.torque.y * data.torque.y
        componente_z_angular2 = data.torque.z * data.torque.z
        
        #Calcula o módulo da velocidade angular
        modulo_velocidade_angular = math.sqrt(componente_x_angular2 + componente_y_angular2 + componente_z_angular2)

        # Cria um objeto Wrench para armazenar as velocidades linear e angular calculadas
        resultado = Wrench()
        resultado.force.x = modulo_velocidade_linear  
        resultado.torque.x = modulo_velocidade_angular  
        
        self.pub.publish(resultado)


if __name__ == '__main__':
    l = Listener()
    rospy.spin()
