#!/usr/bin/python3
import rospy
import random
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3


class Talker():
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        self.pub = rospy.Publisher('velocidades', Wrench, queue_size=10)
        self.rate = rospy.Rate(10)  #10hz
        self.resultado = rospy.Subscriber('resultado', Wrench, self.callback)
    
    
    def callback(self, msg):
        resultado = Wrench()
        resultado = msg
        rospy.loginfo(f"Módulo da Velocidade Linear: {resultado.force.x}\nMódulo da Velocidade Angular: {resultado.torque.x}")
        
        
    def run(self):
        while not rospy.is_shutdown():
            #Iniciando as componentes da velocidade linear randomicamente
            componente_x_linear = random.uniform(-1.0, 1.0)
            componente_y_linear = random.uniform(-1.0, 1.0)
            componente_z_linear = random.uniform(-1.0, 1.0)
            
            #Iniciando a velocidade linear com as componentes randomicamente geradas
            velocidade_linear = Vector3(componente_x_linear, componente_y_linear, componente_z_linear)
            
            #Iniciando as componentes da velocidade angular randomicamente
            componente_x_angular = random.uniform(-1.0, 1.0)
            componente_y_angular = random.uniform(-1.0, 1.0)
            componente_z_angular = random.uniform(-1.0, 1.0)
            
            #Iniciando a velocidade angular com as componentes randomicamente geradas
            velocidade_angular = Vector3(componente_x_angular, componente_y_angular, componente_z_angular)
            
            #Criando um objeto Wrench para armazenar as velocidades 
            data = Wrench()
            data.force = velocidade_linear  
            data.torque = velocidade_angular 
            
            self.pub.publish(data)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        talker = Talker()
        talker.run()
        
    except rospy.ROSInterruptException:
        pass
