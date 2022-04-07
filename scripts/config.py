#!/usr/bin/env python3
import rospy
from customLog import Log

log = Log(context="Config", color="yellow")

class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class configure(metaclass=SingletonMeta): 
    
    def __init__(sefl):
        if(rospy.get_name() == "/unnamed"):
            log.print( "Ros node is not init")
            log.print( "Inicialing Ros node")
            rospy.init_node("GENERIC_NODE_CONFIG")

        log.print("Configure init")
        sefl.swarm_name = rospy.get_name()[1:]
        sefl.frame = str(rospy.get_param("/" + sefl.swarm_name + '/frame',"map"))
        sefl.number = int(rospy.get_param("/" + sefl.swarm_name + '/num_robots',1))
        sefl.mesh = str(rospy.get_param("/" + sefl.swarm_name + '/mesh_file',"package://swarm_visualization/meshes/test.stl"))
        sefl.r1 = float(rospy.get_param("/" + sefl.swarm_name + '/r1',1.1))
        sefl.r2 = float(rospy.get_param("/" + sefl.swarm_name + '/r2',2))


if __name__ == "__main__":    

    s1 = configure()
    s2 = configure()
    if id(s1) == id(s2):
        log.print("Singleton works, both variables contain the same instance.")
    else:
        log.print( "Singleton failed, variables contain different instances.")




