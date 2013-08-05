#!/usr/bin/env python
import rospy
from interactive_world.interactive_world_server import InteractiveWorldServer

if __name__ == '__main__':
    rospy.init_node('interactive_world_server')
    # create the server and continue to request updates
    server = InteractiveWorldServer()
    while not rospy.core.is_shutdown():
        server.update()
