#!/usr/bin/env python
import rospy
from household_objects_database_msgs.srv import GetModelMesh
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
import copy

class FakeMarkerServer():
    def __init__(self):
        # create the IM server
        self.server = InteractiveMarkerServer('fake_marker_server')
        # used to get model meshes
        self.get_mesh = rospy.ServiceProxy('/objects_database_node/get_model_mesh', GetModelMesh)
        self.objects = []
        self.objects.append(18796)
        self.objects.append(18786)
        self.objects.append(18722)
        pose = Pose()
        pose.position.z = .7366
        pose.position.x = 1.5
        pose.position.y = -0.25
        for obj_id in self.objects:
            self.create_mesh(obj_id, pose)
            pose.position.y = pose.position.y + 0.25
        
    def create_mesh(self, mesh_id, pose):
        response = self.get_mesh(mesh_id)
        mesh = response.mesh
        # build the mesh marker
        marker = Marker()
        marker.color.r = 0.125
        marker.color.g = 0.29
        marker.color.b = 0.62352941176
        marker.color.a = 0.66
        marker.frame_locked = False
        marker.type = Marker.TRIANGLE_LIST
        # add the mesh
        for j in range(len(mesh.triangles)):
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[0]])
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[1]])
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[2]])
        # create the interactive marker
        int_marker = InteractiveMarker()
        int_marker.pose = copy.deepcopy(pose)
        int_marker.header.frame_id = 'base_link'
        name = 'object_' + str(mesh_id)
        int_marker.name = name
        # move freely on the X-Y plane
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)
        # pick up on release
        self.server.insert(int_marker, self.process_feedback)
        self.server.setCallback(name, self.release, InteractiveMarkerFeedback.MOUSE_UP)
        self.server.applyChanges()
        
    def process_feedback(self, feedback):
        self.last_feedback = feedback

    def release(self, feedback):
        print feedback
        
    def update(self):
        self.server.applyChanges()

if __name__ == '__main__':    
    rospy.init_node('fake_object_markers')
    server = FakeMarkerServer()
    while not rospy.core.is_shutdown():
        server.update()
