#!/usr/bin/env python
import rospy
from household_objects_database_msgs.srv import GetModelMesh
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_world_hackathon.srv import SaveTemplate, SaveTemplateResponse
import copy
import pickle

TABLE_HEIGHT = 0.75
OFFSET = 0.381
DEPTH_START = 0.0254
DEPTH_END = 0.381
WIDTH_START = -0.3175
WIDTH_END = 0.3175

SAVE_FILE = '/tmp/templates.r0b0t'

class FakeMarkerServer():
    def __init__(self):
        # create the save service
        rospy.Service('~save_template', SaveTemplate, self.save)
        # create the IM server
        self.server = InteractiveMarkerServer('~fake_marker_server')
        # used to get model meshes
        self.get_mesh = rospy.ServiceProxy('/objects_database_node/get_model_mesh', GetModelMesh)
        self.objects = []
        self.objects.append(18796)
        self.objects.append(18786)
        self.objects.append(18722)
        pose = Pose()
        pose.position.z = TABLE_HEIGHT
        pose.position.x = OFFSET * 3
        pose.position.y = -0.25
        for obj_id in self.objects:
            self.create_mesh(obj_id, pose)
            pose.position.y = pose.position.y + 0.25
        # check for saved templates
        try:
            self.templates = pickle.load(open(SAVE_FILE, 'rb'))
            rospy.loginfo('Loaded ' + str(len(self.templates.keys())) + ' template(s).')
        except:
            self.templates = dict()
            rospy.loginfo('New template file started.')
            
    def create_name(self, mesh_id):
        return 'object_' + str(mesh_id)
        
    def create_mesh(self, mesh_id, pose):
        response = self.get_mesh(mesh_id)
        mesh = response.mesh
        # build the mesh marker
        marker = Marker()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.66
        marker.frame_locked = False
        marker.type = Marker.TRIANGLE_LIST
        # add the mesh
        for j in range(len(mesh.triangles)):
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[0]])
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[1]])
            marker.points.append(mesh.vertices[mesh.triangles[j].vertex_indices[2]])
        # create the interactive marker
        name =  self.create_name(mesh_id)
        self.server.insert(self.create_im(marker, pose, name), self.process_feedback)
        self.server.setCallback(name, self.release, InteractiveMarkerFeedback.MOUSE_UP)
        self.server.applyChanges()
        
    def create_im(self, marker, pose, name):
        # create the new interactive marker
        int_marker = InteractiveMarker()
        int_marker.pose = copy.deepcopy(pose)
        int_marker.header.frame_id = 'base_link'
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
        return int_marker
        
    def process_feedback(self, feedback):
        self.last_feedback = feedback
        
    def check_pose(self, x, y):
        return x >= OFFSET + DEPTH_START and x <= OFFSET + DEPTH_END and y >= WIDTH_START and y <= WIDTH_END

    def release(self, feedback):
        im = self.server.get(feedback.marker_name)
        # copy the mesh information
        marker = copy.deepcopy(im.controls[0].markers[0])
        # determine color based on pose
        if self.check_pose(feedback.pose.position.x, feedback.pose.position.y):
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.66
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.66
        # create the new interactive marker
        self.server.insert(self.create_im(marker, feedback.pose, feedback.marker_name), self.process_feedback)
        self.server.setCallback(feedback.marker_name, self.release, InteractiveMarkerFeedback.MOUSE_UP)
        self.server.applyChanges()
        
    def update(self):
        self.server.applyChanges()
        
    def save(self, req):
        # go through each object and check if they are in the valid range
        to_save = []
        for obj_id in self.objects:
            im = self.server.get(self.create_name(obj_id))
            pose = im.pose
            if (self.check_pose(pose.position.x, pose.position.y)):
                to_save.append(copy.deepcopy(im))
        # check if we have anything to save
        if len(to_save) > 0:
            self.templates[req.name] = to_save
            # PICKLE IT!
            pickle.dump(self.templates, open(SAVE_FILE, 'wb'))
            return SaveTemplateResponse(True)
        else:
            return SaveTemplateResponse(False)

if __name__ == '__main__':    
    rospy.init_node('fake_object_markers')
    server = FakeMarkerServer()
    while not rospy.core.is_shutdown():
        server.update()
