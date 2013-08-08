#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('pr2_interactive_object_detection')
roslib.load_manifest('object_manipulation_msgs')
roslib.load_manifest('pr2_object_manipulation_msgs')
roslib.load_manifest('object_manipulator')
roslib.load_manifest('retrieve_medicine')
from retrieve_medicine.msg import navigateGoal, navigateAction, BackupGoal, BackupAction
from object_manipulator.convert_functions import get_transform, pose_to_mat
from household_objects_database_msgs.srv import GetModelMesh
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_world_hackathon.srv import SaveTemplate, SaveTemplateResponse
from interactive_world_hackathon.srv import PrintTemplates, PrintTemplatesResponse
from pr2_object_manipulation_msgs.msg import IMGUIAction, IMGUIOptions, IMGUIGoal
import copy
import pickle
import actionlib
from interactive_world_hackathon.msg import LoadAction, LoadFeedback, LoadResult
from interactive_world_hackathon.srv import GraspCheck, Speak
import time
from pr2_interactive_object_detection.msg import UserCommandAction, UserCommandGoal
from manipulation_msgs.msg import GraspableObject, GraspableObjectList
from object_manipulator.convert_functions import change_pose_stamped_frame, pose_to_mat, mat_to_pose, stamp_pose, create_vector3_stamped
from object_manipulation_msgs.msg import PickupActionResult, PlaceAction, PlaceGoal, GripperTranslation
from math import pi, sin, cos
import tf

#TABLE_HEIGHT = 0.92
PLACE_HEIGHT_OFFSET = 0.04
TABLE_HEIGHT = 0.75
OFFSET = 0.381
DEPTH_START = 0.0254
DEPTH_END = 0.381
WIDTH_START = -0.3175
WIDTH_END = 0.3175

OBJECT1 = 18808 # Campbell's travel soup cup
OBJECT2 = 18744 # Soda can
OBJECT3 = 18799 # Soup can

SAVE_FILE = '/tmp/templates.r0b0t'

class FakeMarkerServer():
    def __init__(self):
        # create a simple TF listener
        self.tf_listener = tf.TransformListener()
        self.grasp_check = rospy.ServiceProxy('/interactive_world_hackathon/grasp_check', GraspCheck)
        self.speak = rospy.ServiceProxy('/tts/speak', Speak)
        self.play = rospy.ServiceProxy('/tts/play', Speak)
        # create the nav client
        self.nav = actionlib.SimpleActionClient('navigate_action', navigateAction)
        self.nav.wait_for_server()
        # create the backup client
        self.backup = actionlib.SimpleActionClient('backup_action', BackupAction)
        self.backup.wait_for_server()
        # create the place action client
        self.place = actionlib.SimpleActionClient('object_manipulator/object_manipulator_place', PlaceAction)
        self.place.wait_for_server()
        # Segmentation client
        self.segclient = actionlib.SimpleActionClient('/object_detection_user_command', UserCommandAction)
        self.segclient.wait_for_server()
        self.recognition=None
        # create the IMGUI action client
        self.imgui = actionlib.SimpleActionClient('imgui_action', IMGUIAction)
        self.imgui.wait_for_server()
        # listen for graspable objects
        rospy.Subscriber('/interactive_object_recognition_result', GraspableObjectList, self.proc_grasp_list)
        # create the save service
        rospy.Service('~save_template', SaveTemplate, self.save)
        self.load_server = actionlib.SimpleActionServer('load_template', LoadAction, execute_cb=self.load, auto_start=False)
        self.load_server.start()
        # create the IM server
        self.server = InteractiveMarkerServer('~fake_marker_server')
        # create return list of templates
        rospy.Service('~print_templates', PrintTemplates, self.get_templates)
        # used to get model meshes
        self.get_mesh = rospy.ServiceProxy('/objects_database_node/get_model_mesh', GetModelMesh)
        # hack to get the grasp
        rospy.Subscriber('/object_manipulator/object_manipulator_pickup/result', PickupActionResult, self.store_grasp)
        self.last_grasp = None
        self.objects = []
        self.objects.append(OBJECT1)
        self.objects.append(OBJECT2)
        self.objects.append(OBJECT3)
        self.reset_objects()
        # check for saved templates
        try:
            self.templates = pickle.load(open(SAVE_FILE, 'rb'))
            rospy.loginfo('Loaded ' + str(len(self.templates.keys())) + ' template(s).')
        except:
            self.templates = dict()
            rospy.loginfo('New template file started.')
        self.play('/home/rctoris/wav/GLaDOS_generic_security_camera_destroyed-2.wav')

    def get_templates(self, req):
        temp_list = []
        if self.templates.keys() is None:
            self.publish_feedback('No templates')
            return
        for obj in self.templates.keys():
            temp_list.append(str(obj))
        #print temp_list
        return PrintTemplatesResponse(temp_list)
        
    def store_grasp(self, msg):
        self.last_grasp = msg.result.grasp
     
    # Given a mesh_id creates a name with format 'object + mesh_id' 
    # ex.)Given '1234', creates 'object_1234' name
    def create_name(self, mesh_id):
        return 'object_' + str(mesh_id)
       
    # Creates a mesh of the given object with the given pose to be visualized by template maker 
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
    
    # Creates an interactive marker 
    # - at the given location and pose 
    # - with a given name 
    # - for given marker object        
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
        
    # Returns true if given (x,y) coordinates are within "Graspable/Placeable(?)" range
    def check_pose(self, x, y):
        return x >= OFFSET + DEPTH_START and x <= OFFSET + DEPTH_END and y >= WIDTH_START and y <= WIDTH_END

    # Checks position of hallucinated interactive markers
    # Changes color and sets position when user releases mouse click (MOUSE_UP) on object
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
    
    # updates server    
    def update(self):
        self.server.applyChanges()
      
    # **Run by save_template service**  
    # Saves given template information to file location
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
            self.play('/home/rctoris/wav/t3_affirmative.wav')
            self.reset_objects()
            return SaveTemplateResponse(True)
        else:
            return SaveTemplateResponse(False)

    # Publishes feedback of current tasks
    def publish_feedback(self, msg):
        rospy.loginfo(msg)
        self.load_server.publish_feedback(LoadFeedback(msg))

    # Publishes final result of action
    def publish_result(self, msg):
        rospy.loginfo(msg)
        self.load_server.set_succeeded(LoadResult(msg))

    # Returns how many objects were recognized
    def proc_grasp_list(self, msg):
        objects = []
        # start by going through each
        size = len(msg.graspable_objects)
        for i in range(size):
            obj = msg.graspable_objects[i]
            # only take recognized objects
            if len(obj.potential_models) is not 0:
                objects.append(copy.deepcopy(obj))
        rospy.loginfo('Found ' + str(len(objects)) + ' object(s).')
        self.recognition = objects

    # Drives to and aligns with counter
    # Segments objects
    def look_for_objects(self):
        self.publish_feedback('Driving robot to counter')
        # drive the robot
        nav_goal = navigateGoal('Snack Nav', True)
        self.nav.send_goal_and_wait(nav_goal)
        res = self.nav.get_result()
        if not res.success:
            self.publish_feedback('Counter alignment failed.')
            return False
        self.publish_feedback('Aligned robot to counter')
        self.publish_feedback('Looking for objects')
        self.recognition = None
        # Segment the table
        self.segclient.send_goal(UserCommandGoal(request=1,interactive=False))
        self.segclient.wait_for_result()
        while self.recognition is None:
            time.sleep(1)
        # Recognize objects
        self.recognition = None
        self.segclient.send_goal(UserCommandGoal(request=2,interactive=False))
        self.segclient.wait_for_result()
        while self.recognition is None:
            time.sleep(1)
        return True

    # **Run by load_template service**
    # Identifies remaining objects needed in template
    # Moves to and aligns with counter
    # Scans and recognizes objects on counter that match template
    # Picks up one object
    # Backs up from counter
    # Drives to table
    # Places object in given template location
    # Repeats
    def load(self, goal):
        name = goal.name
        self.publish_feedback('Loading template ' + name)
        # if requested template does not exist...
        if name not in self.templates.keys():
            self.publish_result(name + ' template does not exist')
            return
        template = copy.deepcopy(self.templates[name])
        self.publish_feedback('Loaded template ' + name)
        self.play('/home/rctoris/wav/help.wav')
        # look for any objects we need
        while len(template) is not 0:
            pickup_arm = None
            # if it does not see any objects/could not drive to counter
            if not self.look_for_objects():
                self.publish_result('Object looking failed.')
                return
            # for each object in template array...
            for template_im in template:
                # for each recognized object
                for rec_obj in self.recognition:
                    if template_im.name == self.create_name(rec_obj.potential_models[0].model_id):
                        # create the object info for it
                        obj_info = self.create_object_info(rec_obj)
                        # pick it up
                        pickup_arm = self.pickup(rec_obj)
                        # if neither arm can could pick up object...
                        if pickup_arm is None:
                            self.publish_result('Pickup failed.')
                            return
                        # make sure we have a grasp
                        self.publish_feedback('Waiting for grasp')
                        while self.last_grasp is None:
                            rospy.sleep(1)
                        # store the grasp
                        obj_info.grasp = self.last_grasp
                        self.last_grasp = None
                        self.publish_feedback('Grasp found')
                        # good job robot, place that object
                        to_place = Pose()
                        # location of object in template on table
                        to_place.position.z = TABLE_HEIGHT - PLACE_HEIGHT_OFFSET
                        to_place.position.x = template_im.pose.position.x
                        to_place.position.y = template_im.pose.position.y
                        placed = self.place_object(obj_info, pickup_arm, to_place)
                        # if the object could not be placed
                        if not placed:
                            self.publish_result('Place failed.')
                            return
                        self.publish_feedback('Placed the object!')
                        if len(template) is not 1:
                            self.play('/home/rctoris/wav/ill-be-back.wav')
                        # removes object from list of objects to pick up from template
                        template.remove(template_im)
            # if no objects are found...
            if pickup_arm is None:
                # No objects found :(
                self.publish_result('No objects found that we need :(')
                return
        # We completed the task!
        self.play('/home/rctoris/wav/down.wav')
        self.publish_result('Great success!')
        
    # resets collision map of world and rescan
    def reset_collision_map(self):
        self.publish_feedback('Reseting collision map')
        goal = IMGUIGoal()
        goal.command.command = 3
        goal.options.reset_choice = 4
        self.imgui.send_goal(goal)
        self.imgui.wait_for_result()
        self.publish_feedback('Collision map reset')
    
    # reset hallucinated interactive marker objects' positions on visualized table
    def reset_objects(self):
        pose = Pose()
        pose.position.z = TABLE_HEIGHT
        pose.position.x = OFFSET * 3
        pose.position.y = -0.25
        for obj_id in self.objects:
            self.create_mesh(obj_id, pose)
            pose.position.y = pose.position.y + 0.25
    
    # Picks up object that matches obj   
    def pickup(self, obj):
        # start by picking up the object
        options = IMGUIOptions()
        options.collision_checked = True
        options.grasp_selection = 1    
        options.adv_options.lift_steps = 10
        options.adv_options.retreat_steps = 10
        options.adv_options.reactive_force = False
        options.adv_options.reactive_grasping = False
        options.adv_options.reactive_place = False
        options.adv_options.lift_direction_choice = 0
        # check which arm is closer
        if obj.potential_models[0].pose.pose.position.y > 0:
            options.arm_selection = 1
        else:
            options.arm_selection = 0
        goal = IMGUIGoal()
        goal.options = options
        goal.options.grasp_selection = 1
        goal.options.selected_object = obj
        goal.command.command = goal.command.PICKUP
        # send it to IMGUI
        self.publish_feedback('Attempting to pick up')
        self.reset_collision_map()
        self.imgui.send_goal(goal)
        self.play('/home/rctoris/wav/humnbehv.wav')
        self.imgui.wait_for_result()
        # check the result
        res = self.imgui.get_result()
        if res.result.value is not 1:
            # try the other arm
            if options.arm_selection is 0:
                options.arm_selection = 1
            else:
                options.arm_selection = 0
            self.publish_feedback('Initial pickup failed, trying other arm')
            self.reset_collision_map()
            self.imgui.send_goal(goal)
            self.imgui.wait_for_result()
            # check the result
            res = self.imgui.get_result()
        if res.result.value is not 1:
            return None
        else:
            # now check if feedback to see if we actually got it
            if options.arm_selection is 0:
                arm = 'right'
            else:
                arm = 'left'
            self.publish_feedback('Checking if object was grasped')
            resp = self.grasp_check(arm)
            if resp.isGrasping is True:
                self.publish_feedback('Object was grasped')
                # attempt to back up
                backup_goal = BackupGoal()
                self.backup.send_goal_and_wait(backup_goal)
                res = self.backup.get_result()
                # if robot could not back up
                if not res.success:
                    self.publish_feedback('Backup failed.')
                    return None
                return options.arm_selection
            else:
                self.move_arm_to_side(options.arm_selection)
                return None
    
    # moves arm to sides        
    def move_arm_to_side(self, arm_selection):
        goal = IMGUIGoal()
        goal.command.command = 4
        goal.options.arm_selection = arm_selection
        goal.options.arm_action_choice = 0
        goal.options.arm_planner_choice = 1
        self.publish_feedback('Moving arm to the side using planner')
        self.imgui.send_goal(goal)
        self.imgui.wait_for_result()
        # check the result
        res = self.imgui.get_result()
        if res.result.value is not 1:
            # try open loop
            self.publish_feedback('Planned arm move failed, trying open loop')
            goal.options.arm_planner_choice = 0
            self.imgui.send_goal(goal)
            self.imgui.wait_for_result()
            # check the result
            res = self.imgui.get_result()
        if res.result.value is not 1:
            self.publish_feedback('Arm move failed.')
            return False
        else:
            self.publish_feedback('Arm move was successful')
            return True
      
    # place object in given arm to given pose  
    def place_object(self, obj_info_orig, arm_selection, pose):
        #drive to the table
        self.publish_feedback('Driving robot to table')
        nav_goal = navigateGoal('Dining Table Nav', True)
        self.play('/home/rctoris/wav/run.wav')
        self.nav.send_goal_and_wait(nav_goal)
        res = self.nav.get_result()
        if not res.success:
            self.publish_feedback('Table alignment failed.')
            return False
        self.publish_feedback('Aligned robot to table')
        self.reset_collision_map()
        self.publish_feedback('Attempting to place the object')
        # make a copy
        obj_info = copy.deepcopy(obj_info_orig)
        obj = obj_info.obj
        goal = PlaceGoal()
        # set the arm
        if arm_selection is 0:  
            goal.arm_name = 'right_arm'
        else:
            goal.arm_name = 'left_arm'
        # rotate and "gu-chunk"
        orig_z = pose.position.z
        pose.orientation.x = 0
        pose.orientation.y = 0
        goal.place_locations = []
        #iterating through possible x-locations to place object
        for x in range(0, 10):
            pose.position.x = pose.position.x + ((x - 5) * 0.0025)
            #iterating through possible y-locations to place object
            for y in range(0, 10):
                pose.position.y = pose.position.y + ((y - 5) * 0.0025)
                # 'i' is for some rotations
                for i in range(0, 10):
                    rads = (pi * (i/10.0))
                    pose.orientation.z = sin(-rads/2.0)
                    pose.orientation.w = cos(-rads/2.0);
                    # 'j' is for the 'z' height
                    for j in range (0, 6):
                        pose.position.z = orig_z + (j * 0.0025)
                        pose_mat = pose_to_mat(pose)
                        to_base_link_mat = pose_mat * obj_info.obj_origin_to_bounding_box
                        grasp_mat = pose_to_mat(obj_info.grasp.grasp_pose.pose)
                        gripper_mat = to_base_link_mat * grasp_mat
                        gripper_pose = stamp_pose(mat_to_pose(gripper_mat), 'base_link')
                        goal.place_locations.append(gripper_pose)
        # use the identity as the grasp
        obj_info.grasp.grasp_pose.pose = Pose()
        obj_info.grasp.grasp_pose.pose.orientation.w = 1
        goal.grasp = obj_info.grasp
        goal.desired_retreat_distance = 0.1
        goal.min_retreat_distance = 0.05
        # set the approach
        goal.approach = GripperTranslation()
        goal.approach.desired_distance = .1
        goal.approach.min_distance = 0.05
        goal.approach.direction = create_vector3_stamped([0. , 0. , -1.], 'base_link')
        # set the collision info
        goal.collision_object_name = obj.collision_name
        goal.collision_support_surface_name = 'table'
        goal.place_padding = 0.02
        goal.use_reactive_place = False
        # send the goal
        self.place.send_goal(goal)
        # wait for result
        finished_within_time = self.place.wait_for_result(rospy.Duration(240))
        if not finished_within_time:
            self.place.cancel_goal()
            return False
        # check the result
        res = self.place.get_result()
        if res.manipulation_result.value == -6 or res.manipulation_result.value == 1:
            # attempt to back up
            backup_goal = BackupGoal()
            self.backup.send_goal_and_wait(backup_goal)
            backup_res = self.backup.get_result()
            # if robot could not back up
            if not backup_res.success:
                self.publish_feedback('Backup failed.')
                return False            
            self.move_arm_to_side(arm_selection)
            return True
        else:
            return False

    def create_object_info(self, obj):
        # get the pose
        pose_stamped = obj.potential_models[0].pose
        # change the frame
        obj_frame_pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, obj.reference_frame_id)
        return ObjectInfo(obj, obj_frame_pose_stamped, self.tf_listener)
        
        
class ObjectInfo():
    def __init__(self, obj, pose, tf_listener):
        # the original GraspableObject message
        self.obj = obj
        # the original pose on the table when detected
        self.pose = pose    
        # the Grasp object returned by the Pickup service
        self.grasp = None
        # where it was grasped, once it is grasped
        self.grasp_pose = None
        # convert to base link pose
        to_base_link = get_transform(tf_listener, obj.cluster.header.frame_id, 'base_link')
        pose_mat = pose_to_mat(pose.pose)
        # object to bounding box transform
        self.obj_origin_to_bounding_box = pose_mat**-1 * to_base_link    

if __name__ == '__main__':    
    rospy.init_node('fake_object_markers')
    server = FakeMarkerServer()
    while not rospy.core.is_shutdown():
        server.update()

