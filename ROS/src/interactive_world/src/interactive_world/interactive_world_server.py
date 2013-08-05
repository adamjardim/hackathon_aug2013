import roslib
roslib.load_manifest('object_manipulation_msgs')
roslib.load_manifest('pr2_object_manipulation_msgs')
from geometry_msgs.msg import Pose, Polygon
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from object_manipulation_msgs.msg import PickupActionResult, PlaceAction, PlaceGoal, GripperTranslation
from manipulation_msgs.msg import GraspableObject, GraspableObjectList
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int32, Float32
import std_msgs
from pr2_object_manipulation_msgs.msg import IMGUIAction, IMGUIOptions, IMGUIGoal
from household_objects_database_msgs.msg import DatabaseModelPose
from object_info import ObjectInfo
from template import Template
from interactive_world.srv import GraspCheck
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from object_manipulator.convert_functions import change_pose_stamped_frame, pose_to_mat, mat_to_pose, stamp_pose, create_vector3_stamped
import tf
import rospy
import actionlib
from math import pi, sin, cos
import random
from sys import maxint
import copy

class InteractiveWorldServer():
    def __init__(self):
        # create the IM server
        self.server = InteractiveMarkerServer('interactive_world')
        # create a simple TF listener
        self.tf_listener = tf.TransformListener()
        # create the IMGUI action client
        self.imgui = actionlib.SimpleActionClient('imgui_action', IMGUIAction)
        self.imgui.wait_for_server()
        # create the place action client
        self.place = actionlib.SimpleActionClient('object_manipulator/object_manipulator_place', PlaceAction)
        self.place.wait_for_server()
        # hack to get the grasp
        rospy.Subscriber('/object_manipulator/object_manipulator_pickup/result', PickupActionResult, self.store_grasp)
        self.last_grasp = None
        # gets the regions
        rospy.Subscriber('/interactive_world/region_request', Polygon, self.store_region)
        self.last_region = None
        # listen for graspable objects
        rospy.Subscriber('/interactive_object_recognition_result', GraspableObjectList, self.proc_grasp_list)
        # change the modes
        rospy.Service('~change_mode', Empty, self.change_mode)
        self.edit_mode = False
        # grasp checker
        rospy.wait_for_service('/interactive_world/grasp_check')
        self.grasp_check = rospy.ServiceProxy('/interactive_world/grasp_check', GraspCheck)
        # save and load
        rospy.Service('~save', Empty, self.save)
        self.template_count = rospy.Publisher('~template_count', Int32)
        rospy.Subscriber('~load', Int32, self.load)
        self.table_height = rospy.Publisher('~table_height', Float32)
        self.init_region_request = rospy.Publisher('~init_region_request', std_msgs.msg.Empty)
        self.last_feedback = None
        self.object_info_list = dict()
        self.object_pose_list = dict()
        self.object_mesh_list = dict()
        self.im_name_list = []
        self.templates = []
        random.seed()
        
    def clear(self):
        # clear the old info
        for name in self.im_name_list:
            self.server.erase(name)
        self.server.clear()
        self.server.applyChanges()
        self.im_name_list = []
        self.object_info_list.clear()
        self.object_pose_list.clear()
        self.object_mesh_list.clear()
        
    def load(self, msg):
        # create the template markers
        template = self.templates[msg.data - 1]
        rospy.loginfo('Loading ' + template.name)
        for key in template.meshes.keys():
            obj = GraspableObject()
            obj.potential_models.append(DatabaseModelPose())
            obj.potential_models[0].pose.pose = template.poses[key]
            obj.potential_models[0].pose.header.frame_id = 'base_link'
            self.create_im(obj, template.meshes[key], 'template-' + key, 0.66, 0.2, 0.76)
        # now, do it!
        for key in template.meshes.keys():
            feedback = InteractiveMarkerFeedback()
            feedback.marker_name = key
            feedback.pose = template.poses[key]
            self.edit_mode = False
            self.release(feedback)
            self.edit_mode = True
    
    def change_mode(self, req):
        # change the mode of the server
        self.edit_mode = not self.edit_mode
        if self.edit_mode:
            rospy.loginfo('Edit mode enabled.')
        else:
            rospy.loginfo('Single object mode enabled.')
        return EmptyResponse()
    
    def save(self, req):
        name = 'Template ' + str(len(self.templates) + 1)
        template = Template(name, self.object_pose_list, self.object_mesh_list)
        self.templates.append(template)
        self.template_count.publish(len(self.templates))
        rospy.loginfo(name + ' created.')
        return EmptyResponse()

    def proc_grasp_list(self, msg):
        self.clear()
        # start by going through each
        size = len(msg.graspable_objects)
        for i in range(size):
            obj = msg.graspable_objects[i]
            # only take recognized objects
            if len(obj.potential_models) is not 0:
                name = str(obj.potential_models[0].model_id)
                self.create_im(obj, msg.meshes[i], name, 0.2, 0.66, 0.76)
                self.object_info_list[name] = self.create_object_info(obj)
                self.object_pose_list[name] = self.object_info_list[name].pose.pose
                self.object_mesh_list[name] = msg.meshes[i]
        rospy.loginfo('Found ' + str(len(self.object_info_list)) + ' new object(s).')

    def store_grasp(self, msg):
        self.last_grasp = msg.result.grasp
        
    def store_region(self, msg):
        self.last_region = msg

    def create_object_info(self, obj):
        # get the pose
        pose_stamped = obj.potential_models[0].pose
        # change the frame
        obj_frame_pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, obj.reference_frame_id)
        return ObjectInfo(obj, obj_frame_pose_stamped, self.tf_listener)

    def create_im(self, obj, mesh, name, r, g, b):
        # build the mesh marker
        marker = Marker()
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
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
        int_marker.pose = obj.potential_models[0].pose.pose
        int_marker.header.frame_id = obj.potential_models[0].pose.header.frame_id
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
        self.im_name_list.append(name)

    def process_feedback(self, feedback):
        self.last_feedback = feedback

    def release(self, feedback):
        # check the mode
        if self.edit_mode:
            # simply store the new pose
            self.object_pose_list[feedback.marker_name] = feedback.pose
        else:
            # publish the height of the object
            self.table_height.publish(feedback.pose.position.z)
            # get the object to deal with
            obj_info = self.object_info_list[feedback.marker_name]
            # pick it up
            arm_selection = self.pickup(obj_info)
            if arm_selection is not None:
                # make sure we have a grasp
                rospy.loginfo('Waiting for grasp...')
                while self.last_grasp is None:
                    rospy.sleep(1)
                # store the grasp
                obj_info.grasp = self.last_grasp
                self.last_grasp = None
                rospy.loginfo('Grasp found and recorded.')
                # move the arm to the side
                self.move_arm_to_side(arm_selection)
                # now try and place
                success = self.place_object(obj_info, arm_selection, feedback.pose)
                if success:
                    # move the arm back
                    self.move_arm_to_side(arm_selection)
                else:
                    # go into "region" mode
                    self.execute_region_place(obj_info, arm_selection, feedback.pose.position.z)
                # remove the old marker
                self.server.erase(feedback.marker_name)
                self.server.applyChanges()
        
    def pickup(self, obj_info):
        obj = obj_info.obj
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
        rospy.loginfo('Attempting to pick up...')
        self.reset_collision_map()
        self.imgui.send_goal(goal)
        self.imgui.wait_for_result()
        # check the result
        res = self.imgui.get_result()
        if res.result.value is not 1:
            # try the other arm
            if options.arm_selection is 0:
                options.arm_selection = 1
            else:
                options.arm_selection = 0
            rospy.loginfo('Initial pickup failed, trying other arm...')
            self.reset_collision_map()
            self.imgui.send_goal(goal)
            self.imgui.wait_for_result()
            # check the result
            res = self.imgui.get_result()
        if res.result.value is not 1:
            self.clear()
            rospy.logwarn('Pickup failed.')
            return None
        else:
            # now check if feedback to see if we actually got it
            if options.arm_selection is 0:
                arm = 'right'
            else:
                arm = 'left'
            resp = self.grasp_check(arm)
            if resp.isGrasping is True:
                rospy.loginfo('Pickup was successful.')
                return options.arm_selection
            else:
                self.move_arm_to_side(options.arm_selection)
                self.clear()
                rospy.logwarn('Pickup failed.')
                return None
        
    def move_arm_to_side(self, arm_selection):
        goal = IMGUIGoal()
        goal.command.command = 4
        goal.options.arm_selection = arm_selection
        goal.options.arm_action_choice = 0
        goal.options.arm_planner_choice = 1
        rospy.loginfo('Moving arm to the side using planner...')
        self.imgui.send_goal(goal)
        self.imgui.wait_for_result()
        # check the result
        res = self.imgui.get_result()
        if res.result.value is not 1:
            # try open loop
            rospy.loginfo('Planned arm move failed, trying open loop...')
            goal.options.arm_planner_choice = 0
            self.imgui.send_goal(goal)
            self.imgui.wait_for_result()
            # check the result
            res = self.imgui.get_result()
        if res.result.value is not 1:
            rospy.logwarn('Arm move failed.')
            return False
        else:
            rospy.loginfo('Arm move was successful.')
            return True
        
    def reset_collision_map(self):
        rospy.loginfo('Reseting collision map...')
        goal = IMGUIGoal()
        goal.command.command = 3
        goal.options.reset_choice = 4
        self.imgui.send_goal(goal)
        self.imgui.wait_for_result()
        rospy.loginfo('Collision map reset.')
        
    def place_object(self, obj_info_orig, arm_selection, pose):
        obj_info = copy.deepcopy(obj_info_orig)
        # reset the collision map
        self.reset_collision_map()
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
        # 'i' is for some rotations
        for i in range(0, 10):
            rads = (pi * (i/10.0))
            pose.orientation.z = sin(-rads/2.0)
            pose.orientation.w = cos(-rads/2.0);
            # 'j' is for the 'z' height
            for j in range (0, 5):
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
        rospy.loginfo('Attempting to place object...')
        self.place.send_goal(goal)
        # wait for result
        finished_within_time = self.place.wait_for_result(rospy.Duration(240))
        if not finished_within_time:
            self.place.cancel_goal()
            rospy.logwarn('Place timed out.')
            return False
        # check the result
        res = self.place.get_result()
        if res.manipulation_result.value == -6 or res.manipulation_result.value == 1:
            rospy.loginfo('Place was successful.')
            return True
        else:
            rospy.logwarn('Place failed with error code ' + str(res.manipulation_result.value) + '.')
            return False
        
    def execute_region_place(self, obj_info, arm_selection, z):
        # make sure we have a region
        self.init_region_request.publish()
        rospy.loginfo('Waiting for region...')
        while self.last_region is None:
            rospy.sleep(1)
        # store the region
        region = self.last_region
        min_x = maxint
        min_y = maxint
        max_x = -maxint
        max_y = -maxint
        # get the bounds (should be a rectangle)
        for i in range(0, 4):
            p = region.points[i]
            if p.x > max_x:
                max_x = p.x
            elif p.x < min_x:
                min_x = p.x
            if p.y > max_y:
                max_y = p.y
            elif p.y < min_y:
                min_y = p.y
        self.last_region = None
        rospy.loginfo('Region found and recorded.')
        # randomly sample the region
        for i in range(0, 20):
            pose = Pose()
            pose.position.x = random.uniform(min_x, max_x)
            pose.position.y = random.uniform(min_y, max_y)
            pose.position.z = z
            pose.orientation.w = 1.0
            rospy.loginfo('Attempting region place #' + str(i) + 
                          ' at (' + str(pose.position.x) + ', ' + str(pose.position.y) + ')...')
            # now try and place
            success = self.place_object(obj_info, arm_selection, pose)
            if success:
                # move the arm back and return
                self.move_arm_to_side(arm_selection)
                return

    def update(self):
        self.server.applyChanges()
