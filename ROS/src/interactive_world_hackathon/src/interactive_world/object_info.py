import roslib
roslib.load_manifest('object_manipulator')
from object_manipulator.convert_functions import get_transform, pose_to_mat

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
