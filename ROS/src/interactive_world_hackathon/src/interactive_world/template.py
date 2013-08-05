class Template():
    def __init__(self, name, pose_list, meshes):
        self.name = name
        self.poses = pose_list.copy()
        self.meshes = meshes.copy()
