import pybullet as pb
import pybullet_data
import json
import time
from os import listdir
from os.path import isfile, join
from random import randrange


class SceneBuilder:
    
    def __init__(self):
        self.physics_client = pb.connect(pb.GUI) # pb.DIRECT
        self.dt = pb.getPhysicsEngineParameters()['fixedTimeStep']
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.8)
        pb.setRealTimeSimulation(1)
        self.load_config_files()
        self.load_paths()

        # camera and light setup
        self.setup_camera()
        self.setup_light()

        self.plane = pb.loadURDF("plane.urdf",[0, 0, 0])
        # self.cad_models_obj = set() 
        # self.collision_client = pb.connect(pb.DIRECT)
    

    def load_config_files(self):
        with open('../config/enviroment_config.json') as env_file:
            self.enviroment_setup = json.loads(env_file.read())

        with open('../config/physics_config.json') as physics_file:
            self.physics_setup = json.loads(physics_file.read())

    '''
    loads bin file path and cad models file paths
    '''
    def load_paths(self):
        self.bin_file_path = self.physics_setup['bin']['bin_small_file_path']
        cad_models_folder_path = self.physics_setup['cad_model']['folder_path']
        self.cad_models_file_paths = [join(cad_models_folder_path, f) for f in listdir(cad_models_folder_path) if isfile(join(cad_models_folder_path, f))]


    def setup_camera(self):
        camera_setup = self.enviroment_setup['camera']
        pb.resetDebugVisualizerCamera(
                cameraDistance = camera_setup['camera_distance'], 
                cameraYaw = camera_setup['camera_yaw'], 
                cameraPitch = camera_setup['camera_pitch'], 
                cameraTargetPosition = camera_setup['camera_target_position']
        )

    
    def setup_light(self):
        light_setup = self.enviroment_setup['light']
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_RENDERING, 1,
            lightPosition =light_setup['light_position']
        )
    

    def create_collision_shape(self, path, mehs_scale, trimesh=False):
        flag = pb.GEOM_FORCE_CONCAVE_TRIMESH if trimesh else pb.URDF_INITIALIZE_SAT_FEATURES
        return pb.createCollisionShape(
            shapeType = pb.GEOM_MESH,
            fileName = path, 
            flags = flag, # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=11962
            meshScale = mehs_scale
        )


    def create_visual_shape(self, path, mesh_scale):
        return pb.createVisualShape(
            shapeType = pb.GEOM_MESH,
            fileName = path,
            meshScale = mesh_scale
        )


    def create_multi_body(self, collision_shape_id, visual_shape_id, mass, base_position, base_orientation=(0, 0, 0, 1)):
        body_id = pb.createMultiBody(
            baseMass = mass,
            baseCollisionShapeIndex = collision_shape_id,
            baseVisualShapeIndex = visual_shape_id,
            basePosition = base_position,
            baseOrientation = base_orientation
        )

        self.setup_object_dynamics(body_id, mass) 
        return body_id
    

    def setup_object_dynamics(self, body_id, mass):
        dynamics_setup = self.physics_setup['dynamics']
        pb.changeDynamics(
        body_id, -1, 
        mass = mass,
        restitution = dynamics_setup['restitution'],
        lateralFriction = dynamics_setup['lateral_friction'],
        rollingFriction = dynamics_setup['rolling_friction'], 
        spinningFriction = dynamics_setup['spinning_friction'],
        )
    
   
    
    '''
    builds bin model object from STL file
    '''
    def build_bin(self, path):
        bin_conf = self.physics_setup['bin']
        collision_shape = self.create_collision_shape(path, bin_conf['mesh_scale'], True)
        visual_shape = self.create_visual_shape(path, bin_conf['mesh_scale'])
        multi_body = self.create_multi_body(visual_shape, collision_shape, bin_conf['mass'], (-2, 1.5, 0))

        pb.changeVisualShape(multi_body, -1, rgbaColor=[0.8,0.8,0.8,1])


    def get_random_object_position(self):
        x = randrange(-50, 50) * 0.02
        y = randrange(-50, 50) * 0.02
        z = randrange(25, 50)

        return x, y, z


    '''
    builds CAD model object from STL file
    '''
    def build_cad_model(self, path): 
        cad_model_conf = self.physics_setup['cad_model'] 
        collision_shape = self.create_collision_shape(path, cad_model_conf['mesh_scale'])
        visual_shape = self.create_visual_shape(path, cad_model_conf['mesh_scale'])

        position = self.get_random_object_position()
        multi_body = self.create_multi_body(visual_shape, collision_shape, cad_model_conf['mass'], position)

        pb.changeVisualShape(multi_body, -1, rgbaColor=(0.5, 0.1, 0.8, 1))


    def build_scene(self):
        self.build_bin(self.bin_file_path)
        for _ in range(50):
            body_id = self.build_cad_model(self.cad_models_file_paths[0])


    def run(self):
        while pb.isConnected():
            pb.stepSimulation()
            time.sleep(self.dt)




    
    
         



