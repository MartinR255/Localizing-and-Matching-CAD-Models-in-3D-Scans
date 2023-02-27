import pybullet as pb
import pybullet_data
import time

class SceneBuilder:
    
    def __init__(self):
        self.physics_client = pb.connect(pb.GUI) # pb.DIRECT
        self.dt = pb.getPhysicsEngineParameters()['fixedTimeStep']
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.8)
        pb.setRealTimeSimulation(1)

        self.MESH_SCALE = [0.01, 0.01, 0.01]
        
        self.CAMERA_DISTANCE = 10
        self.CAMERA_YAW = 90#40
        self.CAMERA_PITCH = -25
        self.CAMERA_TARGET_POSITION = [0, 0, 0]
    #    pb.configureDebugVisualizer

        plane = pb.loadURDF("plane.urdf",[0, 0, 0])
    
    

    def build_bin_from_stl(self, path):
        shape_id = pb.createCollisionShape(
            shapeType = pb.GEOM_MESH,
            fileName = path, 
            flags = pb.URDF_INITIALIZE_SAT_FEATURES,
            meshScale = self.MESH_SCALE
        )

        viz_shape_id = pb.createVisualShape(
            shapeType = pb.GEOM_MESH,
            fileName = path,
            meshScale = self.MESH_SCALE
        )

        body_id = pb.createMultiBody(
            baseMass = 100.0,
            baseCollisionShapeIndex = shape_id,
            baseVisualShapeIndex = viz_shape_id,
            basePosition = (-2, 1.5, 0),
            baseOrientation = (0, 0, 0, 1),
        )

        pb.changeDynamics(
            body_id, -1, 
            mass = 100.0,
            restitution = 0.0,
            lateralFriction = 0.6,
            rollingFriction = 0.001, 
            spinningFriction = 0.001,
            )
        pb.changeVisualShape(body_id,-1,rgbaColor=[0.8,0.8,0.8,1])

   
    def build_object_from_stl(self, path): 
        shape_id = pb.createCollisionShape(
            shapeType = pb.GEOM_MESH,
            fileName = path, 
            flags = pb.URDF_INITIALIZE_SAT_FEATURES,
            meshScale = self.MESH_SCALE
        )

        viz_shape_id = pb.createVisualShape(
            shapeType = pb.GEOM_MESH,
            fileName = path,
            meshScale = self.MESH_SCALE
        )

        body_id = pb.createMultiBody(
            baseMass = 0.5,
            baseCollisionShapeIndex = shape_id,
            baseVisualShapeIndex = viz_shape_id,
            basePosition = (0, 0, 50),
            baseOrientation = (0, 0, 0, 1),
        )

        pb.changeDynamics(
            body_id, -1, 
            mass = 0.5,
            restitution = 0.0,
            lateralFriction = 0.6,
            rollingFriction = 0.001, 
            spinningFriction = 0.001,
            )
        pb.changeVisualShape(body_id,-1,rgbaColor=(0.5, 0.1, 0.8, 1))

    
    def run_gui(self):
        self.setup_camera()
        while pb.isConnected():
            pb.stepSimulation()
            time.sleep(self.dt)

    
    def setup_camera(self):
        pb.resetDebugVisualizerCamera(
                cameraDistance = self.CAMERA_DISTANCE, 
                cameraYaw = self.CAMERA_YAW, 
                cameraPitch = self.CAMERA_PITCH, 
                cameraTargetPosition = self.CAMERA_TARGET_POSITION
        )
         



