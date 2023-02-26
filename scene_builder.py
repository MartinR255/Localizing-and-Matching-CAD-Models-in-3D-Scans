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
        
        self.CAMERA_DISTANCE = 600
        self.CAMERA_YAW = 90#40
        self.CAMERA_PITCH = -25
       
        self.build_plane()
    
    
    def build_plane(self):
        for row in range(-1, 1+1, 1):
            for col in range(-1, 1+1, 1): 
                plane = pb.loadURDF("plane100.urdf",[200 * row,200*col,0])
                pb.changeVisualShape(plane,-1,rgbaColor=[0.1,0.1,0.1,1])


    def build_bin_from_stl(self, path):
        shape_id = pb.createCollisionShape(
            shapeType=pb.GEOM_MESH,
            fileName=path, 
            flags=pb.URDF_INITIALIZE_SAT_FEATURES
        )

        viz_shape_id = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName=path,
        )

        body_id = pb.createMultiBody(
            baseMass=1000,
            baseCollisionShapeIndex=shape_id,
            baseVisualShapeIndex=viz_shape_id,
            basePosition=(-200, 150, 0),
            baseOrientation=(0, 0, 0, 1),
        )

        # pb.changeDynamics(body_id,-1,linearDamping=0, angularDamping=0, rollingFriction=1.001, spinningFriction=0.001)
        pb.changeVisualShape(body_id,-1,rgbaColor=[0.8,0.8,0.8,1])


    def build_object_from_stl(self, path): 
        shape_id = pb.createCollisionShape(
            shapeType=pb.GEOM_MESH,
            fileName=path, 
            flags=pb.URDF_INITIALIZE_SAT_FEATURES
        )

        viz_shape_id = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName=path,
        )

        body_id = pb.createMultiBody(
            baseMass=2,
            baseCollisionShapeIndex=shape_id,
            baseVisualShapeIndex=viz_shape_id,
            basePosition=(0, 0, 150),
            baseOrientation=(0, 0, 0, 1),
        )

        # pb.changeDynamics(body_id,-1,linearDamping=0, angularDamping=0, rollingFriction=1.001, spinningFriction=0.001)
        pb.changeVisualShape(body_id,-1,rgbaColor=(0.5, 0.1, 0.8, 1))

    
    def run_gui(self):
        while pb.isConnected():
            pb.resetDebugVisualizerCamera(
                cameraDistance=self.CAMERA_DISTANCE, 
                cameraYaw=self.CAMERA_YAW, 
                cameraPitch=self.CAMERA_PITCH, 
                cameraTargetPosition=[0,0,0]
            )
            pb.stepSimulation()
            time.sleep(self.dt)
         



