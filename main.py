from scene_builder import SceneBuilder
import time

if __name__ == "__main__":
    sb = SceneBuilder()
    # sb.build_object_from_stl('../stl_dataset/wheel.stl') 
    sb.build_bin_from_stl('../bins/bin_small.stl')
    sb.build_object_from_stl('../stl_dataset/testExport.stl')

    sb.run_gui()
    
    

