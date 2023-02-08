from vision_pipeline import VisionPipeline

camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=[8, 6],debug=1,padding = 0,debug_calib=True)
try:
    while True:
        camera.calib_drone1()
except KeyboardInterrupt:
    pass

while True:
    camera.calibrate_yaw()
