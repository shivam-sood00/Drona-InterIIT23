import sys
import pyzed.sl as sl
import numpy as np
import cv2 as cv
from cv2 import aruco
import time

help_string = "[s] Save side by side image [d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit"
prefix_point_cloud = "Cloud_"
prefix_depth = "Depth_"
path = "./"

count_save = 0
mode_point_cloud = 0
mode_depth = 0
point_cloud_format_ext = ".ply"
depth_format_ext = ".png"

########################################

calib_data_path = "/home/pranjal/INTER_IIT_23/Drona-InterIIT23/vision/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
# print(calib_data.files)


cam_mat = np.load("vision/calib_data/camMatrix.npy")
# print(cam_mat)
# cam_mat = np.array([[1360.2626953125, 0, 974.640075683594],[0, 1361.03882835938, 549.4236767578125],[0,0,1]])

dist_coef = np.load("vision/calib_data/distCoef.npy")
# print(dist_coef)

dist_coef = np.zeros((5, 1))
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]


MARKER_SIZE = 3.9  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()


##########################################

prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0



# def point_cloud_format_name(): 
#     global mode_point_cloud
#     if mode_point_cloud > 3:
#         mode_point_cloud = 0
#     switcher = {
#         0: ".xyz",
#         1: ".pcd",
#         2: ".ply",
#         3: ".vtk",
#     }
#     return switcher.get(mode_point_cloud, "nothing") 
  
# def depth_format_name(): 
#     global mode_depth
#     if mode_depth > 2:
#         mode_depth = 0
#     switcher = {
#         0: ".png",
#         1: ".pfm",
#         2: ".pgm",
#     }
#     return switcher.get(mode_depth, "nothing") 

# def save_point_cloud(zed, filename) :
#     print("Saving Point Cloud...")
#     tmp = sl.Mat()
#     zed.retrieve_measure(tmp, sl.MEASURE.XYZRGBA)
#     saved = (tmp.write(filename + point_cloud_format_ext) == sl.ERROR_CODE.SUCCESS)
#     if saved :
#         print("Done")
#     else :
#         print("Failed... Please check that you have permissions to write on disk")

# def save_depth(zed, filename) :
#     print("Saving Depth Map...")
#     tmp = sl.Mat()
#     zed.retrieve_measure(tmp, sl.MEASURE.DEPTH)
#     saved = (tmp.write(filename + depth_format_ext) == sl.ERROR_CODE.SUCCESS)
#     if saved :
#         print("Done")
#     else :
#         print("Failed... Please check that you have permissions to write on disk")

# def save_sbs_image(zed, filename) :

#     image_sl_left = sl.Mat()
#     zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
#     image_cv_left = image_sl_left.get_data()

#     image_sl_right = sl.Mat()
#     zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
#     image_cv_right = image_sl_right.get_data()

#     sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)

#     cv.imwrite(filename, sbs_image)
    
# def process_key_event(zed, key) :
#     global mode_depth
#     global mode_point_cloud
#     global count_save
#     global depth_format_ext
#     global point_cloud_format_ext

#     if key == 100 or key == 68:
#         save_depth(zed, path + prefix_depth + str(count_save))
#         count_save += 1
#     elif key == 110 or key == 78:
#         mode_depth += 1
#         depth_format_ext = depth_format_name()
#         print("Depth format: ", depth_format_ext)
#     elif key == 112 or key == 80:
#         save_point_cloud(zed, path + prefix_point_cloud + str(count_save))
#         count_save += 1
#     elif key == 109 or key == 77:
#         mode_point_cloud += 1
#         point_cloud_format_ext = point_cloud_format_name()
#         print("Point Cloud format: ", point_cloud_format_ext)
#     elif key == 104 or key == 72:
#         print(help_string)
#     elif key == 115:
#         save_sbs_image(zed, "ZED_image" + str(count_save) + ".png")
#         count_save += 1
#     else:
#         a = 0

# def print_help() :
#     print(" Press 's' to save Side by side images")
#     print(" Press 'p' to save Point Cloud")
#     print(" Press 'd' to save Depth image")
#     print(" Press 'm' to switch Point Cloud format")
#     print(" Press 'n' to switch Depth format")


def main() :

    # Create a ZED camera object
    zed = sl.Camera()
    
    # Set configuration parameters
    input_type = sl.InputType()
    if len(sys.argv) >= 2 :
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    init.depth_minimum_distance = 0.1
    init.depth_maximum_distance = 2
    init.coordinate_units = sl.UNIT.METER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)
    # zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
    # # zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 100)
    # # zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITE_BALANCE, 4600)
    # zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 8)
    # zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, 0)
    # # zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 50)
    # zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO , 0)
    cam_info = zed.get_camera_information()
    # tracking_parameters = sl.PositionalTrackingParameters()
    # tracking_parameters.set_as_static = False
    # err = zed.enable_positional_tracking(tracking_parameters)
    print(cam_info.calibration_parameters_raw.left_cam.fx)
    print(cam_info.calibration_parameters_raw.left_cam.fy)
    print(cam_info.calibration_parameters_raw.left_cam.cx)
    print(cam_info.calibration_parameters_raw.left_cam.cy)
    print(cam_info.calibration_parameters_raw.left_cam.disto)

    # print(cam_info.calibration_parameters.left_cam())
    # print(cam_info.calibration_parameters.right_cam())

    # Display help in console
    # print_help()

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    # image_size.width = 4416
    # image_size.height = 1242
# 4416x1242
    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height)
    depth_image_zed2 = sl.Mat(image_size.width, image_size.height)
    print("NEW:", image_size.width, image_size.height)
    point_cloud = sl.Mat()

    key = ' '
    while key != 113 :
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image, depth image in the half-resolution
            # print(type(image_zed), type(depth_image_zed))
            zed.retrieve_image(image_zed, sl.VIEW.RIGHT, sl.MEM.CPU, image_size)
            zed.retrieve_measure(depth_image_zed, sl.MEASURE.DEPTH, sl.MEM.CPU)
            zed.retrieve_image(depth_image_zed2, sl.VIEW.DEPTH, sl.MEM.CPU)
            # Retrieve the RGBA point cloud in half resolution
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            color_image = image_zed.get_data()
            depth_image = depth_image_zed2.get_data()
            ##################################################

            frame = color_image

            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  

            
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, marker_dict, parameters=param_markers
            )

            # z_depth = depth_image_zed.get_value(image_size.width/2, image_size.height/2)
            # print(z_depth)
            
            if marker_corners:
  
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, cam_mat, dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()


                    mid = [(top_left[0]+top_right[0]+bottom_left[0]+bottom_right[0])/4,(top_left[1]+top_right[1]+bottom_left[1]+bottom_right[1])/4]

                    # pass point as (x, y) in get_value()
                    zDepth = depth_image_zed.get_value(int(mid[0]), int(mid[1]))
                    print("depth:", zDepth)
                    # try:
                    #     x = point3D[0]
                    #     y = point3D[1]
                    #     # z = point3D[2]
                    #     # color = point3D[3]
                    #     print("x: ", x, "y: ")#, y, "z: ", z)
                    # finally:
                    #     pass
                    # print('this is bottom left', mid)


                    # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                    # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                    # Calculating the distance
                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )
                    # Draw the pose of the marker
                    point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                    # cv.putText(
                    #     frame,
                    #     f"id: {ids[0]} Dist: {round(distance, 2)}",
                    #     top_right,
                    #     cv.FONT_HERSHEY_PLAIN,
                    #     1.3,
                    #     (0, 0, 255),
                    #     2,
                    #     cv.LINE_AA,
                    # )
                    cv.putText(
                        frame,
                        f"id: {ids[0]} Dist: {tVec[i][0][2]} Dist rs: {zDepth, 2}",
                        top_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )
                    cv.putText(
                        frame,
                        f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} ",
                        bottom_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )
                    # print(ids, "  ", corners)
                    print("Rotation Vector = ", rVec, ""
                                                        ""
                                                        ""
                                                        "")
                    print("Translation Vector = ", tVec, ""
                                                            ""
                                                            ""
                                                            "")
                    print("Camera Resolution - ", frame.shape)

                global new_frame_time, prev_frame_time
                new_frame_time = time.time()

                # Calculating the fps

                # fps will be number of frame processed in given time frame
                # since their will be most of time error of 0.001 second
                # we will be subtracting it to get more accurate result
                fps = 1 / (new_frame_time - prev_frame_time)
                prev_frame_time = new_frame_time
                print("fps = ", fps)

            # print(depth_image.shape)
            # print(color_image.dtype)

            # cv.namedWindow('Align Example', cv.WINDOW_AUTOSIZE)

            cv.imshow("RGB", frame)
            cv.imshow("Depth", depth_image)
            
            key = cv.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv.destroyAllWindows()
                break

            ##################################################
            


            # process_key_event(zed, key)

    cv.destroyAllWindows()
    zed.close()

    print("\nFINISH")

if __name__ == "__main__":
    main()
