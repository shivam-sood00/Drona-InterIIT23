import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from cv2 import aruco
import time

calib_data_path = "calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)


cam_mat = calib_data["camMatrix"]
print(cam_mat)
# cam_mat = np.array([[1360.2626953125, 0, 974.640075683594],[0, 1361.03882835938, 549.4236767578125],[0,0,1]])

dist_coef = calib_data["distCoef"]
print(dist_coef)

dist_coef = np.zeros((5, 1))
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]


MARKER_SIZE = 3.375  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()

pipeline = rs.pipeline()
res_cols = 1280 #640
res_rows = 720 #480
config = rs.config()
# config.enable_stream(rs.stream.depth, res_cols, res_rows, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90) # max 1280 800 30 fps or 848 480 90

config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)

x = 0
y = 0

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
clipping_distance_in_meters = 10 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale


# used to record the time when we processed last frame
prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0

align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue


        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        frame = color_image

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        x = x + 1

        
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )


        if marker_corners:
            y = y + 1
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

                # pass point as (x, y) in get_distance()
                zDepth = aligned_depth_frame.get_distance(int(mid[0]), int(mid[1]))
                print(zDepth)
                print('this is bottom left', mid)


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
                    f"id: {ids[0]} Dist: {round(tVec[i][0][2], 2)} Dist rs: {round(zDepth, 2)}",
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

            new_frame_time = time.time()

            # Calculating the fps

            # fps will be number of frame processed in given time frame
            # since their will be most of time error of 0.001 second
            # we will be subtracting it to get more accurate result
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            print("fps = ", fps)

        print(depth_image.shape)
        print(color_image.dtype)

        cv.namedWindow('Align Example', cv.WINDOW_AUTOSIZE)

        cv.imshow("frame", frame)
        
        key = cv.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv.destroyAllWindows()
            break
finally:
    pipeline.stop()