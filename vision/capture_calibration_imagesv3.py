import cv2 as cv
import os

CHESS_BOARD_DIM = (9, 7)

n = 0  # image_counter

# checking if  images dir is exist not, if not then create images directory
image_dir_path = "images"

CHECK_DIR = os.path.isdir(image_dir_path)
# if directory does not exist create
if not CHECK_DIR:
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already Exists.')

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret == True:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret



HIGH_VALUE = 10000
WIDTH = HIGH_VALUE
HEIGHT = HIGH_VALUE

import pyzed.sl as sl
import sys

# cap = cv.VideoCapture(6)
# # fourcc = cv.VideoWriter_fourcc(*'XVID')
# #
# cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
# cap.set(cv.CAP_PROP_FPS, 30)
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
# height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# print(width,height)

zed = sl.Camera()
input_type = sl.InputType()
if len(sys.argv) >= 2 :
    input_type.set_from_svo_file(sys.argv[1])
init = sl.InitParameters(input_t=input_type)
init.camera_resolution = sl.RESOLUTION.HD2K
err = zed.open(init)
if err != sl.ERROR_CODE.SUCCESS :
    print(repr(err))
    zed.close()
    exit(1)

image_zed = sl.Mat()
runtime = sl.RuntimeParameters()
runtime.sensing_mode = sl.SENSING_MODE.STANDARD

while True:
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS :
        # Retrieve the left image, depth image in the half-resolution
        # print(type(image_zed), type(depth_image_zed))
        zed.retrieve_image(image_zed, sl.VIEW.RIGHT, sl.MEM.CPU)
    frame = image_zed.get_data()
    copyFrame = frame.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    image, board_detected = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)
    # print(ret)
    cv.putText(
        frame,
        f"saved_img : {n}",
        (30, 40),
        cv.FONT_HERSHEY_PLAIN,
        1.4,
        (0, 255, 0),
        2,
        cv.LINE_AA,
    )

    cv.imshow("frame", frame)
    cv.imshow("copyFrame", copyFrame)

    key = cv.waitKey(1)

    if key == ord("q"):
        break
    if key == ord("s") and board_detected == True:
        # storing the checker board image
        cv.imwrite(f"{image_dir_path}/image{n}.png", copyFrame)

        print(f"saved image number {n}")
        n += 1  # incrementing the image counter
# cap.release()
cv.destroyAllWindows()

print("Total saved Images:", n)
