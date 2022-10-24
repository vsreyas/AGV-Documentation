import numpy as np
import cv2
import time


def pose_estimation(img, CameraMatrix, DistortionVector):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ArucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters_create()

    Corners, Id, rejected_img_points = cv2.aruco.detectMarkers(gray, ArucoDict, parameters=parameters)
    TVectors = []
    # If markers are detected
    if len(Corners) > 0:
        for i in range(0, len(Id)):
            # Estimate pose of each marker and return the values RotVector and TransVector---(different from those of
            # camera coefficients)
            RotVector, TransVector, markerPoints = cv2.aruco.estimatePoseSingleMarkers(Corners[i], 0.01, CameraMatrix,
                                                                                       DistortionVector)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(img, Corners)
            TVectors.append(TransVector)
            # Draw Axis
            cv2.drawFrameAxes(img, CameraMatrix, DistortionVector, RotVector, TransVector, 0.01)
        print(TVectors)
    try:
        dist = np.linalg.norm(TVectors[0] - TVectors[1])
    except:
        dist = -1
    return img, dist


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

k = np.load("/home/venky/PycharmProjects/AGV/calibration_matrix.npy")
d = np.load("/home/venky/PycharmProjects/AGV/distortion_coefficients.npy")

video = cv2.VideoCapture(0)
time.sleep(2.0)

while True:
    ret, frame = video.read()

    if not ret:
        break
    h, w, _ = frame.shape
    width = 600
    height = int(width * (h / w))
    image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    detected_markers = cv2.aruco.drawDetectedMarkers(image=image, corners=corners)
    cv2.imshow("Image", detected_markers)
    output, distance = pose_estimation(frame, k, d)

    cv2.imshow('Estimated Pose', output)
    print(distance)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

video.release()
cv2.destroyAllWindows()

# # Uncomment to save
# cv2.imwrite("output_sample.png",detected_markers)

cv2.waitKey(0)
