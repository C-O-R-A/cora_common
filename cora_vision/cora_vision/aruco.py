import cv2
import cv2.aruco as aruco
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(1)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

mtx = np.array(
    [
        [1.45240572e03, 0.00000000e00, 1.06298441e03],
        [0.00000000e00, 1.47107102e03, 3.59724826e02],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ],
    dtype=np.float32,
)
dist = np.array(
    [0.36914676, -1.43524978, -0.07376983, 0.0213745, 2.47673674], dtype=np.float32
)

frame_count = []
current_frame = [0]
aruco_size = 0.135


def detect_distance(verbose=False, show=True):

    # Capture frame-by-frame
    ret, rawframe = cap.read()

    if not ret:
        return None

    # Convert the frame to grayscale
    gray = cv2.cvtColor(rawframe, cv2.COLOR_BGR2GRAY)

    # Detect the markers
    corners, ids, _ = detector.detectMarkers(gray)

    # Show detected markers
    if ids is not None:
        if verbose:
            print("Detected markers:", ids.flatten())
        frame = aruco.drawDetectedMarkers(rawframe, corners, ids)

    for corner in corners:
        rvec, tvec, markerPoints = aruco.estimateSinglePoseMarkers(
            corner, aruco_size, mtx, dist
        )

        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, aruco_size)

    if show:
        cv2.imshow('ArUco Marker Detection', frame)
    return frame, rawframe
