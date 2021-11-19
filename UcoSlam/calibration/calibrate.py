import numpy as np
import cv2
import glob

objp = np.zeros((6 * 7, 3), np.float32)
objp[ : , : 2] = np.mgrid[0 : 7, 0 : 6].T.reshape(-1, 2)
objpoints = []
imgpoints = []

criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cam = cv2.VideoCapture(0)
(w, h) = (int(cam.get(4)), int(cam.get(3)))

images = glob.glob('*.jpg')
for filename in images:
    frame = cv2.imread(filename)
    if frame is None:
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    if ret == True:
        objpoints.append(objp)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(frame, (7, 6), corners, ret)
        cv2.imshow('Find Chessboard', frame)
        cv2.waitKey(0)
    cv2.imshow('Find Chessboard', frame)
    print("Number of chess boards find:", len(imgpoints))
    if cv2.waitKey(1) == 27:
        break

ret, oldMtx, coef, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                      gray.shape[: : -1], None, None)
newMtx, roi = cv2.getOptimalNewCameraMatrix(oldMtx, coef, (w, h), 1, (w, h))

print("Original Camera Matrix:\n", oldMtx)
print("Optimal Camera Matrix:\n", newMtx)

np.save("Original camera matrix", oldMtx)
np.save("Distortion coefficients", coef)
np.save("Optimal camera matrix", newMtx)

cam.release()
cv2.destroyAllWindows()