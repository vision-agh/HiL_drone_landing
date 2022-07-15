from cmath import rect
from pickletools import uint8
from threading import local
import cv2
import os
import numpy as np
import csv

CENTROID_X = 0
CENTROID_Y = 1


def local_threshold(img: np.ndarray, cSize=7):
    height, width = img.shape
    out_img = np.zeros(img.shape, dtype=np.ubyte)
    for h in range(cSize, height - cSize):
        for w in range(cSize, width - cSize):
            patch = img[h - cSize : h + cSize, w - cSize : w + cSize]
            thrC = cv2.mean(patch)[0]
            out_img[h, w] = 255 if img[h, w] < thrC else 0

    return out_img


def interpolatedWindowedThreshold(img: np.ndarray, pSX=128, pSY=128):
    height, width = img.shape
    outImg = np.zeros(img.shape, dtype=np.ubyte)
    thresholds = np.zeros((height // pSY + 1, width // pSX + 1), dtype=np.ubyte)

    for h in range(0, height, pSY):
        for w in range(0, width, pSX):
            patch = img[h : min(h + pSY - 1, height), w : min(w + pSX - 1, width)]
            minP, maxP, _, _ = cv2.minMaxLoc(patch)
            thrW = np.ubyte((maxP - minP) * 0.5 + minP)
            thresholds[h // pSY, w // pSX] = thrW

    for h in range(0, height):
        for w in range(0, width):
            # Corners of the image
            if (
                h < pSY / 2
                and w < pSX / 2
                or h > (height - pSY / 2)
                and w < pSX / 2
                or h < pSY / 2
                and w > (width - pSX / 2)
                or h > (height - pSX / 2)
                and w > (width - pSY / 2)
            ):
                hT = h // pSY
                wT = w // pSX
                th11 = thresholds[hT, wT]
                th12 = thresholds[hT, wT]
                th21 = thresholds[hT, wT]
                th22 = thresholds[hT, wT]

            # Horizontal borders of the image
            if (
                h > pSY / 2
                and h <= (height - pSY / 2)
                and w < pSX
                or h > pSY / 2
                and h <= (height - pSY / 2)
                and w > (width - pSX / 2)
            ):
                hT = int(h - pSY / 2) // pSY
                wT = int(w - pSX / 2) // pSX
                wT = wT if wT >= 0 else 0  # Added for the Python version
                th11 = thresholds[hT, wT]
                th12 = thresholds[hT + 1, wT]
                th21 = thresholds[hT, wT]
                th22 = thresholds[hT + 1, wT]

            # Vertical borders of the image
            if (
                w > pSX / 2
                and w <= (width - pSX / 2)
                and h < pSY / 2
                or w > pSX / 2
                and w <= (width - pSX / 2)
                and h > (height - pSY / 2)
            ):
                hT = int(h - pSY / 2) // pSY
                hT = hT if hT >= 0 else 0  # Added for the Python version
                wT = int(w - pSX / 2) // pSX
                th11 = thresholds[hT, wT]
                th12 = thresholds[hT, wT]
                th21 = thresholds[hT, wT + 1]
                th22 = thresholds[hT, wT + 1]

            # Inside of image
            if h >= pSY / 2 and h < (height - pSY / 2) and w >= pSX / 2 and w < (width - pSX / 2):
                hT = int(h - pSY / 2) // pSY
                wT = int(w - pSX / 2) // pSX
                th11 = thresholds[hT, wT]
                th12 = thresholds[hT + 1, wT]
                th21 = thresholds[hT, wT + 1]
                th22 = thresholds[hT + 1, wT + 1]

            dX1 = w - pSX / 2 - wT * pSX
            dX2 = (wT + 1) * pSX - (w - pSX / 2)
            dY1 = h - pSY / 2 - hT * pSY
            dY2 = (hT + 1) * pSY - (h - pSY / 2)
            th1 = th11 * (dY2 / pSY) + th12 * (dY1 / pSY)
            th2 = th21 * (dY2 / pSY) + th22 * (dY1 / pSY)
            th = th1 * (dX2 / pSX) + th2 * (dX1 / pSX)
            outImg[h, w] = 255 if img[h, w] < th else 0

    return outImg


def createLUT(dataPath):
    dictLUT = dict()
    with open(dataPath, "r", newline="") as f:
        readerLUT = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        for row in readerLUT:
            dictLUT[row[0]] = row[1]

    return dictLUT


def rotateImagePoint(pt, org, alfa):
    retPoint = (
        int(
            np.cos(alfa) * (pt[CENTROID_X] - org[CENTROID_X])
            - np.sin(alfa) * (pt[CENTROID_Y] - org[CENTROID_Y])
            + org[CENTROID_X]
        ),
        int(
            np.sin(alfa) * (pt[CENTROID_X] - org[CENTROID_X])
            + np.cos(alfa) * (pt[CENTROID_Y] - org[CENTROID_Y])
            + org[CENTROID_Y]
        ),
    )
    return retPoint


def detectLandingPad(img, altitude, LUT):
    HEIGHT, WIDTH = img.shape[:2]

    # Convert to grey
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    gauss = cv2.GaussianBlur(grey, (5, 5), 0.65)

    # Interpolated windowed threshold
    binImg = interpolatedWindowedThreshold(gauss)

    # Connected Component Labelling (CCL)
    _, labels, stats, cent = cv2.connectedComponentsWithStats(binImg, connectivity=8, ltype=cv2.CV_32S)

    # Prepare output image
    outImg = img.copy()
    cv2.line(outImg, (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), (0, 0, 128), 1)
    cv2.line(outImg, (WIDTH // 2, HEIGHT), (WIDTH // 2, 0), (0, 0, 128), 1)

    # Get altitude
    cv2.putText(
        outImg,
        "Altitude: %2.2f [m]" % altitude,
        (0, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 128),
        2,
    )

    # Labeled object analysis
    circleSize = LUT[altitude]
    squareSize = circleSize / 5
    bboxCircle = list()
    bboxSquare = list()
    bboxRect = list()
    centrCircle = list()
    centrSquare = list()
    centrRect = list()
    for obj in stats:  # Each obj: [leftmost(x), topmost(y), width, height, area]
        bboxRatio = max(obj[cv2.CC_STAT_HEIGHT], obj[cv2.CC_STAT_WIDTH]) / min(
            obj[cv2.CC_STAT_HEIGHT], obj[cv2.CC_STAT_WIDTH]
        )
        bboxSize = obj[cv2.CC_STAT_HEIGHT] * obj[cv2.CC_STAT_WIDTH]
        bboxAreaRatio = bboxSize / obj[cv2.CC_STAT_AREA]
        meanSquareSize = (obj[cv2.CC_STAT_HEIGHT] + obj[cv2.CC_STAT_WIDTH]) / 2

        # Detect circles
        if bboxRatio < 1.4 and (
            bboxAreaRatio > 3.0
            and 0.5 * circleSize < meanSquareSize < 1.5 * circleSize
            or 150 < circleSize < 500
            and 1.55 < bboxAreaRatio < 2.0
            and 0.005 * circleSize * circleSize < obj[cv2.CC_STAT_AREA] < 0.04 * circleSize * circleSize
            and meanSquareSize < 0.65 * squareSize
            or circleSize >= 500
            and 1.5 < bboxAreaRatio < 2.0
            and 0.01 * circleSize * circleSize < obj[cv2.CC_STAT_AREA] < 0.04 * circleSize * circleSize
        ):
            bboxCircle.append(obj[:4])
            centrCircle.append(
                (
                    int(obj[cv2.CC_STAT_LEFT] + 0.5 * obj[cv2.CC_STAT_WIDTH]),
                    int(obj[cv2.CC_STAT_TOP] + 0.5 * obj[cv2.CC_STAT_HEIGHT]),
                )
            )
        # Detect squares
        elif (
            bboxRatio < 1.4
            and bboxAreaRatio < 2.25
            and 0.4 * squareSize * squareSize < bboxSize < 2.0 * squareSize * squareSize
            and 0.3 * squareSize * squareSize < obj[cv2.CC_STAT_AREA] < 1.8 * squareSize * squareSize
        ):
            bboxSquare.append(obj[:4])
            centrSquare.append(
                (
                    int(obj[cv2.CC_STAT_LEFT] + 0.5 * obj[cv2.CC_STAT_WIDTH]),
                    int(obj[cv2.CC_STAT_TOP] + 0.5 * obj[cv2.CC_STAT_HEIGHT]),
                )
            )
        # Detect rectangles
        elif (
            1.0 <= bboxRatio < 2.5
            and bboxAreaRatio < 2.3
            and 1.1 * squareSize * squareSize < bboxSize < 10 * squareSize * squareSize
            and obj[cv2.CC_STAT_AREA] > 0.5 * squareSize * squareSize
        ):
            bboxRect.append(obj[:4])
            centrRect.append(
                (
                    int(obj[cv2.CC_STAT_LEFT] + 0.5 * obj[cv2.CC_STAT_WIDTH]),
                    int(obj[cv2.CC_STAT_TOP] + 0.5 * obj[cv2.CC_STAT_HEIGHT]),
                )
            )

    # Detected shapes analysis
    position = None
    angle = None
    if len(bboxCircle) > 0 and len(bboxSquare) > 0 and len(bboxRect) > 0:
        for circleId in range(len(bboxCircle)):
            for squareId in range(len(bboxSquare)):
                for rectId in range(len(bboxRect)):
                    # Check if the centroids are inside the image
                    if (
                        0 < centrCircle[circleId][CENTROID_X] < WIDTH
                        and 0 < centrCircle[circleId][CENTROID_Y] < HEIGHT
                        and 0 < centrSquare[squareId][CENTROID_X] < WIDTH
                        and 0 < centrSquare[squareId][CENTROID_Y] < HEIGHT
                        and 0 < centrRect[rectId][CENTROID_X] < WIDTH
                        and 0 < centrRect[rectId][CENTROID_Y] < HEIGHT
                    ):
                        # Calculate the distance between centroids of a square and a rectangle
                        distSR = (centrSquare[squareId][CENTROID_X] - centrRect[rectId][CENTROID_X]) ** 2 + (
                            centrSquare[squareId][CENTROID_Y] - centrRect[rectId][CENTROID_Y]
                        ) ** 2

                        # Check if there is a square and a rectangle inside a circle
                        if (
                            bboxCircle[circleId][cv2.CC_STAT_LEFT]
                            < centrSquare[squareId][CENTROID_X]
                            < bboxCircle[circleId][cv2.CC_STAT_LEFT] + bboxCircle[circleId][cv2.CC_STAT_WIDTH]
                            and bboxCircle[circleId][cv2.CC_STAT_LEFT]
                            < centrRect[rectId][CENTROID_X]
                            < bboxCircle[circleId][cv2.CC_STAT_LEFT] + bboxCircle[circleId][cv2.CC_STAT_WIDTH]
                            and bboxCircle[circleId][cv2.CC_STAT_TOP]
                            < centrSquare[squareId][CENTROID_Y]
                            < bboxCircle[circleId][cv2.CC_STAT_TOP] + bboxCircle[circleId][cv2.CC_STAT_HEIGHT]
                            and bboxCircle[circleId][cv2.CC_STAT_TOP]
                            < centrRect[rectId][CENTROID_Y]
                            < bboxCircle[circleId][cv2.CC_STAT_TOP] + bboxCircle[circleId][cv2.CC_STAT_HEIGHT]
                        ):
                            # Draw bbox and centroid of the circle
                            cv2.rectangle(outImg, bboxCircle[circleId], (0, 0, 255), thickness=2)
                            cv2.circle(outImg, centrCircle[circleId], 2, (0, 0, 255))

                            # Draw bbox and centroid of the square
                            cv2.rectangle(outImg, bboxSquare[squareId], (0, 255, 0), thickness=2)
                            cv2.circle(outImg, centrSquare[squareId], 2, (0, 255, 0))

                            # Draw bbox and centroid of the rectangle
                            cv2.rectangle(outImg, bboxRect[rectId], (255, 0, 0), thickness=2)
                            cv2.circle(outImg, centrRect[rectId], 2, (255, 0, 0))

                            # Calculate UAV's position w.r.t. landing pad
                            centrePoint = (
                                int(0.5 * (centrSquare[squareId][CENTROID_X] + centrRect[rectId][CENTROID_X])),
                                int(0.5 * (centrSquare[squareId][CENTROID_Y] + centrRect[rectId][CENTROID_Y])),
                            )
                            position = (
                                WIDTH / 2 - centrePoint[CENTROID_X],
                                centrePoint[CENTROID_Y] - HEIGHT / 2,
                            )
                            cv2.putText(
                                outImg,
                                "Position 2D: (%3.2f, %3.2f) [px]" % position,
                                (0, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                (0, 0, 128),
                                2,
                            )
                            if centrePoint[CENTROID_X] > 0 and centrePoint[CENTROID_Y] > 0:
                                cv2.circle(outImg, centrePoint, 2, (0, 0, 255), 2)

                            # Calculate UAV's orientation w.r.t. landing pad (0 deg is up)
                            vecRS = (
                                centrSquare[squareId][CENTROID_X] - centrRect[rectId][CENTROID_X],
                                centrSquare[squareId][CENTROID_Y] - centrRect[rectId][CENTROID_Y],
                            )
                            angle = np.arctan2(
                                vecRS[CENTROID_Y],
                                vecRS[CENTROID_X],
                            ) + (np.pi / 2)

                            cv2.putText(
                                outImg,
                                "Angle: %2.2f [deg]" % (angle * 180 / np.pi),
                                (0, 110),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                (0, 0, 128),
                                2,
                            )

                            # Draw landing pad coordinate system
                            cv2.arrowedLine(
                                outImg,
                                centrRect[rectId],
                                centrSquare[squareId],
                                (0, 128, 255),
                                thickness=2,
                                tipLength=0.2,
                            )
                            cv2.arrowedLine(
                                outImg,
                                rotateImagePoint(centrSquare[squareId], centrePoint, np.pi / 2),
                                rotateImagePoint(centrRect[rectId], centrePoint, np.pi / 2),
                                (0, 255, 255),
                                thickness=2,
                                tipLength=0.2,
                            )

    return position, angle, outImg


if __name__ == "__main__":
    dataPath = "./calib_set" # FIXME
    dataLUTPath = "./circleLUT.csv" # FIXME
    LUT = createLUT(dataLUTPath)

    imgList = [f for f in os.listdir(dataPath) if os.path.isfile(os.path.join(dataPath, f))]
    for imgName in imgList:
        altitude = float(imgName[:-4])
        img = cv2.imread(os.path.join(dataPath, imgName))
        _, outImg = detectLandingPad(img, altitude, LUT)
        cv2.imshow("Output", outImg)
        key = cv2.waitKey(0)
        if key == ord("x"):
            break
    cv2.destroyAllWindows()
