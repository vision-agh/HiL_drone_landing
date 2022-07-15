import setup_path
import airsim

import cv2
import numpy as np
import sys
import serial
import math
import time
import csv
import os

from landing_detection_alg import createLUT

DT = 1
FPS = 25
LAND_X = -20.1
LAND_Y = -7.3


def printUARTdata(data):
    vs = None
    yawRate = None
    landing = False
    needA = False
    needF = False
    triggered = None
    try:
        s = data.decode()
    except:
        return vs

    allMsgs = s.split("\n")
    for msg in allMsgs:
        if msg.startswith("v:"):
            try:
                v = msg.split(":")[-1]
                vs = v.split(",")
                vs = [float(s) for s in vs]
            except:
                pass
        elif msg.startswith("t:"):
            try:
                triggerStatus = int(msg.split(":")[-1])
                if triggerStatus == 0:
                    triggered = False
                elif triggerStatus == 1:
                    triggered = True
            except:
                pass
        elif msg.startswith("a:"):
            needA = True
        elif msg.startswith("f:"):
            needF = True
        elif msg.startswith("y:"):
            try:
                yawRate = msg.split(":")[-1]
                yawRate = float(yawRate)
            except:
                pass
        elif msg.startswith("l:"):
            landing = True
        else:
            pass

    return triggered, landing, vs, yawRate, (needA, needF)


# prepare test results file
outPath = "/home" # FIXME
outFile = "hil_test_" + str(int(np.round(time.time(), 0))) + ".csv"
outHead = ["Test_no", "start_x", "start_y", "start_z", "start_yaw", "time", "x", "y", "z", "q_w", "q_x", "q_y", "q_z"]
with open(os.path.join(outPath, outFile), "w", encoding="UTF8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(outHead)

# open serial connection to Arty-Z7
BAUDRATE = 57600
portName = "/dev/ttyUSB1" # FIXME
artyZ7 = serial.Serial(portName, baudrate=BAUDRATE, timeout=1)
print("Connected to: " + artyZ7.name)

# create LUT for landing detection algorithm
pathLUT = "./circleLUT.csv" # FIXME
LUT = createLUT(pathLUT)

# connect to the AirSim simulator
print("Connecting to AirSim server...")
client = airsim.MultirotorClient()
client.confirmConnection()

cv2.namedWindow("Down camera", cv2.WINDOW_AUTOSIZE)

# start tests
nTests = int(input("Enter number of test:"))
airsim.wait_key("Press any key to start tests")
algTrigger = False
for i in range(nTests):
    print("--- Executing test no. %d ---" % (i + 1))

    # takeoff
    print("Taking off...")
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    # move to start point
    zstart = np.random.uniform(-10.0, -5.0)
    xstart = np.random.uniform(LAND_X - zstart / 3.0, LAND_X + zstart / 3.0)
    ystart = np.random.uniform(LAND_Y - zstart / 2.0, LAND_Y + zstart / 2.0)
    client.moveToPositionAsync(xstart, ystart, zstart, 5).join()
    time.sleep(3)

    # initial rotation
    yawstart = np.random.uniform(0, 360)
    client.rotateToYawAsync(yawstart).join()
    print("Start pose: pos = (" + str(xstart) + ", " + str(ystart) + "," + str(zstart) + "), yaw = " + str(yawstart))

    # hover
    client.hoverAsync().join()

    # wait for drone stabilisation & send altitude data
    for j in range(7):
        distData = client.getDistanceSensorData("Distance", "Drone1")
        altitudeMStr = "Am" + str(int(np.round(math.trunc(distData.distance), 0))).zfill(2)
        altitudeCmStr = "Ac" + str(int(np.round((distData.distance % 1) * 100, 0))).zfill(2)
        artyZ7.write(altitudeMStr.encode("utf-8"))
        artyZ7.write(altitudeCmStr.encode("utf-8"))
        time.sleep(1)

    # start test
    frameClock = time.time() + 1 / FPS
    firstFrameTime = time.time()
    frameCnt = 0
    while algTrigger is not True:
        print("Triggering!\r", end="")
        artyZ7.write(b"T111")
        res = artyZ7.read_all()
        algTrigger, _, _, _, _ = printUARTdata(res)
    print("Triggered!")
    while True:
        # Get data from AirSim
        rawImage = client.simGetImage("down_cam", airsim.ImageType.Scene)
        distData = client.getDistanceSensorData("Distance", "Drone1")

        # Show new image
        if rawImage == None:
            print("Camera is not returning image, please check airsim for error messages")
            sys.exit(0)
        else:
            png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
            cv2.imshow("Down camera", png)
        key = cv2.waitKey(1)
        frameCnt += 1
        actualFPS = frameCnt / (time.time() - firstFrameTime)
        print("FPS: " + str(actualFPS) + "\r", end="")
        if key == ord("x"):
            break

        # Get figures size & round to int
        altitude = np.round(distData.distance, 2) if distData.distance >= 4.0 else 4.00
        circleSize = LUT[altitude]
        squareSize = circleSize / 5

        # Send figure sizes to FPGA
        res = artyZ7.read_all()
        _, landing, PIDv, yawRate, (needAltitude, needFigures) = printUARTdata(res)
        if needAltitude is True:
            altitudeMStr = "Am" + str(int(np.round(math.trunc(distData.distance), 0))).zfill(2)
            altitudeCmStr = "Ac" + str(int(np.round((distData.distance % 1) * 100, 0))).zfill(2)
            artyZ7.write(altitudeMStr.encode("utf-8"))
            artyZ7.write(altitudeCmStr.encode("utf-8"))
        if needFigures is True:
            circleStr = "C" + str(int(np.round(circleSize, 0))).zfill(3)
            squareStr = "S" + str(int(np.round(squareSize, 0))).zfill(3)
            artyZ7.write(circleStr.encode("utf-8"))
            artyZ7.write(squareStr.encode("utf-8"))

        # if algTrigger is True:
        if landing is True:
            print("\n[P] LANDING !!!")
            client.landAsync().join()
            break
        elif PIDv is not None and len(PIDv) == 3:
            if yawRate is not None:
                client.moveByVelocityBodyFrameAsync(
                    PIDv[0], PIDv[1], PIDv[2], DT, yaw_mode=airsim.YawMode(True, yawRate)
                )
            else:
                client.moveByVelocityBodyFrameAsync(PIDv[0], PIDv[1], PIDv[2], DT)

    landingTime = time.time() - firstFrameTime
    while algTrigger is not False:
        print("Cutting off!\r", end="")
        artyZ7.write(b"T000")
        res = artyZ7.read_all()
        algTrigger, _, _, _, _ = printUARTdata(res)
    print("Cutted off!!")

    # get final uav's pose & write to CSV file
    finalPose = client.simGetVehiclePose()
    csvData = [
        i + 1,
        xstart,
        ystart,
        zstart,
        yawstart,
        landingTime,
        finalPose.position.x_val,
        finalPose.position.y_val,
        finalPose.position.z_val,
        finalPose.orientation.w_val,
        finalPose.orientation.x_val,
        finalPose.orientation.y_val,
        finalPose.orientation.z_val,
    ]
    with open(os.path.join(outPath, outFile), "a", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(csvData)

    # reset
    client.reset()
    client.armDisarm(False)

    # wait for server to do the reset
    time.sleep(5)

cv2.destroyAllWindows()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

print("--- Tests done ---")
