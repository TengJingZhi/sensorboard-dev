import time
import hiwonder.ActionGroupControl as AGC    #动作库，必须包含该库
import subprocess   #拍照
import apriltag
import cv2
import numpy as np
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
board = rrc.Board()
ctl = Controller(board)     #转头准备

current_position = np.array([0, 0], dtype=np.float64)
current_orientation = np.array([0, 0], dtype=np.float64)
next_stop = '0'

tag_poses = {}
tag_poses['36'] = np.array([[44.7,23.8,41.5], [44.7, 18.8, 41.5], [44.7, 18.8, 36.5], [44.7, 23.8, 36.5]], dtype=np.float64)
tag_poses['37'] = np.array([[20.6,100,41.7], [25.6, 100, 41.7], [25.6, 100, 36.7], [20.6, 100, 36.7]], dtype=np.float64)
tag_poses['38'] = np.array([[95, 82.2,41.7], [95, 77.2, 41.7], [95, 77.2, 36.7], [95, 82.2, 36.7]], dtype=np.float64)
tag_poses['39'] = np.array([[76.5, 0, 42.1], [71.5, 0, 42.1], [71.5, 0, 37.1], [76.5, 0, 37.1]], dtype=np.float64)
centre_poses = {}
centre_poses['1'] = np.array([14.7,21.3], dtype=np.float64)
centre_poses['2'] = np.array([23.1, 70], dtype=np.float64)
centre_poses['3'] = np.array([65,79.7], dtype=np.float64)
centre_poses['4'] = np.array([74, 30], dtype=np.float64)
target_orientations = {}
target_orientations['1'] = np.array([1,0], dtype=np.float64)
target_orientations['2'] = np.array([0,1], dtype=np.float64)
target_orientations['3'] = np.array([1,0], dtype=np.float64)
target_orientations['4'] = np.array([0,-1], dtype=np.float64)
target_orientations['5'] = np.array([1,0], dtype=np.float64)

def calculate_diff():
    position_diff = np.array(current_position) - np.array(centre_poses[next_stop])
    orientation_diff = np.array(current_orientation) - np.array(target_orientations[next_stop])
    return position_diff, orientation_diff

def calc_distance(position_diff):  #只计算水平距离
    distance = np.linalg.norm(position_diff)  # 只计算x和y方向的距离
    return distance

def capture_image():
    """拍照"""
    timestamp = int(time.time())
    filename = f"/home/pi/Pictures/photo_{timestamp}.jpg"
    cmd = f"fswebcam -r 2592x1944 --no-banner -S 3 {filename}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"拍照失败: {result.stderr}")
        return None
    
    print(f"照片已保存: {filename}")
    return filename

def detect_apriltag(filename):
    # load the input image and convert it to grayscale
    print("[INFO] loading image...")
    image = cv2.imread(filename)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # define the AprilTags detector options and then detect the AprilTags
    # in the input image
    print("[INFO] detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    return results

def locate_camera():
    intrinsic=np.array(([1.944903664123011e+03,0,1.283069051100245e+03],[0,1.950095436307893e+03,9.831983420212778e+02],[0,0,1]),dtype=np.double) #内参矩阵
    distortion=np.array([-0.384402275498781,0.284681889150075,0,0]) #畸变系数

    global current_position, current_orientation

    current_position = None  # 先置None，失败时保持None，成功时被覆盖

    objlist = []
    imglist = []
    filename = capture_image()
    if filename is None:
        print("无法拍摄照片，程序终止。")
        return None
    for r in detect_apriltag(filename):
        print("[INFO] Detected AprilTag ID: {}".format(r.tag_id))
        objlist.extend(tag_poses[str(r.tag_id)])
        imglist.extend(r.corners)

    if len(objlist) < 1:
        print("检测到的AprilTag数量不足，无法计算相机位姿。")
        return None
    ifsuccess, rvec, tvec = cv2.solvePnP(np.array(objlist, dtype=np.float64), np.array(imglist, dtype=np.float64), intrinsic, distortion)

    if not ifsuccess:
        print("PnP求解失败，无法计算相机位姿。")
        return None

    rotateMatrix = cv2.Rodrigues(rvec)[0]

    current_position = -np.linalg.inv(rotateMatrix)@tvec
    current_orientation = np.linalg.inv(rotateMatrix)@(np.array([[0],[0],[1]])-tvec)-current_position

    current_position = current_position[:2].flatten()  # 只保留x和y坐标，展平为一维
    print("相机的坐标为：",current_position)

    current_orientation = current_orientation[:2].flatten()  # 只保留x和y方向的朝向，展平为一维
    norm = np.linalg.norm(current_orientation)
    if norm != 0:
        current_orientation /= norm
    print("相机的朝向为：",current_orientation)

def decide_panning_action(position_diff, orientation_xOy):
    # 根据位置差异决定平移方向
    new_position_diff_forward = position_diff + 3*orientation_xOy  # 向前移动
    df = calc_distance(new_position_diff_forward)
    new_position_diff_backward = position_diff - 4*orientation_xOy  # 向后移动
    db = calc_distance(new_position_diff_backward)
    new_position_diff_left = position_diff + 2.9*np.array([-orientation_xOy[1],orientation_xOy[0]])  # 向左移动
    dl = calc_distance(new_position_diff_left)
    new_position_diff_right = position_diff + 2.1*np.array([orientation_xOy[1],-orientation_xOy[0]])  # 向右移动
    dr = calc_distance(new_position_diff_right)

    distances = {'go_forward_one_step': df, 'back': db, 'left_move': dl, 'right_move': dr}
    best_direction = min(distances, key=distances.get)
    print("最佳移动方向：", best_direction)
    # 这里可以根据具体需求选择合适的平移方向
    return best_direction  # 示例：返回距离最近的移动方向

def decide_rotation_action(orientation_diff):
    target = current_orientation - orientation_diff  # 推导出目标朝向

    # 向左转21°（逆时针旋转）
    theta_left = np.radians(21)
    R_left = np.array([[np.cos(theta_left), -np.sin(theta_left)],
                       [np.sin(theta_left), np.cos(theta_left)]])
    new_od_left = R_left @ current_orientation - target
    dist_left = np.linalg.norm(new_od_left)

    # 向右转30°（顺时针旋转）
    theta_right = np.radians(-30)
    R_right = np.array([[np.cos(theta_right), -np.sin(theta_right)],
                        [np.sin(theta_right), np.cos(theta_right)]])
    new_od_right = R_right @ current_orientation - target
    dist_right = np.linalg.norm(new_od_right)

    print(f"左转后od模长: {dist_left:.4f}, 右转后od模长: {dist_right:.4f}")

    if dist_left < dist_right:
        print("最佳旋转方向：turn_left")
        return 'turn_left'
    else:
        print("最佳旋转方向：turn_right")
        return 'turn_right'

next_stop = '1'
while True:
    locate_camera()
    if current_position is None:
        AGC.runActionGroup('left_move')
        print("未检测到相机位置，执行左移动作以尝试重新定位。")
        continue
    print("当前相机位置：", current_position)
    print("当前相机朝向：", current_orientation)
    pd,od = calculate_diff()
    print("位置差异pd：", pd)
    print("朝向差异od：", od)
    distance = calc_distance(pd)
    if (np.linalg.norm(od) > 0.19): #od的模大于0.19,15 degrees
        action = decide_rotation_action(od)
        AGC.runActionGroup(action)
    elif (np.linalg.norm(pd) > 3): #pd的模大于3
        action = decide_panning_action(pd, current_orientation)
        AGC.runActionGroup(action)
    else:
        break

print("到达第1个目标点，准备前往第2个目标点。")
next_stop = '2'
while True:
    locate_camera()
    if current_position is None:
        AGC.runActionGroup('turn_left')
        print("未检测到相机位置，执行左转动作以尝试重新定位。")
        continue
    print("当前相机位置：", current_position)
    print("当前相机朝向：", current_orientation)
    pd,od = calculate_diff()
    print("位置差异pd：", pd)
    print("朝向差异od：", od)
    distance = calc_distance(pd)
    if (np.linalg.norm(od) > 0.19): #od的模大于0.19,15 degrees
        action = decide_rotation_action(od)
        AGC.runActionGroup(action)
    elif (np.linalg.norm(pd) > 3): #pd的模大于3
        action = decide_panning_action(pd, current_orientation)
        AGC.runActionGroup(action)
    else:
        break

print("到达第2个目标点，准备前往第3个目标点。")
next_stop = '3'
while True:
    locate_camera()
    if current_position is None:
        attemp = 0
        if attemp % 2 == 0:
            AGC.runActionGroup('turn_right')
            print("未检测到相机位置，执行右转动作以尝试重新定位。")
            continue
        if attemp % 2 == 1:
            AGC.runActionGroup('turn_left,times=2')
            print("未检测到相机位置，执行左转动作以尝试重新定位。")
            continue
        attemp += 1
    attemp = 0
    print("当前相机位置：", current_position)
    print("当前相机朝向：", current_orientation)
    pd,od = calculate_diff()
    print("位置差异pd：", pd)
    print("朝向差异od：", od)
    distance = calc_distance(pd)
    if (np.linalg.norm(od) > 0.19): #od的模大于0.19,15 degrees
        action = decide_rotation_action(od)
        AGC.runActionGroup(action)
    elif (np.linalg.norm(pd) > 3): #pd的模大于3
        action = decide_panning_action(pd, current_orientation)
        AGC.runActionGroup(action)
    else:
        break

print("到达第3个目标点，准备前往第4个目标点。")
next_stop = '4'
while True:
    locate_camera()
    if current_position is None:
        AGC.runActionGroup('turn_right')
        print("未检测到相机位置，执行右转动作以尝试重新定位。")
        continue
    print("当前相机位置：", current_position)
    print("当前相机朝向：", current_orientation)
    pd,od = calculate_diff()
    print("位置差异pd：", pd)
    print("朝向差异od：", od)
    distance = calc_distance(pd)
    if (np.linalg.norm(od) > 0.19): #od的模大于0.19,15 degrees
        action = decide_rotation_action(od)
        AGC.runActionGroup(action)
    elif (np.linalg.norm(pd) > 3): #pd的模大于3
        action = decide_panning_action(pd, current_orientation)
        AGC.runActionGroup(action)
    else:
        break

print("到达第4个目标点，准备前往第5个目标点。")
next_stop = '5'
AGC.runActionGroup('turn_left', times=4)
AGC.runActionGroup('go_forward', times=3)