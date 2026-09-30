import threading
import cv2
from time import monotonic
import numpy as np


latest_frame = np.zeros((720, 1280, 3), dtype=np.uint8) 
latest_result = np.zeros((720, 1280, 3), dtype=np.uint8) 
hsv_ranges = [((140, 70, 70), (179, 255, 255))]
kernel_size = 5
stop_event = False


def read_cam():
    cam = cv2.VideoCapture(2)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    start_time = monotonic()
    live_window = 'CAM'
    vis_window = 'SEGMENT'
    cv2.namedWindow(live_window)
    cv2.namedWindow(vis_window)
    
    global latest_result
    global latest_frame
    global stop_event

    while monotonic() - start_time < 100:
        ret, frame = cam.read()
        cv2.imshow(live_window, frame)
        cv2.imshow(vis_window, latest_result)
        print('show frame')
        latest_frame = frame

        if cv2.waitKey(1) == ord('q'): 
            stop_event = True
            break

    cam.release()
    cv2.destroyAllWindows()


def make_mask(img_rgb):
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

    mask = None
    for lower, upper in hsv_ranges:
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        part = cv2.inRange(hsv, lower_np, upper_np)
        mask = part if mask is None else cv2.bitwise_or(mask, part)

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # type: ignore
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def calc_pose():
    global latest_frame
    global latest_result

    while not stop_event:
        # vis = latest_frame.copy()

        mask = make_mask(latest_frame)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        latest_result = mask_bgr
        print('Finished calculation')


cam_thread = threading.Thread(target=read_cam)
pose_thread = threading.Thread(target=calc_pose)

cam_thread.start()
pose_thread.start()
cam_thread.join()
pose_thread.join()

print('Done')