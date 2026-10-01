import threading
from concurrent.futures import ThreadPoolExecutor
import cv2
from time import monotonic
import numpy as np

RES_HEIGHT = 720
RES_WIDTH = 2 * 1280

latest_frame = np.zeros((RES_HEIGHT, RES_WIDTH, 3), dtype=np.uint8) 
latest_result = np.zeros((RES_HEIGHT, RES_WIDTH, 3), dtype=np.uint8) 
latest_vis = np.zeros((RES_HEIGHT, RES_WIDTH, 3), dtype=np.uint8) 


hsv_ranges = [((140, 70, 70), (179, 255, 255))]
kernel_size = 5
stop_event = threading.Event()
frame_lock = threading.Lock()


def read_cam():
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, RES_WIDTH)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_HEIGHT)

    start_time = monotonic()
    live_window = 'CAM'
    vis_window = 'SEGMENT'
    bbox_window = 'BBOX'

    cv2.namedWindow(live_window)
    cv2.namedWindow(vis_window)
    cv2.namedWindow(bbox_window)
    
    global latest_result
    global latest_frame
    global stop_event

    while monotonic() - start_time < 1000:
        ret, frame = cam.read()
        display_result = frame
        display_vis = frame

        with frame_lock:
            latest_frame = frame.copy()
            display_result = latest_result.copy()
            display_vis = latest_vis.copy()

        cv2.imshow(live_window, frame)
        cv2.imshow(vis_window, display_result)
        cv2.imshow(bbox_window, display_vis)
        
        print('show frame')

        if cv2.waitKey(1) == ord('q'): 
            stop_event.set()
            break

    cam.release()
    cv2.destroyAllWindows()
    stop_event.set()


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


def best_bbox_from_mask(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = -1

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 8:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        # optional extra filters
        if w < 8 or h < 8:
            continue

        if area > best_area:
            best_area = area
            best = (x, y, w, h)
    return best


def calc_pose():
    global latest_frame
    global latest_result
    global latest_vis

    while not stop_event.is_set():
        vis = None
        with frame_lock:
            vis = latest_frame.copy()

        mask = make_mask(vis)
        bbox = best_bbox_from_mask(mask)
        
        if bbox is not None:
            x1, y1, w, h = bbox
            cv2.rectangle(vis, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        with frame_lock:
            latest_result = mask_bgr
            latest_vis = vis
        print('Finished calculation')


def main():
    pool = ThreadPoolExecutor()
    with pool as executor:
        executor.submit(read_cam)
        executor.submit(calc_pose)

    print('Done')


if __name__ == '__main__':
    main()