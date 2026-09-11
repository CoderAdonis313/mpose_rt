import cv2
import numpy as np
from argparse import ArgumentParser
from pathlib import Path
from configs.config import FPS, COLOR_RANGES, OUT_RES
from mpose_runner import MegaPoseRunner
from contour_runner import ContourRunner
from time import perf_counter, time


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--video_file', required=True, help='Name of the video')
    parser.add_argument('--mesh', required=True, help='path of mesh file')
    parser.add_argument('--cam_file', required=True, help='path of the camera configuration file')
    parser.add_argument('--model', default='megapose-1.0-RGB')
    return parser.parse_args()


def main():
    args = parse_args()
    mesh_path = Path(f'models/{args.mesh}')
    cam_file_path = Path(f'configs/{args.cam_file}')

    mpose = MegaPoseRunner(mesh_path, 'fiducial', args.model, cam_file_path)
    detector = ContourRunner(COLOR_RANGES)

    cap = cv2.VideoCapture(f'vids/{args.video_file}')
    VID_FPS = cap.get(cv2.CAP_PROP_FPS)
    FRAME_COUNT = cap.get(cv2.CAP_PROP_FRAME_COUNT)

    win_name = 'OUTPUT'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, (1280, 720))

    if cap.isOpened():
        print(f'Video has been opened successfully')
    else:
        print('Video could not be opened successfully')

    print('Video FPS: ', VID_FPS)
    print('Video FRAMES: ', FRAME_COUNT)

    assert VID_FPS >= FPS
    wait_time = int(1000 / FPS) if FPS > 0 else 1

    codec = cv2.VideoWriter_fourcc(*'mp4v')
    tstamp = int(time())
    vid = cv2.VideoWriter(f'outputs/o_track_{tstamp}.mp4', codec, FPS, OUT_RES)

    while True:
        start_time = perf_counter()
        isWorking, frame = cap.read()
        res_img = frame

        if isWorking == True:
            try:
                img = cv2.resize(frame, OUT_RES, interpolation=cv2.INTER_LINEAR)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                _, bboxs = detector.estimate(img)
                mpose.load_detection(bboxs[0])
                pose = mpose.estimate(img)

                res_img = mpose.draw_triaxis()
                print('Pose: ', pose)
            except Exception as e:
                print(e)
            finally:
                vid.write(res_img)
                cv2.imshow(win_name, res_img)
        else:
            print('Video finished')
            break

        if cv2.waitKey(1) == ord('q'):
            break

        end_time = perf_counter()
        print('Time taken: ', end_time - start_time)

    cap.release()
    vid.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()