import cv2
import numpy as np
from argparse import ArgumentParser
from pathlib import Path
from configs.config import FPS, COLOR_RANGES, OUT_RES
from mpose_runner import MegaPoseRunner
from contour_runner import ContourRunner
from time import perf_counter, time
from traceback import print_exc, format_exc


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

    in_vid = cv2.VideoCapture(f'inputs/{args.video_file}')
    VID_FPS = int(in_vid.get(cv2.CAP_PROP_FPS))
    FRAME_COUNT = int(in_vid.get(cv2.CAP_PROP_FRAME_COUNT))

    win_name = 'OUTPUT'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    if in_vid.isOpened():
        print(f'Video has been opened successfully')
    else:
        print('Video could not be opened successfully')

    print('Video FPS: ', VID_FPS)
    print('Video FRAMES: ', FRAME_COUNT)

    assert VID_FPS >= FPS
    wait_time = int(1000 / FPS) if FPS > 0 else 1

    #Write video
    codec = cv2.VideoWriter_fourcc(*'mp4v') #type: ignore
    tstamp = int(time())
    out_path = f'outputs/no_track_{tstamp}.mp4'
    out_vid = cv2.VideoWriter(out_path, codec, FPS, OUT_RES)
    s_time = perf_counter()

    txt_path = f'outputs/no_track_{tstamp}.txt'
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("timestamp x_robot y_robot z_robot roll_robot pitch_robot yaw_robot runtime\n")

        for i in range(FRAME_COUNT):
            isWorking, frame = in_vid.read()
            res_img = frame

            if isWorking == True:
                try:
                    start_time = perf_counter()
                    img = cv2.resize(frame, OUT_RES, interpolation=cv2.INTER_LINEAR)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    _, bboxs = detector.estimate(img)
                    mpose.load_detection(bboxs[0])
                    pose = mpose.estimate(img)

                    res_img = mpose.draw_triaxis()
                    print('Pose: ', pose)

                    end_time = perf_counter()
                    print('Time taken: ', end_time - start_time)
                    x, y, z, roll, pitch, yaw = pose    #type: ignore
                    f.write(
                        f"{i:04d} "
                        f"{x} {y} {z} "
                        f"{roll} {pitch} {yaw} {end_time - start_time}\n"
                    )
                    f.flush()
                except Exception as e:
                    f.write(f"{i:04d} ERROR {e}\n {type(e).__name__}\n, {e}\n, {format_exc()}\n")
                    f.flush()
                    print_exc()
                finally:
                    out_vid.write(res_img)
                    cv2.imshow(win_name, res_img)
            else:
                print('Video finished')
                break

            if cv2.waitKey(wait_time) == ord('q'):
                break


    e_time = perf_counter()
    print('Stats: \n' \
        f'Video name: {out_path}\n'\
        f'Pose file: {txt_path}\n'\
        f'FPS: {FPS}\n'\
        f'No. of frames: {FRAME_COUNT}\n'\
        f'Total time: {e_time - s_time}\n'\
    )
    in_vid.release()
    out_vid.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()