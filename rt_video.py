import cv2
import numpy as np
import torch
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
    parser.add_argument(
        '--megapose-batch-size',
        type=int,
        default=16,
        help='Megapose batch size. Lower this to 8 or 4 if CUDA OOM continues.',
    )
    return parser.parse_args()


def calculate_iou(A, B):
    if A is None or len(A) == 0:
        return 0.0

    if B is None or len(B) == 0:
        return 0.0

    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    area_a = max(0.0, A[2] - A[0] + 1) * max(0.0, A[3] - A[1] + 1)
    area_b = max(0.0, B[2] - B[0] + 1) * max(0.0, B[3] - B[1] + 1)

    inter_x1 = max(A[0], B[0])
    inter_y1 = max(A[1], B[1])
    inter_x2 = min(A[2], B[2])
    inter_y2 = min(A[3], B[3])

    inter = max(0.0, inter_x2 - inter_x1 + 1) * max(0.0, inter_y2 - inter_y1 + 1)
    union = area_a + area_b - inter

    return inter / union if union > 0 else 0.0


def cleanup_cuda_after_oom(mpose):
    mpose.reset_tracking()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


def main():
    args = parse_args()
    mesh_path = Path(f'models/{args.mesh}')
    cam_file_path = Path(f'configs/{args.cam_file}')

    mpose = MegaPoseRunner(
        mesh_path,
        'fiducial',
        args.model,
        cam_file_path,
        batch_size=args.megapose_batch_size,
    )
    detector = ContourRunner(COLOR_RANGES)

    in_vid = cv2.VideoCapture(f'inputs/{args.video_file}')
    VID_FPS = int(in_vid.get(cv2.CAP_PROP_FPS))
    FRAME_COUNT = int(in_vid.get(cv2.CAP_PROP_FRAME_COUNT))
    IOU_OVERLAP = 0.5

    win_name = 'OUTPUT'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    if in_vid.isOpened():
        print('Video has been opened successfully')
    else:
        raise RuntimeError('Video could not be opened successfully')

    print('Video FPS: ', VID_FPS)
    print('Video FRAMES: ', FRAME_COUNT)
    print('Megapose batch size: ', args.megapose_batch_size)

    assert VID_FPS >= FPS
    wait_time = int(1000 / FPS) if FPS > 0 else 1

    Path('outputs').mkdir(parents=True, exist_ok=True)

    codec = cv2.VideoWriter_fourcc(*'mp4v')  # type: ignore
    tstamp = int(time())
    out_path = f'outputs/pose_track_{tstamp}.mp4'
    out_vid = cv2.VideoWriter(out_path, codec, FPS, OUT_RES)
    s_time = perf_counter()

    txt_path = f'outputs/pose_track_{tstamp}.txt'
    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('timestamp x_robot y_robot z_robot roll_robot pitch_robot yaw_robot runtime\n')

        for i in range(FRAME_COUNT):
            isWorking, frame = in_vid.read()

            if not isWorking:
                print('Video finished')
                break

            start_time = perf_counter()
            img_bgr = cv2.resize(frame, OUT_RES, interpolation=cv2.INTER_LINEAR)
            img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            res_img = img_bgr

            x = y = z = roll = pitch = yaw = float('nan')

            try:
                _, bboxs = detector.estimate(img)
                pose = None

                if bboxs is None or len(bboxs) == 0:
                    mpose.reset_tracking()
                else:
                    try:
                        pred_bboxs = mpose.mpose_bboxes()
                    except Exception:
                        pred_bboxs = []

                    iou = calculate_iou(bboxs[0], pred_bboxs)

                    if iou < IOU_OVERLAP:
                        mpose.reset_tracking()

                    if not mpose.tracking_active:
                        print('Reset Tracking')
                        mpose.load_detection(bboxs[0])

                    pose = mpose.estimate(img)

                if pose is not None:
                    x, y, z, roll, pitch, yaw = pose
                    res_img = mpose.draw_triaxis()

                end_time = perf_counter()
                print('Time taken: ', end_time - start_time)
                f.write(
                    f'{i:04d} '
                    f'{x} {y} {z} '
                    f'{roll} {pitch} {yaw} {end_time - start_time}\n'
                )
                f.flush()

            except torch.cuda.OutOfMemoryError as e:
                cleanup_cuda_after_oom(mpose)
                f.write(f'{i:04d} CUDA_OOM {e}\n')
                f.flush()
                print_exc()

            except Exception as e:
                f.write(f'{i:04d} ERROR {e}\n{type(e).__name__}\n{format_exc()}')
                f.flush()
                print_exc()

            finally:
                out_vid.write(res_img)
                cv2.imshow(win_name, res_img)

            if cv2.waitKey(wait_time) == ord('q'):
                break

    e_time = perf_counter()
    print(
        'Stats: \n'
        f'Video name: {out_path}\n'
        f'Pose file: {txt_path}\n'
        f'FPS: {FPS}\n'
        f'IOU: {IOU_OVERLAP}\n'
        f'No. of frames: {FRAME_COUNT}\n'
        f'Total time: {e_time - s_time}\n'
    )

    in_vid.release()
    out_vid.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()