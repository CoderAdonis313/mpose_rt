import cv2
from pathlib import Path
from time import time_ns, time
from argparse import ArgumentParser

# Open the default camera
print(
    """
    Fill out res 
    v4l2-ctl --list-devices
    v4l2-ctl --device=/dev/video0 --list-formats-ext
    Specify cam_id using v4l2-ctl
    Target resolution (e.g., --res 1920 1080)
    """
)
print('################################ PLEASE SET CAM ID FOR PROPER VIDEO #############################')

parser = ArgumentParser()
parser.add_argument('--cam_id', default=0, type=int, help='Specify cam_id using v4l2-ctl')
parser.add_argument('--res', nargs=2, type=int, metavar=('WIDTH', 'HEIGHT'), default=[1280, 720], help='Target resolution (e.g., --res 1920 1080)')
parser.add_argument('--save', action='store_true', help='specify explicitly to save video')
args = parser.parse_args()


CAM_ID = args.cam_id
RES = tuple(args.res)
OUTPUT_FOLDER = 'capture_videos'
out = None

cam = cv2.VideoCapture(CAM_ID)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, RES[0])
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, RES[1])


# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
if args.save:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')    #type: ignore
    tstamp = str(int(time() * 1000))
    fpath = Path.home() / OUTPUT_FOLDER
    fpath.mkdir(exist_ok=True, parents=True)
    fname = str(fpath / f'capture_{tstamp}.mp4')
    out = cv2.VideoWriter(fname, fourcc, 20.0, (frame_width, frame_height))

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    if args.save:
        out.write(frame)    #type: ignore

    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
if args.save:
    out.release()   #type: ignore
cv2.destroyAllWindows()
