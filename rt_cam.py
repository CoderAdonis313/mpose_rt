import cv2
import numpy as np


def main():
    cam = cv2.VideoCapture(0)
    FPS = cam.get(cv2.CAP_PROP_FPS)
    win_name = 'OUTPUT'
    cv2.namedWindow(win_name)

    while True:
        isWorking, frame = cam.read()

        if isWorking == True:
            cv2.imshow(win_name, frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()