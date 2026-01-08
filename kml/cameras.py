from picamera2 import Picamera2
import cv2
import time
import numpy as np

def main():
    picam_left = Picamera2(camera_num=0)
    picam_right = Picamera2(camera_num=1)

    picam_left.configure(picam_left.create_preview_configuration())
    picam_right.configure(picam_right.create_preview_configuration())

    picam_left.start()
    picam_right.start()

    time.sleep(0.5)

    window_name = "Raspberry Pi Dual Camera Preview"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Output resolution
    TARGET_WIDTH = 640
    TARGET_HEIGHT = 480

    # Video settings
    FPS = 30
    FOURCC = cv2.VideoWriter_fourcc(*"mp4v")

    out_left = cv2.VideoWriter(
        "out3.mp4", FOURCC, FPS, (TARGET_WIDTH, TARGET_HEIGHT)
    )
    out_right = cv2.VideoWriter(
        "out4.mp4", FOURCC, FPS, (TARGET_WIDTH, TARGET_HEIGHT)
    )

    print("Recording... Press 'q' or ESC to stop and save videos.")

    try:
        while True:
            frame_left = picam_left.capture_array()
            frame_right = picam_right.capture_array()

            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)

            # Write frames to video files
            out_left.write(frame_left)
            out_right.write(frame_right)

            # Display side-by-side preview
            combined_frame = np.hstack((frame_left, frame_right))
            cv2.imshow(window_name, combined_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

    finally:
        print("Stopping cameras and saving videos...")

        picam_left.stop()
        picam_right.stop()

        out_left.release()
        out_right.release()

        cv2.destroyAllWindows()
        print("Saved: out1.mp4 and out2.mp4")

if __name__ == "__main__":
    main()
