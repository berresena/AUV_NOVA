from ultralytics import YOLO
import cv2
import numpy as np
import time
import math
CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
fps = 0


def set_camera_properties(cap, width, height):
    cap.set(3, width)
    cap.set(4, height)

def capture_frame(cap):
    ret, frame = cap.read()
    if not ret:
        raise ValueError("Failed to read a frame from the camera")
    return frame

def detect_circles_with_yolov8(frame, yolo_model, focal_length_pixels, actual_radius_mm):
    results_list = yolo_model.predict(source=frame, show=False)
    results = results_list[0]
    circles = []
    for detection in results.boxes:
        x_min, y_min, x_max, y_max, conf, _ = detection.data[0].tolist()  # Kutu koordinatlarını al
        label = results.names[int(detection.cls[0])]  # Sınıf etiketini al

        # Yalnızca 'cember' etiketine sahip nesneleri işle
        if label == 'cember':
            # Merkez koordinatları ve yarıçapı hesapla
            x_center = int((x_min + x_max) / 2)
            y_center = int((y_min + y_max) / 2)
            radius = int((x_max - x_min) / 2)
            area = int(math.pi * radius * radius)
            # Dairenin kameradan olan uzaklığını tahmin et
            distance_to_circle = estimate_distance_to_circle(radius, focal_length_pixels, actual_radius_mm)

            print(f"Circle Center: ({x_center}, {y_center}), Radius: {radius}, Confidence: {conf},"
                  f" Label: {label}, Distance: {distance_to_circle} mm , Area: {area}")
            circles.append((x_center, y_center, radius, distance_to_circle))

            if(x_center > 100 and x_center < 200):
                if(y_center>100 and y_center<180):
                    frame = cv2.putText(frame, "Torpido At", (250,240), cv2.FONT_HERSHEY_SIMPLEX,0.5, (200,200,200), 2)
    return circles


def draw_circles_on_frame(frame, circles):
    output = frame.copy()
    if circles:
        for (x, y, r) in circles:
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    return output


def visualize_fps(image, fps: float) -> np.ndarray:
    if len(np.shape(image)) < 3:
        text_color = (255, 255, 255)  # white
    else:
        text_color = (0, 255, 0)  # green

    row_size = 20  # pixels
    left_margin = 24  # pixels
    font_size = 1
    font_thickness = 1

    fps_text = 'FPS: {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    return image


def estimate_distance_to_circle(radius_in_pixels, focal_length_in_pixels, actual_radius_in_mm):

    # Dairenin gerçek çapını ve piksel çapını karşılaştırarak bir ölçek faktörü bulunur.
    scale_factor = actual_radius_in_mm / (2 * radius_in_pixels)

    # Kameradan daireye olan mesafeyi tahmin etmek için ölçek faktörü kullanılır.
    distance_mm = focal_length_in_pixels * scale_factor

    return distance_mm


def main():
    try:
        cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
        if not cap.isOpened():
            raise ValueError("Could not open the camera")
        set_camera_properties(cap, IMAGE_WIDTH, IMAGE_HEIGHT)

        print("Press 'Esc' to exit...")

        fps = 0  # Initialize the fps variable

        # model_path = r"C:/Users/Monster/Desktop/yolov8/runs/detect/train/weights/best.pt"
        model_path = "/home/raclabnova/Downloads/best.pt"
        yolo_model = YOLO(model_path)

        focal_length_pixels = 450  # Örnek bir odak uzunluğu değeri (piksel cinsinden)
        actual_radius_mm = 50  # Gerçek daire çapı (milimetre cinsinden)
        while True:
            start_time = time.time()

            frame = capture_frame(cap)
            frame = cv2.line(frame, (180,140), (180,0), (0,0,0), 1)
            frame = cv2.line(frame, (180,140), (180,280), (0,0,0), 1)
            frame = cv2.line(frame, (180,140), (360,140), (0,0,0), 1)
            frame = cv2.line(frame, (180,140), (0,140), (0,0,0), 1)

            cv2.circle(frame, (180, 140), 100, (34, 255, 157), 2)
            circles = detect_circles_with_yolov8(frame, yolo_model, focal_length_pixels, actual_radius_mm)

            if circles:

                for (x, y, r, distance) in circles:
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds

            # Overlay FPS and display frames
            cv2.imshow("Frame", visualize_fps(frame, fps))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(e)

    finally:
        cv2.destroyAllWindows()
        cap.release()


if __name__ == "__main__":
    main()
