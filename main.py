import cv2
import mediapipe as mp
import numpy as np
import serial

ser = serial.Serial('/dev/cu.usbmodem1101', 9600)

draw = mp.solutions.drawing_utils
mpHands = mp.solutions.hands

hands = mpHands.Hands(
    static_image_mode=False,
    model_complexity=1,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75,
    max_num_hands=2
)

hand=[[1, 2, 3, 4],      
[5, 6, 7, 8]  ,     
[9, 10, 11, 12],    
[13, 14, 15, 16],
[17, 18, 19, 20]]

def main():
    cap = cv2.VideoCapture(0)
    prevf = 0
    try:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break
            frame = cv2.flip(frame, 1)
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            processed = hands.process(frameRGB)

            llandmark, rlandmark = getlandmarks(frame, processed, draw, mpHands)

            cv2.imshow('Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            fingers = fingersheld(llandmark,rlandmark)
            if prevf != fingers:
                ser.write((str(fingers) + '\n').encode())
                prevf = fingers

    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()  

def getlandmarks(frame, processed, draw, mpHands):
    rlandmark = []
    llandmark = []

    if processed.multi_hand_landmarks:
        for handlm in processed.multi_hand_landmarks:
            for i, found_landmark in enumerate(handlm.landmark):
                height, width, _ = frame.shape
                x, y = int(found_landmark.x * width), int(found_landmark.y * height)
                
                landmark = [i, x, y]

                if handlm == processed.multi_hand_landmarks[0]:
                    llandmark.append(landmark)
                elif len(processed.multi_hand_landmarks) > 1 and handlm == processed.multi_hand_landmarks[1]:
                    rlandmark.append(landmark)

            draw.draw_landmarks(frame, handlm, mpHands.HAND_CONNECTIONS)

    return llandmark, rlandmark

def fingersheld(left, right):
    held = 0

    if len(left) > 20:
        for finger in hand:
            if get_angle(*(left[i] for i in finger)) < 30:
             held += 1
    if len(right) > 20:
        for finger in hand:
            if get_angle(*(right[i] for i in finger)) < 30:
                held += 1
    
    return held


def get_angle(p1, p2, p3, p4):

    v1 = np.array([p2[1] - p1[1], p2[2] - p1[2]])
    v2 = np.array([p3[1] - p2[1], p3[2] - p2[2]])
    v3 = np.array([p4[1] - p3[1], p4[2] - p3[2]])

    angle1 = np.arctan2(v1[1], v1[0])
    angle2 = np.arctan2(v2[1], v2[0])
    angle3 = np.arctan2(v3[1], v3[0])

    angle = np.abs(angle1 - angle2) + np.abs(angle2 - angle3)
    angle = np.degrees(angle)

    return angle


if __name__ == '__main__':
    main()

