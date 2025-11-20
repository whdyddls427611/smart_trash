import cv2
import numpy as np
import tensorflow as tf

# 클래스 이름 로드
with open("classes.txt", "r") as f:
    class_names = [line.strip() for line in f]

# TFLite 모델 로딩
interpreter = tf.lite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']
height, width = input_shape[1], input_shape[2]

# 웹캠 열기
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 전처리: 리사이즈 + 정규화
    input_img = cv2.resize(frame, (width, height))
    input_img = np.expand_dims(input_img, axis=0).astype(np.float32) / 255.0

    # 추론
    interpreter.set_tensor(input_details[0]['index'], input_img)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]

    # confidence 기준 적용
    confidence = max(output_data)
    pred = np.argmax(output_data)

    if confidence < 0.6:
        label = "Unrecognized"
    else:
        label = f"{class_names[pred]} ({confidence:.2f})"

    # 화면에 표시
    cv2.putText(frame, f"Prediction: {label}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("TFLite Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()