import cv2

# Load Haar Cascade cho full body
cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'  # Ä‘áº£m báº£o file nĂ y náº±m cĂ¹ng thÆ° má»¥c
body_cascade = cv2.CascadeClassifier(cascade_path)

if body_cascade.empty():
    raise IOError("KhĂ´ng táº£i Ä‘Æ°á»£c file cascade: " + cascade_path)

# Khá»Ÿi táº¡o webcam
cap = cv2.VideoCapture(0)  # 0 lĂ  webcam máº·c Ä‘á»‹nh

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Chuyá»ƒn sang áº£nh xĂ¡m
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # PhĂ¡t hiá»‡n ngÆ°á»i toĂ n thĂ¢n
    bodies = body_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

    # Váº½ hĂ¬nh chá»¯ nháº­t quanh Ä‘á»‘i tÆ°á»£ng
    for (x, y, w, h) in bodies:
        cv2.putText(frame, 'Human', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Hiá»ƒn thá»‹ káº¿t quáº£
    cv2.imshow('Full Body Detection (Webcam)', frame)

    # Nháº¥n ESC Ä‘á»ƒ thoĂ¡t
    if cv2.waitKey(1) == 27:
        break

# Giáº£i phĂ³ng tĂ i nguyĂªn
cap.release()
cv2.destroyAllWindows()
