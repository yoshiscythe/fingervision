import cv2
from datetime import datetime

threshold = 50

cap = cv2.VideoCapture(4) # 任意のカメラ番号に変更する

while True:
    ret, frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_hue = cv2.extractChannel(frame_hsv, 0)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, img_thresh = cv2.threshold(frame_hue, threshold, 179, cv2.THRESH_BINARY)
    cv2.imshow("camera", frame)
    cv2.imshow("hue_thresh", img_thresh)
    cv2.imshow("gray", frame_gray)

    k = cv2.waitKey(1)&0xff # キー入力を待つ
    if k == ord('p'):
        # 「p」キーで画像を保存
        date = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = "./img/" + date + ".png"
        cv2.imwrite(path, frame) # ファイル保存

        cv2.imshow(path, frame) # キャプチャした画像を表示
    elif k == ord('q'):
        # 「q」キーが押されたら終了する
        break

# キャプチャをリリースして、ウィンドウをすべて閉じる
cap.release()
cv2.destroyAllWindows()