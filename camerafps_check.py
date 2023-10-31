import cv2

# 카메라 열기 (기본 카메라를 사용하려면 0을 사용)
camera_capture = cv2.VideoCapture(2)

# 원하는 FPS 설정 (예: 50 프레임/초)
desired_fps = 50.0
camera_capture.set(cv2.CAP_PROP_FPS, desired_fps)

# FPS 확인
actual_fps = camera_capture.get(cv2.CAP_PROP_FPS)
print(f"Actual FPS: {actual_fps}")

# 나머지 카메라 처리 코드 작성

# 카메라 닫기
camera_capture.release()
