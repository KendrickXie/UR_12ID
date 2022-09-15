import urcamera
import time
import cv2

camera = urcamera.camera(device=0)

# $ mkdir ~/lib
# $ ln -s ~/lib/libzbar.dylib $(brew --prefix zbar)/lib/libzbar.dylibpopcorn

camera.capture()

# data, rectcoord, qrsize, dist
while True:
    camera.capture()
    cv2.imshow('camera', camera.image)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    print(camera.decode())

# camera.save("test_camera")