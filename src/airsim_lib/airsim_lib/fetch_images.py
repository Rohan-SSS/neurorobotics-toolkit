import airsim
import numpy as np
import os
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -150, 5)#.join()

while True:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("3", airsim.ImageType.Scene, False, False),
        ])
        
    response = responses[0]
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    #img_rgb = np.flipud(img_rgb)
    cv2.imshow("rgb1", img_rgb)
    
    response = responses[1]
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    #img_rgb = np.flipud(img_rgb)
    cv2.imshow("rgb2", img_rgb)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
