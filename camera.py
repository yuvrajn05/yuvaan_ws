# import pyzed.sl as sl
# import numpy as np
# import cv2  # For visualization or saving images

# # Create a ZED camera
# zed = sl.Camera()

# init_params = sl.InitParameters()
# init_params.depth_mode = sl.DEPTH_MODE.ULTRA # Use ULTRA depth mode
# init_params.coordinate_units = sl.UNIT.MILLIMETER # Use millimeter units (for depth measurements)

# image = sl.Mat()
# depth_map = sl.Mat()
# runtime_parameters = sl.RuntimeParameters()
# if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS :
#   # A new image and depth is available if grab() returns SUCCESS
#   zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve left image
#   zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH) # Retrieve depth

# depth_for_display = sl.Mat()
# zed.retrieve_image(depth_for_display, sl.VIEW.DEPTH)


import pyzed.sl as sl
import numpy as np
import cv2  # For visualization or saving images

# Create a ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.METER  # Use millimeter units (for depth measurements)

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Error opening camera: {err}")
    exit(1)

# Prepare containers for retrieving data
image = sl.Mat()
depth_map = sl.Mat()
depth_for_display = sl.Mat()
runtime_parameters = sl.RuntimeParameters()

# Main loop
while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image and depth is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT)  # Retrieve left image
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)  # Retrieve depth
        zed.retrieve_image(depth_for_display, sl.VIEW.DEPTH)  # Retrieve depth visualization
        
        # Convert to numpy arrays for OpenCV
        image_ocv = image.get_data()
        depth_ocv = depth_map.get_data()
        depth_display_ocv = depth_for_display.get_data()
        
        max = -1
        
        for d in depth_ocv:
            for i in d:
                if max<i and i != np.nan:
                    max=i
        print(max)
        
        # Display images
        cv2.imshow("Image", image_ocv)
        cv2.imshow("Depth", depth_display_ocv)
        
        # Break loop with 'q' key
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
    else:
        print("Error grabbing frame")

# Close camera
zed.close()
cv2.destroyAllWindows()