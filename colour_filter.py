import numpy as np
import cv2

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image

def filter_colour(rgb_image, lower_bound_colour, upper_bound_colour):
    #convert the image into the HSV colour space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, lower_bound_colour, upper_bound_colour)

    return mask



if __name__ == '__main__':
    video_capture = cv2.VideoCapture(0)
    while(True):
        ret, frame = video_capture.read()
        cv2.imshow("Frame",frame)
        #find the upper and lower bound of the Clothing
        clothUpper = (29, 186, 255)
        clothLower = (24, 78, 128)
        binaryImage = filter_colour(frame, clothLower, clothUpper)

        bgrImage = cv2.cvtColor(binaryImage, cv2.COLOR_GRAY2BGR)

        # Apply an erosion + dilation to get rid of small noise:

        # Set kernel (structuring element) size:
        kernelSize = 3

        # Set operation iterations:
        opIterations = 3

        # Get the structuring element:
        maxKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))

        # Perform closing:
        openingImage = cv2.morphologyEx(binaryImage, cv2.MORPH_OPEN, maxKernel, None, None, opIterations, cv2.BORDER_REFLECT101)

        contours, _ = cv2.findContours(openingImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centres = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])         
            if area < 1000:
                continue  
            imageMoments = cv2.moments(contours[i])
                    # Compute centroid
            if imageMoments["m00"] != 0:
                cx = int(imageMoments['m10']/imageMoments['m00'])
                cy = int(imageMoments['m01']/imageMoments['m00'])
            else:
                # set values as what you need in the situation
                cx, cy = 0, 0

            centres.append((cx, cy))
            cv2.circle(bgrImage, centres[-1], 3, (0, 255, 0), -1)


        cv2.imshow("centroid", bgrImage)

        if cv2.waitKey(1000) & 0xFF == ord ('q'):
            break
    video_capture.release()
    cv2.destroyAllWindows()
