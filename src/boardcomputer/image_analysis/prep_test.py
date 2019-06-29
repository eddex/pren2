import cv2
import numpy as np

def prepare_image_for_processing(color_image, search_high_signals):
        # resized_image = color_image
        # resized_image = cv2.resize(color_image, (self.new_w, self.new_h), interpolation = cv2.INTER_CUBIC)

        # resize image to shape 416x416 by cutting off the edges
        # the images should already be 416x416 from the PiCamera.
        resized_image = []
        for row in color_image[:416]:
            resized_image.append(row[:416])
        
        #cv2.imwrite("c_resized.jpg", resized_image)

        # create all black 416x416 image
        canvas = np.full((416, 416, 3), 0)
        cv2.imwrite("canvas.jpg", canvas)
        if search_high_signals:
            # searching for info and start signals in upper half of the image
            canvas[:208, :416, :] = color_image[:208, :416]
            cv2.imwrite("canvas2.jpg", canvas)
        else:
            # stop signals are in the lower half of the image
            canvas[208:416, :416, :] = color_image[208:416, :416, :]
            cv2.imwrite("canvas3.jpg", canvas)

        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

        return prepimg

def prepare_image_for_processing_double_image(color_image_1, color_image_2, search_high_signals):

    # EXPECTATION: the images should already be 416x416 from the PiCamera.

    # create all green 416x416 image
    canvas = np.full((416, 416, 3), [0,255,0])
    
    if search_high_signals:
        # combine the 2 upper halfs of the image to one image
        canvas[:208, :416, :] = color_image_1[:208, :416]
        canvas[208:416, :416, :] = color_image_2[:208, :416]
    else:
        # combine the 2 lower halfs of the image to one image
        canvas[:208, :416, :] = color_image_1[208:416, :416, :]
        canvas[208:416, :416, :] = color_image_2[208:416, :416, :]

    prepimg = canvas
    prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
    prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

    cv2.imwrite("double.jpg", canvas)

    return prepimg

if __name__ == "__main__":
    img1 = cv2.imread("signal-2600.jpg")
    img2 = cv2.imread("signal-2601.jpg")
    prep_img = prepare_image_for_processing_double_image(img1, img2, True)
