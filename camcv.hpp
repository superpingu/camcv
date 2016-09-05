/* This library provides access to raspberry pi's native camera
 * (connected to the board's camera connector).
 */

#include <opencv/cv.hpp>

/* Initialize and configure camera. This should be called before any other
 * function of the library.
 * arguments :
 *      width and height: frame dimensions in pixel
 *      framerate: number of frame per second
 *      useRGB: 1 for BGR (instead of RGB for openCV compatibility), 0 for YUV
 *
 *  note : due to hardware limitation, U and V are undersampled (1 sample per
 *    2x2 pixel square) and then expanded to match required resolution */
int initCamera(int width, int height, int framerate, int useRGB);

/* Stop capture and destroy camera object. */
void destroyCamera();

/* get the latest frame taken by the camera as a 3 channel openCV matrix
 * The user have to free the buffer after using the returned frame */
cv::Mat getFrame();
