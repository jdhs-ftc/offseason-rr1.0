package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PoleChatGPTPipeline extends OpenCvPipeline {

    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private Mat contourImage = new Mat();
    private List<Rect> orangeRectangles = new ArrayList<>();
    private Mat maskedInputMat = new Mat();
    public Scalar minOrange = new Scalar(0, 150, 150);
    public Scalar maxOrange = new Scalar(40, 255, 255);

    @Override
    public Mat processFrame(Mat input) {

        // Convert the input image to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Threshold the image to find orange objects

        Core.inRange(hsvImage, minOrange, maxOrange, mask);

        // Find contours in the mask image


        // Clear the list of orange rectangles from the previous frame
        orangeRectangles.clear();
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, mask);
        Imgproc.GaussianBlur(maskedInputMat, maskedInputMat, new org.opencv.core.Size(5, 5), 0);



        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, contourImage, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(maskedInputMat, contours, -1, new Scalar(0, 255, 0), 2);
        // Loop over all contours found in the mask image
        for (int i = 0; i < contours.size(); i++) {
            Rect rectangle = Imgproc.boundingRect(contours.get(i));
            double aspectRatio = rectangle.width / (double) rectangle.height;
            if (aspectRatio < 1 && rectangle.width > 10 && rectangle.height > 10) {
                orangeRectangles.add(rectangle);
            }
        }

        // Draw rectangles around the orange objects in the input image
        for (Rect rectangle : orangeRectangles) {
            Imgproc.rectangle(maskedInputMat, rectangle.tl(), rectangle.br(), new Scalar(0, 255, 0), 2);
        }


        return maskedInputMat;
    }

    public List<Point> getPoles() {
        List<Point> orangePoles = new ArrayList<>();
        for (Rect rectangle : orangeRectangles) {
            orangePoles.add(new Point(rectangle.x + rectangle.width / 2.0, rectangle.y + rectangle.height / 2.0));
        }
        return orangePoles;
    }

}
