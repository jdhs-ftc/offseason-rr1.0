package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeDetectorPipeline extends OpenCvPipeline {

    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private Mat contourImage = new Mat();
    private List<Rect> orangeRectangles = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {

        // Convert the input image to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Threshold the image to find orange objects
        Scalar minOrange = new Scalar(0, 150, 150);
        Scalar maxOrange = new Scalar(50, 255, 255);
        Core.inRange(hsvImage, minOrange, maxOrange, mask);

        // Find contours in the mask image
        Imgproc.findContours(mask, new ArrayList<>(), contourImage, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear the list of orange rectangles from the previous frame
        orangeRectangles.clear();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, contourImage, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Loop over all contours found in the mask image
        for (int i = 0; i < contours.size(); i++) {
            Rect rectangle = Imgproc.boundingRect(contours.get(i));
            double aspectRatio = rectangle.width / (double) rectangle.height;
            if (aspectRatio > 0.5 && aspectRatio < 2.0) {
                orangeRectangles.add(rectangle);
            }
        }

        // Draw rectangles around the orange objects in the input image
        for (Rect rectangle : orangeRectangles) {
            Imgproc.rectangle(input, rectangle.tl(), rectangle.br(), new Scalar(0, 255, 0), 2);
        }

        // Return the input image with rectangles drawn around the orange objects
        return input;
    }

    public List<Point> getCones() {
        List<Point> orangeCones = new ArrayList<>();
        for (Rect rectangle : orangeRectangles) {
            orangeCones.add(new Point(rectangle.x + rectangle.width / 2.0, rectangle.y + rectangle.height / 2.0));
        }
        return orangeCones;
    }

}
