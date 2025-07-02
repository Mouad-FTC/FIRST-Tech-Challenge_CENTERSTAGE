package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CameraPipeline extends OpenCvPipeline {
     Telemetry telemetry;
     Mat mat = new Mat();
     LOCATION position;
     public enum LOCATION {
         LEFT,
         RIGHT,
         CENTER
     }
     public  CameraPipeline (Telemetry t){
         telemetry = t;
     }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(100, 150, 50);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Scalar lowerRed = new Scalar(0, 70, 50);
        Scalar upperRed = new Scalar(10, 255, 255);

        Mat maskBlue = new Mat();
        Mat maskRed = new Mat();
        Core.inRange(mat, lowerBlue, upperBlue, maskBlue);
        Core.inRange(mat, lowerRed, upperRed, maskRed);

        Mat mask = new Mat();
        Core.bitwise_or(maskBlue, maskRed, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }

        if (bestRect != null) {
            Imgproc.rectangle(input, bestRect.tl(), bestRect.br(), new Scalar(0, 255, 0), 2);
            double centerX = bestRect.x + (bestRect.width / 2);

            if (centerX < input.width() / 3) {
                position = LOCATION.LEFT;
                telemetry.addData("Position", position);
            } else if (centerX > input.width() * 2 / 3) {
                position = LOCATION.RIGHT;
                telemetry.addData("Position", position);
            } else {
                position = LOCATION.CENTER;
                telemetry.addData("Position", position);
            }
        }

        telemetry.update();

        mat.release();
        maskBlue.release();
        maskRed.release();
        mask.release();
        input.release();

        return mat;
    }
    public LOCATION getLocation(){
        return position;
    }
}
