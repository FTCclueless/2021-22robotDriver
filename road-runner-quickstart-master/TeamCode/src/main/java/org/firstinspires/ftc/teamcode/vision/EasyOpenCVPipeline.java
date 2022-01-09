package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class EasyOpenCVPipeline extends OpenCvPipeline {
    Mat submat;
    double detectionX;  //x value of the location of the detection (value is -1 if no detections)

    @Override
    public void init(Mat firstFrame) {  //Stuff that only runs once
        submat = firstFrame.submat(100,380,0,640);  //Crop
    }

    @Override
    public Mat processFrame(Mat input) {    //Stuff that is run iteratively
        return null;
    }

    public double getLatestDetections() {
        return detectionX;
    }
}
