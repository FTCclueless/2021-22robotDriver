package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(group = "Auto")
public class AprilTagAutonomousInitDetectionTest extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics (UNITS ARE PIXELS). Calibration for C270 webcam at 640x480
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.0762;    //0.166

    int ID_TAG_OF_INTEREST = 17; // Tag ID 17 from the 36h11 family
    boolean tagOfInterestFound;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Do nothing
            }
        });

        telemetry.setMsTransmissionInterval(50);

        // The INIT-loop: This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested()) {
            //Detecting AprilTags
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            tagOfInterestFound = false;
            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    tagOfInterestFound = true;
                    break;
                }
            }

            if (tagOfInterestFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else if (tagOfInterest != null) {   //this means a the tag of interest was found previously, but not anymore
                telemetry.addLine("Tag no longer in view; previously found at:");
                tagToTelemetry(tagOfInterest);
            }
            else {
                telemetry.addLine("No Tag.");
            }

            telemetry.update();
            sleep(20);
        }


        // BELOW STUFF HAPPENS AFTER START IS PRESSED

        /* Update the telemetry with tag data */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag detected.");
            telemetry.update();
        }

        /* Autonomous Routine */
        if(tagOfInterest == null) {
            //AUTONOMOUS ROUTINE ASSUMING TAG HAS not BEEN DETECTED
        }
        else {
            //AUTONOMOUS ROUTINE ASSUMING TAG HAS BEEN DETECTED

            if(tagOfInterest.pose.x <= 20) {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50) {
                // do something else
            }
        }


        // temporary; prevent auto from ending immediately
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meters", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meters", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meters", detection.pose.z));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}