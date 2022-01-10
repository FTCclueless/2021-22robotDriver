package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Auto")
public class EasyOpenCVDetectionTest extends LinearOpMode {
    OpenCvCamera camera;
    EasyOpenCVPipeline pipeline;
    double detectionX;
    double previousDetectionX = -1;
    int previousDetectionLoopsCounter = 0;

    int RANDOMIZATION = 3;  //default value=3 (shipping hub high level)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVPipeline();

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
        });

        // The INIT-loop: This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested()) {
            detectionX = pipeline.getLatestDetections();
            telemetry.addLine("TSE X = " + detectionX);
            if (detectionX > 0) {   //Not nothing was detected
                previousDetectionX = detectionX;
                previousDetectionLoopsCounter = 0;
            }
            if (previousDetectionLoopsCounter > 25) {
                previousDetectionX = -1;
                previousDetectionLoopsCounter = 0;
            }
            previousDetectionLoopsCounter++;
        }

        // BELOW STUFF HAPPENS AFTER START IS PRESSED
        //TODO: figure out the correct thresholds using the telemetry (123 and 234 are fillers)
        if (detectionX < 0 && previousDetectionX < 0) { //This means nothing was recently detected
            RANDOMIZATION = 3;
        }
        else {
            if (detectionX < 0 && previousDetectionX > 0) { //Something was detected in the last few frames
                detectionX = previousDetectionX;
            }

            //Logic for Blue Alliance
            if (detectionX > 234 || detectionX < 0) {
                RANDOMIZATION = 3;
            }
            else if (detectionX < 234 && detectionX > 123) {
                RANDOMIZATION = 2;
            }
            else {
                RANDOMIZATION = 1;
            }
        }
        telemetry.addLine("RANDOMIZATION: " + RANDOMIZATION);
        telemetry.update();


        // temporary; prevent auto from ending immediately
        while (opModeIsActive()) {
            sleep(20);
        }
    }
}
