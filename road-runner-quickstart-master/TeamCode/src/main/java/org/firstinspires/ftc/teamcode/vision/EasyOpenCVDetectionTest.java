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

            //TODO: figure out the correct thresholds using the telemetry (123 and 234 are fillers)
            if (detectionX > 234 || detectionX < 0) {
                RANDOMIZATION = 3;
            }
            else if (detectionX < 234 && detectionX > 123) {   //this means TSE was found previously, but not anymore
                RANDOMIZATION = 2;
            }
            else {
                RANDOMIZATION = 1;
            }
            telemetry.addLine("RANDOMIZATION: " + RANDOMIZATION);
            telemetry.update();
        }

        // BELOW STUFF HAPPENS AFTER START IS PRESSED

        // temporary; prevent auto from ending immediately
        while (opModeIsActive()) {
            sleep(20);
        }
    }
}
