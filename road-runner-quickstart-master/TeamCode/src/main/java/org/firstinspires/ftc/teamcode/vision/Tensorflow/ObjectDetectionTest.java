package org.firstinspires.ftc.teamcode.vision.Tensorflow;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.hardware.Camera;
import android.view.Surface;
import android.widget.FrameLayout;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.TFODModels.CAPSTONE_V1_MOBILETNET3;
import org.firstinspires.ftc.teamcode.vision.TFODModels.CAPSTONE_V2_MOBILETNET3;
import org.firstinspires.ftc.teamcode.vision.TFODModels.DetectionModel;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.List;

@TeleOp(name = "Object Detection Test", group = "Vision")
public class ObjectDetectionTest extends LinearOpMode {
    private static final DetectionModel TFOD_MODEL = new CAPSTONE_V2_MOBILETNET3();
    public void runOpMode() {
        ObjectDetector detector = new ObjectDetector(TFOD_MODEL, hardwareMap);
        detector.initialize();
        telemetry.addData("Loaded Model", TFOD_MODEL);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Loaded Model", TFOD_MODEL);

            List<Recognition> updatedRecognitions = detector.getDetections();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.addData("DetectionX", detector.getPositionX(ObjectDetector.MatchMode.DEFAULT));
                telemetry.update();
            }

            detector.autoAdjustZoom();
        }
    }

}
