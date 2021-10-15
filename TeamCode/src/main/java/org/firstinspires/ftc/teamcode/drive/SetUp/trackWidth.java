package org.firstinspires.ftc.teamcode.drive.SetUp;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Encoder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "SetUp")
public class trackWidth extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("instructions1", "rotate the robot 8 rotations to the left");
        telemetry.addData("instructions2", "then clock the a-button");
        telemetry.update();
        waitForStart();
        int numEncoders = drive.encoders.length;
        Encoder[] encoders = new Encoder[numEncoders];
        for (int i = 0; i < numEncoders; i ++){
            encoders[i] = new Encoder(new Vector2d(0,0), drive.localizer.encoders[i].scaleFactor);
        }
        double[] left = new double[numEncoders];
        double[] right = new double[numEncoders];
        double rotationLeft = Math.toRadians(360 * 8);
        double rotationRight = Math.toRadians(360 * -8);

        for (int i = 0; i < numEncoders; i ++){
            encoders[i].update(drive.encoders[i]);
        }
        while (!isStopRequested() && !gamepad1.a) {
            double turn = gamepad1.left_stick_x * 0.35;
            double p1 = turn;
            double p2 = turn;
            double p3 = turn * -1;
            double p4 = turn * -1;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
        }
        telemetry.addData("instructions1", "rotate the robot 8 rotations to the right");
        telemetry.addData("instructions2", "then clock the b-button");
        telemetry.update();
        for (int i = 0; i < numEncoders; i ++){
            encoders[i].update(drive.encoders[i]);
            left[i] = encoders[i].getDelta()/rotationLeft;
        }
        while (!isStopRequested() && !gamepad1.b) {
            double turn = gamepad1.left_stick_x * 0.35;
            double p1 = turn;
            double p2 = turn;
            double p3 = turn * -1;
            double p4 = turn * -1;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
        }
        for (int i = 0; i < numEncoders; i ++){
            encoders[i].update(drive.encoders[i]);
            right[i] = encoders[i].getDelta()/rotationRight;
        }
        drive.setMotorPowers(0,0,0,0);

        telemetry.addData("right Y","T1 " + left[0] + "T2 " + right[0]);
        telemetry.addData("left Y" ,"T1 " + left[1] + "T2 " + right[1]);
        telemetry.addData("back X" ,"T1 " + left[2] + "T2 " + right[2]);
        if (numEncoders == 4){
            telemetry.addData("front X" ,"T1 " + left[3] + "T2 " + right[3]);
        }
        telemetry.addData("average Right Y",(left[0] + right[0])/2.0);
        telemetry.addData("average left Y" ,(left[1] + right[1])/2.0);
        telemetry.addData("average back X" ,(left[2] + right[2])/2.0);
        if (numEncoders == 4){
            telemetry.addData("average front X" ,(left[3] + right[3])/2.0);
        }
        telemetry.update();
        while (!isStopRequested()) {}

    }
}

