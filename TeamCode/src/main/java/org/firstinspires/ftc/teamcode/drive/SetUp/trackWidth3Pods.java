package org.firstinspires.ftc.teamcode.drive.SetUp;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Encoder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "SetUp")
public class trackWidth3Pods extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("instructions1", "rotate the robot 8 rotations to the left");
        telemetry.addData("instructions2", "then clock the a-button");
        telemetry.update();
        waitForStart();
        Encoder[] encoders = new Encoder[3];
        encoders[0] = new Encoder(new Vector2d(0,0), -1.0);
        encoders[1] = new Encoder(new Vector2d(0,0), 1.0);
        encoders[2] = new Encoder(new Vector2d(0,0), -1.0);
        for (int i = 0; i < drive.encoders.length; i ++){
            Log.e("Encoder",""+drive.encoders[i]);
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
        telemetry.addData("instructions3", "rotate the robot 8 rotations to the right");
        telemetry.addData("instructions4", "then clock the b-button");
        telemetry.update();
        for (int i = 0; i < drive.encoders.length; i ++){
            Log.e("Encoder",""+drive.encoders[i]);
            encoders[i].update(drive.encoders[i]);
        }
        double frontHeadingLeft = encoders[0].getDelta() - encoders[1].getDelta();
        double sideHeadingLeft = encoders[2].getDelta();
        while (!isStopRequested() && !gamepad1.b) {
            double turn = gamepad1.left_stick_x * 0.35;
            double p1 = turn;
            double p2 = turn;
            double p3 = turn * -1;
            double p4 = turn * -1;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
        }
        for (int i = 0; i < drive.encoders.length; i ++){
            Log.e("Encoder2",""+drive.encoders[i]);
            encoders[i].update(drive.encoders[i]);
        }
        double frontHeadingRight = encoders[0].getDelta() - encoders[1].getDelta();
        double sideHeadingRight = encoders[2].getDelta();
        double rotationLeft = Math.toRadians(360 * 8);
        double rotationRight = Math.toRadians(360 * -8);
        telemetry.addData("trackWidthFrontLeft",    frontHeadingLeft/rotationLeft);
        telemetry.addData("trackWidthSideLeft",     sideHeadingLeft/rotationLeft);
        telemetry.addData("trackWidthFrontRight",   frontHeadingRight/rotationRight);
        telemetry.addData("trackWidthSideRight",    sideHeadingRight/rotationRight);
        telemetry.update();
        drive.setMotorPowers(0,0,0,0);
        while (!isStopRequested() && !gamepad1.a) {}
        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double y = (-1.0*(frontHeadingRight/rotationRight)*deltaRight)/(deltaLeft-deltaRight);
        double averageTrackWidth = (frontHeadingLeft/rotationLeft + frontHeadingRight/rotationRight)/2.0;
        double averageTrackHeight = (sideHeadingRight/rotationRight + sideHeadingLeft/rotationLeft)/2.0;
        telemetry.addData("averageTrackWidth", averageTrackWidth);
        telemetry.addData("right y",y);
        telemetry.addData("left y",y-averageTrackWidth);
        telemetry.addData("horizontal x",averageTrackHeight);
        telemetry.update();
        while (!isStopRequested()) {}

    }
}

