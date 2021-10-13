package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.Encoder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class trackWidth4Pods extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("instructions1", "rotate the robot 8 rotations to the left");
        telemetry.addData("instructions2", "then clock the a-button");
        telemetry.update();
        waitForStart();
        Encoder[] encoders = new Encoder[4];
        encoders[0] = new Encoder(new Vector2d(0,0),1.0);
        encoders[1] = new Encoder(new Vector2d(0,0),-1.0);
        encoders[2] = new Encoder(new Vector2d(0,0),1.0);
        encoders[3] = new Encoder(new Vector2d(0,0),-1.0);
        while (!isStopRequested() && !gamepad1.a) {
            double forward = gamepad1.right_stick_y * -0.4;
            double left = gamepad1.right_stick_x * 0.6;
            double turn = gamepad1.left_stick_x * 0.35;
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
        }
        telemetry.addData("instructions3", "rotate the robot 8 rotations to the right");
        telemetry.addData("instructions4", "then clock the b-button");
        telemetry.update();
        for (int i = 0; i < drive.encoders.length; i ++){
            encoders[i].update(drive.encoders[i]);
        }
        double frontHeadingLeft = encoders[0].getDelta() - encoders[1].getDelta();
        double sideHeadingLeft = encoders[2].getDelta() - encoders[3].getDelta();
        while (!isStopRequested() && !gamepad1.b) {
            double forward = gamepad1.right_stick_y * -0.4;
            double left = gamepad1.right_stick_x * 0.6;
            double turn = gamepad1.left_stick_x * 0.35;
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
        }
        for (int i = 0; i < drive.encoders.length; i ++){
            encoders[i].update(drive.encoders[i]);
        }
        double frontHeadingRight = encoders[0].getDelta() - encoders[1].getDelta();
        double sideHeadingRight = encoders[2].getDelta() - encoders[3].getDelta();
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
        double deltaBack = encoders[2].getDelta();
        double deltaFront = encoders[3].getDelta();
        double y = (-1.0*(frontHeadingRight/rotationRight)*deltaRight)/(deltaLeft-deltaRight);
        double x = (-1.0*(sideHeadingRight/rotationRight)*deltaBack)/(deltaFront-deltaBack);
        double averageTrackWidth = (frontHeadingLeft/rotationLeft + frontHeadingRight/rotationRight)/2.0;
        double averageTrackHeight = (sideHeadingRight/rotationRight + sideHeadingLeft/rotationLeft)/2.0;
        telemetry.addData("averageTrackWidth", averageTrackWidth);
        telemetry.addData("averageTrackHeight", averageTrackHeight);
        telemetry.addData("right y",y);
        telemetry.addData("left y",y-averageTrackWidth);
        telemetry.addData("front x",x);
        telemetry.addData("back x",x-averageTrackHeight);
        telemetry.update();
        while (!isStopRequested()) {}

    }
}

