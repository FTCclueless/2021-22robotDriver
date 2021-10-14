package org.firstinspires.ftc.teamcode.drive.SetUp;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "SetUp")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double lockHeadAngle = 0;


        while (!isStopRequested()) {
            drive.update();
            double forward = gamepad1.left_stick_y * -0.4;
            double left = gamepad1.left_stick_x * 0.6;
            double turn = gamepad1.right_stick_x * 0.35;

            boolean lockHeading = true;
            if (Math.abs(turn) > 0.01){ lockHeading = false; }
            if (lockHeading){
                double turnVal = drive.currentPose.getHeading()-lockHeadAngle;
                if (Math.abs(turnVal) >= 0.08){
                    turn += turnVal;
                }
            }
            else {lockHeadAngle = drive.currentPose.getHeading();}

            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("X", drive.currentPose.getX());
            telemetry.addData("Y", drive.currentPose.getY());
            telemetry.addData("Heading", drive.currentPose.getHeading());

            telemetry.update();

        }
    }
}
