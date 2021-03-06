package org.firstinspires.ftc.teamcode.drive.SetUp;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.T265;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "SetUp")
public class T265Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,true,false);
        drive.trajectorySequenceRunner.initT265Robot();
        T265 a = new T265();
        a.T265Init(new Pose2d(-8.3,-1.5),1.8574, hardwareMap.appContext);
        a.start();
        long start = System.currentTimeMillis();
        waitForStart();
        double lockHeadAngle = 0;
        while (!isStopRequested()) {
            drive.update();
            if (System.currentTimeMillis() - start >= 5000) {
                Pose2d t265Estimate = a.getPoseEstimate();
                a.sendOdometry(drive.relCurrentVelocity);
                drive.trajectorySequenceRunner.updateT265(t265Estimate);
            }

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


            /*
            telemetry.addData("X", drive.currentPose.getX());
            telemetry.addData("Y", drive.currentPose.getY());
            telemetry.addData("Heading", drive.currentPose.getHeading());
            telemetry.addData("T265X", t265Estimate.getX());
            telemetry.addData("T265Y", t265Estimate.getY());
            telemetry.addData("T265Heading", t265Estimate.getHeading());
            telemetry.update();
             */

        }
    }
}
