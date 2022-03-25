package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Comp Drive")
public class PresentationTeleop extends LinearOpMode {
    SampleMecanumDrive drive;

    double side = -1;
    boolean intake = true;

    long start;

    Pose2d sharedHubEndpoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
    Pose2d allianceHubEndpoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.v4barOffset = Math.toRadians(-10);

        sharedHubEndpoint = new Pose2d(65.125, 16 * side, Math.toRadians(90) * side);
        allianceHubEndpoint = new Pose2d(12, 65.125 * side, Math.toRadians(0));

        Pose2d startingPose = new Pose2d(45,65.25 * side,0);
        drive.localizer.setPoseEstimate(startingPose);

        start = System.currentTimeMillis();

        drive.intakeCase = -1;
        drive.currentIntake = -1;

        while(opModeIsActive() && !isStarted()){
            drive.update();
            drive.setV4barOrientation(drive.v4barInterfaceAngle);
            drive.setTurretTarget(0);
            drive.setSlidesLength(drive.returnSlideLength, 0.4);
            drive.servos.get(0).setPosition(drive.rightIntakeMid);
            drive.servos.get(1).setPosition(drive.leftIntakeMid);
        }

        while (!isStopRequested()) {

            drive.update();

            drive.servos.get(7).setPosition(0.471);
            drive.servos.get(6).setPosition(1);

            drive.update();
            if (drive.intakeCase == -1) {
                drive.setV4barOrientation(drive.v4barInterfaceAngle);
                drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);
                drive.setSlidesLength(drive.returnSlideLength, 0.4);
                drive.servos.get(0).setPosition(drive.rightIntakeDrop);
                drive.servos.get(1).setPosition(drive.leftIntakeMid);
            }
            if (gamepad1.a){
                if (drive.intakeCase == -1) {
                    drive.intakeCase = 0;
                }
                drive.startIntake(intake);
            }
        }
    }
}
