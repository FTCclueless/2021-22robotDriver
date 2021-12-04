package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.widget.ToggleButton;

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
@TeleOp(group = "Comp Drive")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(12,66,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        int hub = 1;
        Pose2d endpoint = new Pose2d();
        Pose2d hubLocation = new Pose2d();
        double height = 0;

        boolean intake = true;

        ButtonToggle b = new ButtonToggle();

        while (!isStopRequested()) {
            switch(hub) {
                case 1: endpoint = new Pose2d(64, 12, Math.toRadians(90));
                        hubLocation = new Pose2d(48, 0);
                        height = 6;
                        intake = true;
                            break;
                case 2: endpoint = new Pose2d(12, 64, Math.toRadians(0));
                        hubLocation = new Pose2d(-12, 24);
                        height = 20;
                        intake = false;
                            break;
            }

            drive.update();

            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * 0.35;
            if (!gamepad1.left_stick_button){
                forward *= 0.4;
                left *= 0.5;
            }

            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            if(gamepad1.right_bumper) {
                drive.startDeposit(endpoint, hubLocation, height);
            }

            if(gamepad1.right_trigger >= 0.5) {
                drive.startIntake(intake);
            }

        }
    }
}
