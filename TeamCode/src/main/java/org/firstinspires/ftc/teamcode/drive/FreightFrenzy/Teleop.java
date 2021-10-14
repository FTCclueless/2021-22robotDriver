package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private ElapsedTime runtime = new ElapsedTime();

    private boolean slowMode = false;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            double forward = gamepad1.left_stick_y * -0.75;
            double left = gamepad1.left_stick_x * 0.75;
            double turn = gamepad1.right_stick_x * 0.75;

            if (slowMode == true) {
                forward *= -0.45;
                left *= 0.45;
                turn *= 0.45;
            }
            else{
                forward *= -0.75;
                left *= 0.75;
                turn *= 0.75;
            }

            drive.setMotorPowers((forward + left + turn), (forward - left + turn), (forward + left - turn), (forward - left - turn));
            drive.update();

            telemetry.addData("speedX", drive.currentVelocity.getX());
            telemetry.addData("speedY", drive.currentVelocity.getY());
            telemetry.addData("speedHeading", drive.currentVelocity.getHeading());
            telemetry.update();

        }
    }
}
