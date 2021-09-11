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
@TeleOp(group = "drive")
public class Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Controller 1
    private ButtonToggle button_rb = new ButtonToggle();

    //Controller 2

    private ButtonToggle buttonA1 = new ButtonToggle();

    private ButtonToggle buttonA2 = new ButtonToggle();
    private ButtonToggle buttonY2 = new ButtonToggle();

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

//            if(drive.controlMotors.update(gamepad1.right_bumper)) {
//                slowMode = !slowMode;
//            }

            if (slowMode == true) {
                forward = gamepad1.left_stick_y * -0.45;
                left = gamepad1.left_stick_x * 0.45;
                turn = gamepad1.right_stick_x * 0.45;
            }

            drive.slidesLift.update(gamepad2.y); // Controlling Linear Slides Motor
            drive.controlIntake.update(gamepad2.a); // Controlling Intake Motor
            drive.controlServo.update(gamepad1.b); // Controlling Dropper Servo
            drive.controlCapstoneServo.update(gamepad1.a);//Controlling Capstone Servo

            drive.setMotorPowers((forward + left + turn), (forward - left + turn), (forward + left - turn), (forward - left - turn));

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("speedX", drive.currentVelocity.getX());
            telemetry.addData("speedY", drive.currentVelocity.getY());
            telemetry.addData("speedHeading", drive.currentVelocity.getHeading());
            telemetry.update();

        }
    }
}
