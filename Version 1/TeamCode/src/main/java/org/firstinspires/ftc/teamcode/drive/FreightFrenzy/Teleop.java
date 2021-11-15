package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        double lockHeadAngle = 0;

        drive.startIntake(true);
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;

        while (!isStopRequested()) {
            drive.update();
            double forward = gamepad1.left_stick_y;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            turn *= 0.35;
            if (!gamepad1.left_stick_button){
                forward *= - 0.4;
                left *= 0.6;
            }
            boolean lockHeading = true;
            if (Math.abs(turn) > 0.01){ lockHeading = false; }
            if (lockHeading){
                double turnVal = drive.currentPose.getHeading()-lockHeadAngle;
                if (Math.abs(turnVal) >= 0.08){
                    turn += turnVal + 0.04*Math.signum(turnVal);
                }
            }
            else {lockHeadAngle = drive.currentPose.getHeading();}
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

        }
    }
}
