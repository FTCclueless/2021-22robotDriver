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
        boolean lastA1 = false;
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(12,66,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
        long startDuckTime = System.currentTimeMillis();
        double lockHeadAngle = 0;

        ButtonToggle b = new ButtonToggle();

        while (!isStopRequested()) {
            drive.update();

            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * 0.35;
            if (!gamepad1.left_stick_button){
                forward *= 0.4;
                left *= 0.5;
            }

            boolean a1 = gamepad1.a;
            if (a1 != lastA1 && a1){
                drive.startIntake(true);
                drive.startDeposit(new Pose2d(12,-64,0), new Pose2d(-12,-24),20);
            }
            lastA1 = a1;

            if (gamepad1.b){
                drive.deposit();
            }

            b.update(gamepad2.b);

            if (b.getToggleState()){
                drive.servos.get(7).setPosition(0.5);
                if (gamepad2.x && System.currentTimeMillis() - startDuckTime >= 1500 ){
                    startDuckTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - startDuckTime <= 1500){
                    drive.duckSpin.setPower(-1);
                }
                else {
                    drive.duckSpin.setPower(0);
                }
            }
            else {
                drive.servos.get(7).setPosition(1.0);
                drive.duckSpin.setPower(0);
            }

            boolean lockHeading = true;
            if (Math.abs(turn) > 0.01){ lockHeading = false; }
            if (lockHeading){
                double turnVal = drive.currentPose.getHeading()-lockHeadAngle;
                if (Math.abs(turnVal) >= 0.08){
                    //turn += turnVal + 0.04*Math.signum(turnVal);
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
