package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Outreach Drive")
public class outreachTeleop extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        drive.servos.get(5).setPosition(0.237);

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(12,66,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        drive.servos.get(0).setPosition(drive.rightIntakeDrop);
        drive.servos.get(1).setPosition(drive.leftIntakeDrop);

        drive.intakeCase = -1;

        while (!isStopRequested()) {
            drive.update();
            if (!gamepad2.a) {
                double forward = gamepad1.left_stick_y * -0.4;
                double left = gamepad1.left_stick_x * 0.4;
                double turn = gamepad1.right_stick_x * 0.35;
                double p1 = forward + left + turn;
                double p2 = forward - left + turn;
                double p3 = forward + left - turn;
                double p4 = forward - left - turn;
                drive.pinMotorPowers(p1, p2, p3, p4);
                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                if (a ^ b){
                    if (a){
                        drive.intake.setPower(-1);
                    }
                    else {
                        drive.intake.setPower(1);
                    }
                }
                else {
                    drive.intake.setPower(0);
                }
            }
            else {
                drive.setMotorPowers(0,0,0,0);
                drive.intake.setPower(0);
            }
        }
    }
}
