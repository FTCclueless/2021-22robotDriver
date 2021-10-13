package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double lockHeadAngle = 0;
        double totalElapsedRLITime = 0;
        int loops = 0;

        while (!isStopRequested()) {
            drive.update();
            double forward = gamepad1.right_stick_y * -0.4;
            double left = gamepad1.right_stick_x * 0.6;
            double turn = gamepad1.left_stick_x * 0.35;

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

            long startTime = System.nanoTime();
            int rli = drive.color.alpha();
            double elapsedTimeRLI = (System.nanoTime() - startTime)/1000000.0;
            totalElapsedRLITime += elapsedTimeRLI;
            loops ++;

            telemetry.addData("Reflected Light Intensity", rli);
            telemetry.addData("ReadTime", elapsedTimeRLI);
            telemetry.addData("Average ReadTime", elapsedTimeRLI/(double)loops);
            telemetry.update();

        }
    }
}
