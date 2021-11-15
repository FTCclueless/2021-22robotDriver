package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "SetUp")
public class motorPIDTuner extends LinearOpMode {
    PIDFCoefficients TURRET_PID;
    PIDFCoefficients SLIDES_PID;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetAssemblies();
        waitForStart();
        String state = "idle";
        TURRET_PID = drive.turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        SLIDES_PID = drive.slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        long start = System.currentTimeMillis();
        double targetPos = 0;
        drive.slidesCase = -1;
        while (!isStopRequested()) {
            if (gamepad1.a){
                state = "idle";
            }
            if (gamepad1.x){
                state = "slides";
                drive.slides.setPower(1);
                drive.turret.setPower(0);
                start = System.currentTimeMillis();
                drive.slides.setTargetPosition((int)(5*drive.slideTickToInch));
            }
            if (gamepad1.y){
                state = "turret";
                drive.slides.setPower(0);
                drive.turret.setPower(1);
                start = System.currentTimeMillis();
                drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.slideTickToInch));
            }
            double currentPos = 0;
            switch (state){
                case "idle":
                    break;
                case "slides":
                    currentPos = drive.slideExtensionLength;
                    if(Math.abs(drive.slideExtensionLength - 5) <= 0.2 && System.currentTimeMillis() - start >= 5000){
                        drive.slides.setPower(1);
                        drive.slides.setTargetPosition((int)(20*drive.slideTickToInch));
                        targetPos = 20;
                        start = System.currentTimeMillis();
                    }
                    if(Math.abs(drive.slideExtensionLength - 20) <= 0.2 && System.currentTimeMillis() - start >= 5000){
                        drive.slides.setPower(1);
                        drive.slides.setTargetPosition((int)(5*drive.slideTickToInch));
                        targetPos = 5;
                        start = System.currentTimeMillis();
                    }
                    break;
                case "turret":
                    currentPos = Math.toDegrees(drive.turretHeading);
                    if(Math.abs(drive.turretHeading - Math.toRadians(30)) <= 0.2 && System.currentTimeMillis() - start >= 5000){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.slideTickToInch));
                        targetPos = 30;
                        start = System.currentTimeMillis();
                    }
                    if(Math.abs(drive.turretHeading - Math.toRadians(-30)) <= 0.2 && System.currentTimeMillis() - start >= 5000){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(30)*drive.slideTickToInch));
                        targetPos = -30;
                        start = System.currentTimeMillis();
                    }
                    break;
            }

            telemetry.addData("Motor", state);
            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("error", targetPos-currentPos);
            telemetry.update();
            drive.trajectorySequenceRunner.error = targetPos-currentPos;
            drive.update();
        }
    }
}
