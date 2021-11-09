package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "SetUp")
public class motorPIDTuner extends LinearOpMode {
    public static PIDFCoefficients V4BAR_PID;
    public static PIDFCoefficients TURRET_PID;
    public static PIDFCoefficients SLIDES_PID;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetAssemblies();
        waitForStart();
        String state = "idle";
        V4BAR_PID  = drive.v4bar.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        TURRET_PID = drive.turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        SLIDES_PID = drive.slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        double targetPos = 0;
        while (!isStopRequested()) {
            drive.v4bar.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, V4BAR_PID);
            drive.turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, TURRET_PID);
            drive.slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, SLIDES_PID);
            if (gamepad1.a){
                state = "idle";
            }
            if (gamepad1.b){
                state = "v4bar";
                drive.v4bar.setPower(1);
                drive.slides.setPower(0);
                drive.turret.setPower(0);
                drive.v4bar.setTargetPosition((int)(Math.toRadians(15)*drive.v4barTickToRadians));
            }
            if (gamepad1.x){
                state = "slides";
                drive.v4bar.setPower(0);
                drive.slides.setPower(1);
                drive.turret.setPower(0);
                drive.slides.setTargetPosition((int)(5*drive.slideTickToInch));
            }
            if (gamepad1.y){
                state = "turret";
                drive.v4bar.setPower(0);
                drive.slides.setPower(0);
                drive.turret.setPower(1);
                drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.slideTickToInch));
            }
            double currentPos = 0;
            switch (state){
                case "idle":
                    break;
                case "v4bar":
                    currentPos = Math.toDegrees(drive.v4barOrientation);
                    if(Math.abs(drive.v4barOrientation - Math.toRadians(15)) <= Math.toRadians(1)){
                        drive.v4bar.setPower(1);
                        drive.v4bar.setTargetPosition((int)(Math.toRadians(180)*drive.v4barTickToRadians));
                        targetPos = 180;
                    }
                    if(Math.abs(drive.v4barOrientation - Math.toRadians(180)) <= Math.toRadians(1)){
                        drive.v4bar.setPower(1);
                        drive.v4bar.setTargetPosition((int)(Math.toRadians(15)*drive.v4barTickToRadians));
                        targetPos = 15;
                    }
                    break;
                case "slides":
                    currentPos = drive.slideExtensionLength;
                    if(Math.abs(drive.slideExtensionLength - 5) <= 0.2){
                        drive.slides.setPower(1);
                        drive.slides.setTargetPosition((int)(20*drive.slideTickToInch));
                        targetPos = 20;
                    }
                    if(Math.abs(drive.slideExtensionLength - 20) <= 0.2){
                        drive.slides.setPower(1);
                        drive.slides.setTargetPosition((int)(5*drive.slideTickToInch));
                        targetPos = 5;
                    }
                    break;
                case "turret":
                    currentPos = Math.toDegrees(drive.turretHeading);
                    if(Math.abs(drive.turretHeading - Math.toRadians(30)) <= 0.2){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.slideTickToInch));
                        targetPos = 30;
                    }
                    if(Math.abs(drive.turretHeading - Math.toRadians(-30)) <= 0.2){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(30)*drive.slideTickToInch));
                        targetPos = -30;
                    }
                    break;
            }

            telemetry.addData("Motor", state);
            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.update();
            drive.update();
        }
    }
}
