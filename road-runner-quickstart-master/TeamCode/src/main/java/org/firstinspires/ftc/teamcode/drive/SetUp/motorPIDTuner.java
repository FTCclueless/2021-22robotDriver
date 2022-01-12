package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "SetUp")
public class motorPIDTuner extends LinearOpMode {
    public static double tF = 32767.0 / (1150.0 / 60.0 * 145.1);
    public static double tP = tF * 0.1 + 0.2;
    public static double tI = tF * 0.01;
    public static double tD = 0;
    public static double tPP = 15;
    public static double sF = 32767.0 / (223.0 / 60.0 * 751.83);
    public static double sP = sF * 0.1;
    public static double sI = sF * 0.01;
    public static double sD = 0;
    public static double sPP = 20;
    long time = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetAssemblies();
        waitForStart();
        String state = "idle";
        long start = System.currentTimeMillis();
        double targetPos = 0;
        drive.slidesCase = -1;
        double offset = 1;
        drive.expansion2 = true;
        while (!isStopRequested()) {

            drive.turret.setVelocityPIDFCoefficients(tP,tI,tD,tF);
            drive.turret.setPositionPIDFCoefficients(tPP);
            drive.slides.setVelocityPIDFCoefficients(sP,sI,sD,sF);
            drive.slides.setPositionPIDFCoefficients(sPP);
            drive.slides2.setVelocityPIDFCoefficients(sP,sI,sD,sF);
            drive.slides2.setPositionPIDFCoefficients(sPP);

            if (gamepad1.a){
                state = "idle";
            }
            if (gamepad1.x){
                state = "slides";
                drive.turret.setPower(0);
                start = System.currentTimeMillis();
                targetPos = 5;
            }
            if (gamepad1.y){
                state = "turret";
                drive.slides.setPower(0);
                drive.slides2.setPower(0);
                drive.turret.setPower(1);
                start = System.currentTimeMillis();
                drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.turretTickToRadians));
            }
            double currentPos = 0;
            switch (state){
                case "idle":
                    break;
                case "slides":
                    currentPos = drive.slideExtensionLength;
                    if(Math.abs(drive.slideExtensionLength - 5) <= offset && System.currentTimeMillis() - start >= time){
                        targetPos = 20;
                        start = System.currentTimeMillis();
                    }
                    if(Math.abs(drive.slideExtensionLength - 20) <= offset && System.currentTimeMillis() - start >= time){
                        targetPos = 5;
                        start = System.currentTimeMillis();
                    }
                    drive.setSlidesLength(targetPos);
                    break;
                case "turret":
                    currentPos = Math.toDegrees(drive.turretHeading);
                    if(Math.abs(drive.turretHeading - Math.toRadians(30)) <= 0.2 && System.currentTimeMillis() - start >= time){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(-30)*drive.turretTickToRadians));
                        targetPos = -30;
                        start = System.currentTimeMillis();
                    }
                    if(Math.abs(drive.turretHeading - Math.toRadians(-30)) <= 0.2 && System.currentTimeMillis() - start >= time){
                        drive.turret.setPower(1);
                        drive.turret.setTargetPosition((int)(Math.toRadians(30)*drive.turretTickToRadians));
                        targetPos = 30;
                        start = System.currentTimeMillis();
                    }
                    break;
            }

            telemetry.addData("Motor", state);
            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("pos", drive.slides.getCurrentPosition());
            telemetry.addData("pos2", drive.slides2.getCurrentPosition());
            telemetry.addData("error", targetPos-currentPos);
            telemetry.update();
            drive.trajectorySequenceRunner.error = targetPos-currentPos;
            drive.update();
        }
    }
}
