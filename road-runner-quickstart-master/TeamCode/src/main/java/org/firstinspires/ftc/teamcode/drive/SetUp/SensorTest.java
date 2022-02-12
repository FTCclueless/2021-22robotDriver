package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(group = "SetUp")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //DigitalChannel a = hardwareMap.digitalChannel.get("test");
        //a.setMode(DigitalChannel.Mode.OUTPUT);
        //a.setState(true);

        drive.servos.get(0).setPosition(drive.rightIntakeDrop);
        drive.servos.get(1).setPosition(drive.leftIntakeDrop);

        waitForStart();
        //a.setState(false);

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
                    //turn += turnVal;
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
            //Orientation orientation = drive.imu.getAngularOrientation();
            //double angle = drive.imu.getAngularOrientation().secondAngle;
            //double current = drive.rightRear.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
           // double voltage = drive.getBatteryVoltage();
            double val1 = drive.leftWall.alpha();
            double val2 = drive.rightWall.alpha();
            double elapsedTimeRLI = (System.nanoTime() - startTime)/1000000.0;
            totalElapsedRLITime += elapsedTimeRLI;
            loops ++;

            //telemetry.addData("Reflected Light Intensity", rli);
            //telemetry.addData("Imu heading", orientation.secondAngle);
            //telemetry.addData("Imu heading", angle);
            //telemetry.addData("currentMotor", current);
            //telemetry.addData("volatage", drive.getBatteryVoltage());



            telemetry.addData("left Wall", val1);
            telemetry.addData("right Wall", val2);
            telemetry.addData("left Color",drive.leftWall.argb()/1000000.0);
            telemetry.addData("right Color",drive.rightWall.argb()/1000000.0);
            telemetry.addData("left Intake", drive.leftIntake.getVoltage());
            telemetry.addData("right Intake", drive.rightIntake.getVoltage());
            telemetry.addData("depo", drive.depositSensor.getVoltage());
            telemetry.addData("dist", drive.dist.getVoltage());
            telemetry.addData("mag", drive.mag1.getVoltage());
            telemetry.addData("rli", rli);
            //telemetry.addData("ReadTime", elapsedTimeRLI);
            //telemetry.addData("Average ReadTime", totalElapsedRLITime/(double)loops);
            telemetry.update();

        }
    }
}
