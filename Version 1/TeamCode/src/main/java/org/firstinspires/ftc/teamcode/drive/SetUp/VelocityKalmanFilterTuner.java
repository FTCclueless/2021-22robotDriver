package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp(group = "SetUp")
public class VelocityKalmanFilterTuner extends LinearOpMode {

    public static double DISTANCE = 100;
    public static double FULL_SPEED_DIST = 20;
    public static double POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<Double> vel = new ArrayList<Double>();

        waitForStart();

        while (!isStopRequested() && !gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && drive.currentPose.getX() <= DISTANCE) {
            drive.update();
            if (drive.currentPose.getX() >= FULL_SPEED_DIST){
                vel.add(drive.relCurrentVelocity.getX());
            }
            drive.pinMotorPowers(POWER,POWER,POWER,POWER);
        }
        drive.setMotorPowers(0,0,0,0);
        for (int i = 0; i < 100; i ++){
            double w = ((double)i+1.0)/100.0;

        }
    }
}
