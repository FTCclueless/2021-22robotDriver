package org.firstinspires.ftc.teamcode.drive.SetUp;

import android.util.Log;

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
    public static double FULL_SPEED_DIST = 0;
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

        double minSDev = -1;
        double bestW = 0;
        for (int i = 0; i < 100; i ++){
            double w = ((double)i+1.0)/100.0;
            ArrayList<Double> v = new ArrayList<Double>();
            v.add(vel.get(0));
            double sum = 0;
            for (int j = 1; j < vel.size(); j ++){
                v.add(v.get(v.size()-1)*(1.0-w) + vel.get(j)*w);
                sum += v.get(v.size()-1);
            }
            double average = sum/v.size();
            double error = 0;
            for (int j = 0; j < v.size(); j ++){
                error += Math.pow(v.get(j)-average,2);
            }
            double SDev = Math.sqrt(error/v.size());
            Log.e(w + " ", SDev + " ");
            if (minSDev == -1 || SDev < minSDev){
                minSDev = SDev;
                bestW = w;
            }
        }
        telemetry.addData("Best SDev ", minSDev);
        telemetry.addData("best W ", bestW);
        telemetry.update();
        while (opModeIsActive()){ }
    }
}
