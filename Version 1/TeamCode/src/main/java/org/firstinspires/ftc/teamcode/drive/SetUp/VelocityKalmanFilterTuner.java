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
    public static double FULL_SPEED_DIST = 10;
    public static double POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested() && !gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && gamepad1.y) {
            idle();
        }
        ArrayList<Double> pose = new ArrayList<Double>();
        ArrayList<Double> vel = new ArrayList<Double>();
        ArrayList<Double> times = new ArrayList<Double>();
        double sumPose = 0;
        double timeSum = 0;
        long start = System.nanoTime();

        while (!isStopRequested() && drive.currentPose.getX() <= DISTANCE) {
            drive.pinMotorPowers(POWER,POWER,POWER,POWER);

            drive.update();

            double speed = drive.relCurrentVelocity.getX();
            vel.add(speed);

            if (drive.currentPose.getX() >= FULL_SPEED_DIST){
                double time = (System.nanoTime()-start)/1000000000.0;

                sumPose += drive.currentPose.getX();
                timeSum += time;

                pose.add(drive.currentPose.getX());
                times.add(time);
            }
        }
        drive.setMotorPowers(0,0,0,0);

        double averageTime = timeSum/times.size();
        double averagePose = sumPose/pose.size();
        double SPose = 0;
        double STime = 0;
        for (int i = 0; i < pose.size(); i ++){
            SPose += Math.pow(pose.get(i)-averagePose,2);
            STime += Math.pow(times.get(i)-averageTime,2);
        }
        SPose = Math.sqrt(SPose/((double)pose.size()-1.0));
        STime = Math.sqrt(STime/((double)times.size()-1.0));
        double r = 0;
        for (int i = 0; i < pose.size(); i ++){
            double ZPose = (pose.get(i)-averagePose)/SPose;
            double ZTime = (times.get(i)-averageTime)/STime;
            r += ZPose * ZTime;
        }
        r = Math.sqrt(r/((double)pose.size()-1.0));
        double averageSpeed = SPose/STime * r;

        double minSDev = -1;
        double bestGain = 0;
        double fidelity = 1000.0;
        for (int i = 0; i < fidelity; i ++){
            double w = ((double)i+1.0)/fidelity;
            ArrayList<Double> v = new ArrayList<Double>();
            v.add(vel.get(0));
            for (int j = 1; j < vel.size(); j ++){
                v.add(v.get(v.size()-1)*(1.0-w) + vel.get(j)*w);
            }
            double error = 0;
            for (int j = 0; j < v.size(); j ++){
                if (j >= vel.size()-pose.size()) {
                    error += Math.pow(v.get(j) - averageSpeed, 2);
                }
            }
            double SDev = Math.sqrt(error/(v.size()-1.0));
            if (minSDev == -1 || SDev < minSDev){
                minSDev = SDev;
                bestGain = w;
            }
        }
        telemetry.addData("Best SDev ", minSDev);
        telemetry.addData("best Gain ", bestGain);
        telemetry.update();
        while (opModeIsActive()){ }
    }
}
