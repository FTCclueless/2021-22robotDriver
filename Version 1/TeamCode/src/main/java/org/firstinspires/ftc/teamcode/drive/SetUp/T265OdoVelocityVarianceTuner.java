package org.firstinspires.ftc.teamcode.drive.SetUp;

import static org.firstinspires.ftc.teamcode.drive.T265.slmra;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.T265;

import java.util.ArrayList;

@Config
@TeleOp(group = "SetUp")
public class T265OdoVelocityVarianceTuner extends LinearOpMode {

    public static double DISTANCE = 100;
    public static double FULL_SPEED_DIST = 0;
    public static double POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.trajectorySequenceRunner.initT265Robot();
        T265.T265Init(new Pose2d(-8.3,-1.5),100, hardwareMap.appContext);
        T265.start();

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

            Pose2d t265Estimate = T265.getPoseEstimate();
            T265.sendOdometry(drive.relCurrentVelocity);
            drive.trajectorySequenceRunner.updateT265(t265Estimate);
            Pose2d a = T265.getRelVelocity();

            if (drive.currentPose.getX() >= FULL_SPEED_DIST){
                double speed = drive.relCurrentVelocity.getX();
                double time = (System.nanoTime()-start)/1000000000.0;

                sumPose += drive.currentPose.getX();
                timeSum += time;

                pose.add(drive.currentPose.getX());
                times.add(time);
                vel.add(speed);
            }
            telemetry.addData("Speed X",a.getX());
            telemetry.addData("Speed Y",a.getY());
            telemetry.update();
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
        double variance = 0;
        for (int i = 0; i < vel.size(); i ++){
            variance += Math.pow(averageSpeed-vel.get(i),2);
        }
        variance /= ((double)pose.size()-1.0);
        telemetry.addData("variance",variance);
        telemetry.update();
        while (opModeIsActive()){ }
    }
}
