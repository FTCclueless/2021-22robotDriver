package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class WearhouseAutoBlue extends LinearOpMode {
    Long start;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.resetAssemblies();

        Pose2d startingPose = new Pose2d(12,65.25,0);
        Pose2d endPoint = new Pose2d(12,64.75,0);

        //TODO: Implement ML here

        int capNum = 2;
        setUp(startingPose);
        drive.transferMineral = true;

        waitForStart();

        drive.update();

        drive.servos.get(2).setPosition(0.614);
        start = System.currentTimeMillis();

        depositFirst(capNum, endPoint);

        int numMinerals = 0;

        while (System.currentTimeMillis() - start <= 30000 - 3150 - 750 && opModeIsActive()){
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),20,6);
            drive.startIntake(false);
            driveToPoint(new Pose2d(18.5, endPoint.getY(),0), true,1, 0.8);
            driveToPoint(new Pose2d(36.5, endPoint.getY(),0), true,1, 0.6);
            driveToPoint(new Pose2d(40 + numMinerals * 4, endPoint.getY() - (numMinerals % 3) * 3,0), true,2, 0.45);
            intakeMineral(0.25,Math.toRadians((numMinerals % 3) * -15),2000);
            driveToPoint(new Pose2d(36.5, endPoint.getY(),0), true,2, 0.8);
            driveToPoint(endPoint,false, 0.5, 0.6);
            waitForDeposit();
            numMinerals ++;
        }

        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 0.8);
        while (opModeIsActive()){
            double points = 0;
            if (drive.currentPose.getX() >= 43.5 - 8){
                points += 10;
            }
            else if (drive.currentPose.getX() >= 43.5 + 8){
                points += 5;
            }
            points += 12 * (numMinerals + 1);
            points += 20;
            telemetry.addData("Points", points);
            telemetry.update();
            drive.update();
        }
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 18;
        double r = 5;
        switch (capNum) {
            case 0: r = 8; h = 8; break;
            case 1: r = 7; h = 12.13; break;
            case 2: r = 6; h = 20; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        //driveToPoint(endPoint,false, 0.25, 0.8);
        waitForDeposit();
    }
    public void waitForDeposit(){
        drive.deposit();
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
        }
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power){
        //while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > 0.5 || Math.abs(drive.currentPose.getY()-target.getY()) > 0.5)){ //&& (drive.intakeCase <= 2 || !intake)){
        double maxPowerForward = power;
        double maxPowerSide = power;
        double maxPowerTurn = power/2.0;
        double slowDownDist = 6;
        double slowTurnAngle = 8;
        double numLeft = 0;
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > error || Math.abs(drive.currentPose.getY()-target.getY()) > error || Math.abs(drive.currentPose.getHeading() - target.getHeading()) > Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake)){
            drive.update();
            Pose2d relError = new Pose2d(
                    Math.cos(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.sin(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()),
                    Math.cos(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()) - Math.sin(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()),
                    target.getHeading()-drive.currentPose.getHeading()
            );
            double forward = Math.min(Math.max(relError.getX()*maxPowerForward/slowDownDist,-maxPowerForward),maxPowerForward);
            double left = Math.min(Math.max(relError.getY()*maxPowerSide/slowDownDist,-maxPowerSide),maxPowerSide);
            double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/Math.toRadians(slowTurnAngle),-maxPowerTurn),maxPowerTurn);
            double p1 = forward-left-turn;
            double p2 = forward+left-turn;
            double p3 = forward-left+turn;
            double p4 = forward+left+turn;
            double max = Math.max(Math.max(Math.max(Math.max(Math.abs(p1),Math.abs(p2)),Math.abs(p3)),Math.abs(p4)),1);
            if (Math.abs(drive.currentPose.getY()-target.getY()) > 0.5 && Math.abs(drive.relCurrentVelocity.getY()) <= 1 && Math.signum(drive.relCurrentVelocity.getY()) == Math.signum(drive.currentPose.getY()) * -1){
                numLeft ++;
            }
            else {
                numLeft = 0;
            }
            if (numLeft >= 30) {
                numLeft = 0;
                drive.localizer.setPoseEstimate(new Pose2d(drive.currentPose.getX(),65.25 * Math.signum(drive.currentPose.getY()),0));
                telemetry.addData("trigger", "wall trigger");
                telemetry.update();
            }
            max *= 1.0/(1.0 - DriveConstants.kStatic);
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            p1 += DriveConstants.kStatic * Math.signum(p1);
            p2 += DriveConstants.kStatic * Math.signum(p2);
            p3 += DriveConstants.kStatic * Math.signum(p3);
            p4 += DriveConstants.kStatic * Math.signum(p4);
            drive.pinMotorPowers(p1, p2, p3, p4);
        }
        drive.setMotorPowers(0,0,0,0);
    }
    public void intakeMineral(double power, double targetHeading, long maxTime){
        if (drive.intakeCase > 2){
            return;
        }
        long startingTime = System.currentTimeMillis();
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double turn = drive.currentPose.getHeading() - targetHeading;
            drive.pinMotorPowers(power+turn,power+turn,power-turn,power-turn);
            drive.update();
        }
    }
    public void setUp(Pose2d startingPose){
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
}