package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

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
        Pose2d endPoint = new Pose2d(12,65.25,0);

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
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0),20,6);
            drive.startIntake(false);
            driveIn(endPoint,numMinerals);
            driveOut(endPoint);
            waitForDeposit();
            numMinerals ++;
        }

        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 0.8, 1000);

        while (opModeIsActive()){
            drive.update();
        }
    }
    public void driveIn(Pose2d endPoint, int numMinerals){
        int a = 4;
        int b = numMinerals/a;
        double angle = b * Math.toRadians(10);
        double x = 42 + (numMinerals % a) * 4;
        double y = 71.25 - Math.sin(angle) * 8.0 - Math.cos(angle) * 6.0 - b * 5;
        driveToPoint(new Pose2d(18.5, endPoint.getY(),0), true,1, 0.5,1000);
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), true,1, 0.5,1000);
        driveToPoint(new Pose2d(x,y,angle), true,2, 0.42,1000);
        sleep(2000);
        intakeMineral(0.25,3000);
    }
    public void driveOut(Pose2d endPoint){
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), false,2, 0.5,1000);
        driveToPoint(endPoint, false,2, 0.6,1000);
        driveToPoint(endPoint,false, 1, 0.4,1000);
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        switch (capNum) {
            case 0: r = 8; h = 8; break;
            case 1: r = 7; h = 12.125; break;
            case 2: r = 6; h = 20; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        waitForDeposit();
    }
    public void waitForDeposit(){
        drive.deposit();
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
        }
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime){
        double maxPowerTurn = power/2.0;
        double slowDownDist = 3;
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > error || Math.abs(drive.currentPose.getY()-target.getY()) > error || Math.abs(drive.currentPose.getHeading() - target.getHeading()) > Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            Pose2d relError = new Pose2d(
                    Math.cos(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.sin(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()),
                    Math.cos(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()) - Math.sin(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()),
                    target.getHeading()-drive.currentPose.getHeading()
            );
            double kStatic = DriveConstants.kStatic;
            double powerAdjust = power-kStatic;
            double turnAdjust = maxPowerTurn-kStatic;
            double forward = Math.min(Math.max(relError.getX()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getX()) * kStatic * Math.max(Math.signum(Math.abs(relError.getX()) - error),0);
            double left = Math.min(Math.max(relError.getY()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getY()) * kStatic * Math.max(Math.signum(Math.abs(relError.getY()) - error),0);
            double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/Math.toRadians(slowTurnAngle),-turnAdjust),turnAdjust) + Math.signum(relError.getHeading()) * kStatic * Math.max(Math.signum(Math.abs(relError.getHeading()) - Math.toRadians(5)),0);
            double p1 = forward-left-turn;
            double p2 = forward+left-turn;
            double p3 = forward-left+turn;
            double p4 = forward+left+turn;
            double max = Math.max(Math.max(Math.max(Math.max(Math.abs(p1),Math.abs(p2)),Math.abs(p3)),Math.abs(p4)),1);
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
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double turn = 0;//Math.signum(Math.sin(startingTime * Math.PI/500.0)) * 0.5;
            double multiplier = 1.0/(Math.abs(power) + Math.abs(turn));
            drive.pinMotorPowers((power+turn)*multiplier,(power+turn)*multiplier,(power-turn)*multiplier,(power-turn)*multiplier);
            drive.update();
        }
        drive.setMotorPowers(0,0,0,0);
    }
    public void setUp(Pose2d startingPose){
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
}