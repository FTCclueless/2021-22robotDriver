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

        drive.setTurretTarget(Math.toRadians(drive.intakeTurretInterfaceHeading));
        drive.setDepositAngle(drive.depositTransferAngle);
        drive.setV4barOrientation(Math.toRadians(-5));
        waitForStart();

        drive.update();

        drive.servos.get(2).setPosition(0.614);
        start = System.currentTimeMillis();

        depositFirst(capNum, endPoint);

        int numMinerals = 0;

        while (System.currentTimeMillis() - start <= 30000 - 3150 - 750 && opModeIsActive()){
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0),17,5);
            drive.startIntake(false);
            driveIn(endPoint,numMinerals);
            driveOut(endPoint);
            waitForDeposit(endPoint);
            numMinerals ++;
        }

        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 0.8, 1000, 3);

        while (opModeIsActive()){
            drive.update();
        }
    }
    public void driveIn(Pose2d endPoint, int numMinerals){
        int a = 4;
        int b = numMinerals/a;
        double angle = b * Math.toRadians(-10);
        double x = 42 + (numMinerals % a) * 4;
        double y = 71.25 - Math.sin(angle) * 8.0 - Math.cos(angle) * 6.0 - b * 5;
        driveToPoint(new Pose2d(18.5, endPoint.getY(),0), true,1, 0.8,1000,1); //0.5
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), true,1, 0.6,1000,1); //0.5
        driveToPoint(new Pose2d(x,y,angle), true,2, 0.25,1000,5); //0.35
        intakeMineral(0.25,3000);
    }
    public void driveOut(Pose2d endPoint){
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), false,2, 0.8,1000,1); //0.5
        driveToPoint(endPoint, false,2, 0.8,1000,6); //0.6
        //driveToPoint(endPoint,false, 0.25, 0.2,1000); //1,0.4
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        switch (capNum) {
            case 0: r = 8.5; h = 6; break;
            case 1: r = 7; h = 12.125; break;
            case 2: r = 5; h = 17; break;
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
    public void waitForDeposit(Pose2d target){
        drive.deposit();
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            updateMotors(getRelError(target),DriveConstants.kStatic,0.25,0.25,2,Math.toRadians(8),0.25);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime, double slowDownDist){
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/2.0,kStatic * 1.5);
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > error || Math.abs(drive.currentPose.getY()-target.getY()) > error || Math.abs(drive.currentPose.getHeading() - target.getHeading()) > Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            updateMotors(getRelError(target), kStatic, power, maxPowerTurn, slowDownDist, Math.toRadians(slowTurnAngle), error);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public Pose2d getRelError(Pose2d target){
        return new Pose2d(
                Math.cos(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.sin(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()),
                Math.cos(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()) - Math.sin(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()),
                target.getHeading()-drive.currentPose.getHeading()
        );
    }
    public void updateMotors(Pose2d relError, double kStatic, double power, double maxPowerTurn, double slowDownDist, double slowTurnAngle, double error){
        double powerAdjust = power-kStatic;
        double turnAdjust = maxPowerTurn-kStatic;
        double forward = Math.min(Math.max(relError.getX()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getX()) * kStatic * Math.max(Math.signum(Math.abs(relError.getX()) - error),0);
        double left = Math.min(Math.max(relError.getY()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getY()) * kStatic * Math.max(Math.signum(Math.abs(relError.getY()) - error),0);
        double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/slowTurnAngle,-turnAdjust),turnAdjust) + Math.signum(relError.getHeading()) * kStatic * Math.max(Math.signum(Math.abs(relError.getHeading()) - Math.toRadians(5)),0);
        double [] p = new double[4];
        p[0] = forward-left-turn;
        p[1] = forward+left-turn;
        p[2] = forward-left+turn;
        p[3] = forward+left+turn;
        double max = 1;
        for (int i = 0; i < p.length; i ++){
            max = Math.max(Math.abs(p[i]),max);
        }
        max *= 1.0/(1.0 - kStatic);
        for (int i = 0; i < p.length; i ++){
            p[i] /= max;
            p[i] += kStatic * Math.signum(p[i]);
        }
        drive.pinMotorPowers(p[0], p[1], p[2], p[3]);
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