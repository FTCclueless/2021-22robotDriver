package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Logger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class WearhouseAutoRed extends LinearOpMode {
    Long start;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.resetAssemblies();

        double side = -1;
        boolean intake = true;

        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.25* side, 0);

        //TODO: Implement ML here

        int capNum = 2;
        setUp(startingPose);
        drive.transferMineral = true;

        drive.currentIntake = side;

        drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);
        drive.setV4barDeposit(drive.depositTransferAngle,Math.toRadians(-5));
        waitForStart();

        Logger a = new Logger("Alliance",false);
        a.addData("red");

        drive.update();

        drive.servos.get(2).setPosition(0.614);
        start = System.currentTimeMillis();

        depositFirst(capNum, endPoint);

        int numMinerals = 0;

        while (System.currentTimeMillis() - start <= 30000 - 3270 && opModeIsActive()){
            drive.startIntake(intake);
            driveIn(endPoint,numMinerals,side);
            double i;
            if (numMinerals < 3){
                i = 0;
            }
            else if (numMinerals < 6){
                i = -2;
            }
            else {
                i = -1;
            }
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * side),13.5,3); //0.5
            Pose2d newEnd = new Pose2d(endPoint.getX() + i, endPoint.getY(), endPoint.getHeading());
            driveOut(newEnd);
            waitForDeposit(newEnd);
            numMinerals ++;
        }
        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 1, 1000, 3);
        drive.setMotorPowers( 0 , 0, 0, 0);
    }
    public void driveIn(Pose2d endPoint, int numMinerals, double side){
        int a = 3;
        int b = (numMinerals/(a - 1));
        double angle = (numMinerals % a) * Math.toRadians(-20) * side;
        double x = 42 + b * 4;
        double y = 71.25 * side - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        driveToPoint(new Pose2d(18.5, endPoint.getY(),0), new Pose2d(36.5, endPoint.getY(),0), true,1, 0.8,500,1);
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), new Pose2d(x,y,angle), true,1, 0.8,500,1);
        driveToPoint(new Pose2d(x,y,angle), new Pose2d(72,24 * side,angle), true,1, 0.5,500,3); // 0.5
        intakeMineral(0.35,1500); //0.35
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
    }
    public void driveOut(Pose2d endPoint){
        drive.deposit();
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), endPoint, false,1, 0.8,1000,1);
        driveToPoint(endPoint, false,2, 0.5,1000,3);
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        switch (capNum) {
            case 0: r = 8.5; h = 6; break;
            case 1: r = 7; h = 12.125; break;
            case 2: r = 3; h = 13.5; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        waitForDeposit();
    }
    public void waitForDeposit(){
        drive.deposit();
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();

            if (drive.intakeCase == 9){
                if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
                if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
            }
        }
    }
    public void waitForDeposit(Pose2d target){
        drive.deposit();
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            Pose2d error = drive.getRelError(target);
            double dist = Math.pow(error.getX()*error.getX() + error.getX()*error.getX(),0.5);
            if (dist > 1) {
                drive.updateMotors(error, 0.25, 0.25, 4, Math.toRadians(8), 1);
            }
            else {
                drive.setMotorPowers(0,0,0,0);
            }

            if (drive.intakeCase == 9){
                if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
                if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
            }
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist){
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/2.0,kStatic * 1.5);
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            if (x){
                relError = new Pose2d(0,relError.getY(),relError.getHeading());
            }
            if (y){
                relError = new Pose2d(relError.getX(),0,relError.getHeading());
            }
            drive.updateMotors(relError, power, maxPowerTurn, slowDownDist, Math.toRadians(slowTurnAngle), error);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
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
            drive.updateMotors(drive.getRelError(target), power, maxPowerTurn, slowDownDist, Math.toRadians(slowTurnAngle), error);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double turn = 0;//Math.signum(Math.sin(startingTime * Math.PI/500.0)) * 0.5;
            double multiplier = Math.min(1.0/(Math.abs(power) + Math.abs(turn)),1);
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