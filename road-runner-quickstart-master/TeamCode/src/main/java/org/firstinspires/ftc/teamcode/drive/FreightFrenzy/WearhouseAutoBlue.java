package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        drive.servos.get(2).setPosition(0.614);
        start = System.currentTimeMillis();

        depositFirst(capNum,startingPose,endPoint);

        int numIntakes = 11;
        int numMinerals = 0;

        while (numMinerals < numIntakes && System.currentTimeMillis() - start <= 30000 - 3150 - 750 && opModeIsActive()){
            drive.startIntake(false);
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),18,5);
            driveToPoint(new Pose2d(45 - numMinerals * 4, endPoint.getY() + (int)(numMinerals/3) * 3,0), true);
            intakeMineral(0.25,Math.toRadians((numMinerals % 3) * -15),2000);
            driveToPoint(endPoint,false);
            waitForDeposit();
            numMinerals ++;
        }

        driveToPoint(new Pose2d(45, endPoint.getY(),0), false);
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
    public void depositFirst(int capNum, Pose2d startingPose, Pose2d endPoint){
        double h = 18;
        double r = 5;
        switch (capNum) {
            case 0: r = 8; h = 8; break;
            case 1: r = 7; h = 12.13; break;
            case 2: r = 5; h = 18; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        driveToPoint(endPoint,false);
        waitForDeposit();
    }
    public void waitForDeposit(){
        drive.deposit();
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
        }
    }
    public void driveToPoint(Pose2d target, boolean intake){
        while (opModeIsActive() && (drive.currentPose.getX()-target.getX() > 0.5 && drive.currentPose.getY()-target.getY() > 0.5) && (drive.intakeCase <= 2 || !intake)){
            Pose2d relError = new Pose2d(
                    Math.cos(target.getHeading()) * (target.getX()-drive.currentPose.getX()) - Math.sin(target.getHeading()) * (target.getY()-drive.currentPose.getY()),
                    Math.sin(target.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.cos(target.getHeading()) * (target.getY()-drive.currentPose.getY()),
                    target.getHeading()-drive.currentPose.getHeading()
            );
            double forward = Math.max(Math.min(relError.getX()*5.0/0.8,-0.8),0.8);
            double left = Math.max(Math.min(relError.getY()*5.0/0.8,-0.8),0.8);
            double turn = Math.max(Math.min(relError.getHeading()*Math.toRadians(5)/0.4,-0.4),0.4);
            double p1 = forward-left+turn;
            double p2 = forward+left+turn;
            double p3 = forward-left-turn;
            double p4 = forward+left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);
        }
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