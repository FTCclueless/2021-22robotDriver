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
public class WearhouseAutoMaster extends LinearOpMode {
    public int capNum = 0;
    Long start;
    SampleMecanumDrive drive;
    double alliance = 0;
    Pose2d startingPose;
    Pose2d endPoint;
    int numIntakes = 11;
    int numMinerals = 0;
    ArrayList<TrajectorySequence> intake = new ArrayList<TrajectorySequence>();
    public WearhouseAutoMaster(boolean blue, boolean red, SampleMecanumDrive mecanumDrive){
        drive = mecanumDrive;
        if (blue ^ red){
            if (blue){
                alliance = 1;
            }
            if (red){
                alliance = -1;
            }
        }
        startingPose = new Pose2d(12,65.25 * alliance,0);
        endPoint = new Pose2d(12,64.75 * alliance,0);

        for (int i = 0; i < numIntakes; i ++) {
            intake.add(drive.trajectorySequenceBuilder(endPoint)
                    .splineTo(new Vector2d(36.5, endPoint.getY()), 0)
                    .splineTo(new Vector2d(40 + (i / 3) * 4,endPoint.getY() - (i / 3) * 2 * alliance),0)
                    .build());
        }

        setUp(startingPose);
        drive.transferMineral = true;
    }
    public void runOpMode() throws InterruptedException {
        waitForStart();
        depositFirst(capNum,startingPose,endPoint);
        while (numMinerals < numIntakes && System.currentTimeMillis() - start <= 30000 - 3150 - 750){
            drive.startIntake(false);
            drive.startDeposit(endPoint, new Pose2d(-12,24 * alliance),16,4);
            drive.followTrajectorySequence(intake.get(numMinerals)); //going into the wearhouse
            intakeMineral(0.75,Math.toRadians((numMinerals % 3) * -15) * alliance,2000); // getting a mineral
            drive.followTrajectorySequence(returnToScoring(endPoint)); //going to an area to drop off the mineral
            waitForDeposit(); // deposit the block when first possible
            numMinerals ++;
        }
        drive.followTrajectorySequence(intake.get(0));
        while (opModeIsActive()){
            drive.update();
        }
    }
    public void depositFirst(int capNum, Pose2d startingPose, Pose2d endPoint){
        double h = 0;
        double r = 0;
        switch (capNum) {
            case 0: r = 8; h = 8; break;
            case 1: r = 7; h = 12.13; break;
            case 2: r = 0; h = 20; break;
        }
        drive.startDeposit(endPoint,new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        TrajectorySequence c = drive.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(endPoint.getX(), endPoint.getY()), endPoint.getHeading())
                .build();
        drive.followTrajectorySequence(c);
        waitForDeposit();
    }
    public void waitForDeposit(){
        drive.deposit();
        while (drive.slidesCase <= 4 && opModeIsActive()) { //System.currentTimeMillis() - drive.depositTime <= 1500
            drive.update();
        }
    }
    public TrajectorySequence returnToScoring(Pose2d endPoint){
        return drive.trajectorySequenceBuilder(new Pose2d(drive.currentPose.getX(),drive.currentPose.getY(),0))
                .setReversed(true)
                .splineTo(new Vector2d(36.5,endPoint.getY()),Math.toRadians(180))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(endPoint.getX(),endPoint.getY()),Math.toRadians(180))
                .build();
    }
    public void intakeMineral(double power, double targetHeading, long maxTime){
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