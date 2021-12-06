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

        ArrayList<TrajectorySequence> intake = new ArrayList<TrajectorySequence>();
        Pose2d endPoint = new Pose2d(12,64.75,0);
        int numIntakes = 11;
        int numMinerals = 0;
        for (int i = 0; i < numIntakes; i ++) {
            int n = i/3;
            intake.add(drive.trajectorySequenceBuilder(endPoint)
                    .splineTo(new Vector2d(36.5, endPoint.getY()), 0)
                    .splineTo(new Vector2d(45 + i * 4, endPoint.getY() + n * 3), 0)
                    .addTemporalMarker(0.8,0, () -> {
                        drive.trajectorySequenceRunner.remainingMarkers.clear();
                    })
                    .build());
        }
        //TODO: Implement ML here
        int capNum = 2;
        setUp(startingPose);
        drive.transferMineral = true;
        waitForStart();
        drive.servos.get(2).setPosition(0.614);
        start = System.currentTimeMillis();
        depositFirst(capNum,startingPose,endPoint);
        while (numMinerals < numIntakes && System.currentTimeMillis() - start <= 30000 - 3150 - 750 && opModeIsActive()){
            drive.stopTrajectoryIntake = true;
            drive.startIntake(false);
            drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),18,5);
            drive.followTrajectorySequence(intake.get(numMinerals)); //going into the wearhouse
            intakeMineral(0.25,Math.toRadians((numMinerals % 3) * -15),2000); // getting a mineral
            drive.followTrajectorySequence(returnToScoring(endPoint)); //going to an area to drop off the mineral
            waitForDeposit(); // deposit the block when first possible
            numMinerals ++;
        }
        drive.followTrajectorySequence(intake.get(0));
        //waiting out the rest of the time in auto
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