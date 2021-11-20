package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class Auto extends LinearOpMode {
    Long start;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPose = new Pose2d(12,66,0);

        //TODO: implement alliances
        TrajectorySequence[] intake = new TrajectorySequence[5];
        Pose2d endPoint = new Pose2d(12,64,0);
        int numIntakes = 7;
        int numMinerals = 0;
        for (int i = 0; i < numIntakes; i ++) {
            intake[i] = drive.trajectorySequenceBuilder(endPoint)
                    .splineTo(new Vector2d(36.5, 64), 0)
                    .splineTo(new Vector2d(40 + (i / 3) * 4,64),Math.toRadians((i % 3) * -15))
                    .build();
        }
        int capNum = 2;
        waitForStart();
        //The program begins
        setUp(startingPose);

        //grabCapstone(capNum,endPoint);
        depositFirst(capNum,startingPose,endPoint);

        while (numMinerals < numIntakes && System.currentTimeMillis() - start <= 30000 - 3150 - 750){
            drive.startIntake(false);
            drive.startDeposit(endPoint, new Pose2d(-12,24),20);
            drive.followTrajectorySequence(intake[numMinerals]); //going into the wearhouse
            intakeMineral(0.4,drive.currentPose.getHeading(),2000); // getting a mineral
            drive.followTrajectorySequence(returnToScoring(endPoint)); //going to an area to drop off the mineral
            waitForDeposit(); // deposit the block when first possible
            numMinerals ++;
        }
        drive.followTrajectorySequence(intake[0]);
        //waiting out the rest of the time in auto
        while (opModeIsActive()){
            drive.update();
        }
    }
    public void depositFirst(int capNum, Pose2d startingPose, Pose2d endPoint){
        switch (capNum) {
            case 0: drive.startDeposit(endPoint, new Pose2d(-0.7, 35.3), 6.25); break;
            case 1: drive.startDeposit(endPoint, new Pose2d(-2.1, 33.9), 12.13); break;
            case 2: drive.startDeposit(endPoint, new Pose2d(-12, 24), 20); break;
        }
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
        return drive.trajectorySequenceBuilder(drive.currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(36.5,endPoint.getY()),Math.toRadians(180))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(endPoint.getX(),endPoint.getY()),Math.toRadians(180))
                .turn(0)
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
        start = System.currentTimeMillis();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
    public void grabCapstone(double capNum, Pose2d endPoint, Pose2d startingPose){
        Vector2d pos = new Vector2d(26 + 8.83 * capNum,46.75);
        double hed = 0;
        if(capNum == 2){
            pos = new Vector2d(36.2, 44.7);
            hed = 0.35622;
        }
        TrajectorySequence grabCap = drive.trajectorySequenceBuilder(startingPose)
                .splineTo(pos,hed)
                .build();
        TrajectorySequence goToStart = drive.trajectorySequenceBuilder(new Pose2d(pos.getX(),pos.getY(),hed))
                .splineToConstantHeading(new Vector2d(endPoint.getX(),endPoint.getY()),endPoint.getHeading())
                .build();
        //TODO: Drop the arm
        drive.startDeposit(new Pose2d(pos.getX(),pos.getY(),hed), new Pose2d(12,24),16); //TODO: Make the height vary on the capNum
        drive.followTrajectorySequence(grabCap);
        waitForDeposit();
        //TODO: Lift up arm
        drive.followTrajectorySequence(goToStart);
    }
}