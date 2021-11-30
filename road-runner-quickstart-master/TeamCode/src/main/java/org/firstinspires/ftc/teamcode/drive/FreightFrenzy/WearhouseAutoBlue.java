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
        drive.intakeTurretInterfaceHeading = 0;
        Pose2d startingPose = new Pose2d(12,65.25,0);

        ArrayList<TrajectorySequence> intake = new ArrayList<TrajectorySequence>();
        Pose2d endPoint = new Pose2d(12,64.75,0);
        int numIntakes = 11;
        int numMinerals = 0;
        for (int i = 0; i < numIntakes; i ++) {
            intake.add(drive.trajectorySequenceBuilder(endPoint)
                    .splineTo(new Vector2d(36.5, endPoint.getY()), 0)
                    .splineTo(new Vector2d(40 + (i / 3) * 4,endPoint.getY() - (i / 3) * 2),0)
                    .build());
        }
        //TODO: Implement ML here
        int capNum = 2;
        setUp(startingPose);
        drive.transferMineral = true;
        telemetry.addData("Start", "Ready");
        telemetry.update();
        waitForStart();
        //The program begins
        telemetry.addData("Start", "p1");
        telemetry.update();

        //grabCapstone(capNum,endPoint);
        depositFirst(capNum,startingPose,endPoint);
        telemetry.addData("Start", "p2");
        telemetry.update();

        while (numMinerals < numIntakes && System.currentTimeMillis() - start <= 30000 - 3150 - 750){
            drive.startIntake(false);
            drive.startDeposit(endPoint, new Pose2d(-12,24),20);
            drive.followTrajectorySequence(intake.get(numMinerals)); //going into the wearhouse
            intakeMineral(0.75,Math.toRadians((numMinerals % 3) * -15),2000); // getting a mineral
            drive.followTrajectorySequence(returnToScoring(endPoint)); //going to an area to drop off the mineral
            waitForDeposit(); // deposit the block when first possible
            numMinerals ++;
        }
        drive.followTrajectorySequence(intake.get(0));
        //waiting out the rest of the time in auto
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
        double d = Math.sqrt(Math.pow(endPoint.getX(),2) + Math.pow(endPoint.getY(),2));
        double x1 = r * (12-endPoint.getX())/d;
        double y1 = r * (-24-endPoint.getY())/d;
        drive.startDeposit(endPoint, new Pose2d(-12.0 + x1, (24.0 + y1) * Math.signum(endPoint.getY())), h);
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
        start = System.currentTimeMillis();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
}