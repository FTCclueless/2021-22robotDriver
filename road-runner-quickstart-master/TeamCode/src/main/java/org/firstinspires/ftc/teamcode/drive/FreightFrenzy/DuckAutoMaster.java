package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
public class DuckAutoMaster extends LinearOpMode {
    boolean alliance;
    boolean wearhouse;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public DuckAutoMaster(boolean red, boolean wearhouse){
        this.alliance = red;
        this.wearhouse = wearhouse;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        double am = 1.0;
        if (alliance){
            am = -1.0;
        }
        Pose2d startingPose = new Pose2d(-36,66 * am,Math.toRadians(180));
        //TODO: Implement ML here
        int capNum = 2;
        waitForStart();
        depositFirst(capNum,startingPose,new Pose2d(-36, 64 * Math.signum(startingPose.getY()), Math.toRadians(180)));
        grabCapstone(capNum,new Pose2d(-36, 64 * Math.signum(startingPose.getY())));
        spinDuck();
        grabDuck(startingPose);
        if (wearhouse){
            // go to the other end of the field to park
        }
        else {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.currentPose)
                    .splineToConstantHeading(new Vector2d(-60,36 * Math.signum(startingPose.getY())),Math.toRadians(180))
                    .build());
        }

    }
    public void grabDuck(Pose2d startPos){
        //activateML possibly
        //retract the spin spin servo
        drive.startIntake(!alliance);
        Pose2d depositPoint = new Pose2d(-48,48 * Math.signum(startPos.getY()),Math.toRadians(180));
        drive.startDeposit(depositPoint, new Pose2d(-12.0, 24.0 * Math.signum(startPos.getY())), 16,4);
        //go forward till intake
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.currentPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(depositPoint.getX(),depositPoint.getY()),depositPoint.getHeading())
                .build());
        waitForDeposit();
    }
    public void spinDuck(){
        //calculate the duck spin spin servo pos
        //get it to there
        //spin off duck
    }
    public void grabCapstone(double capNum, Pose2d startingPose){
        double markerPos = capNum;
        if (alliance){
            markerPos = 2 - markerPos;
        }
        Vector2d pos = new Vector2d(-28.5 - 8.38 * markerPos,46.75 * Math.signum(startingPose.getY()));
        //TODO: Drop the arm

        //going to the marker pos spot that the servo goes over the correct area
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startingPose).splineToConstantHeading(pos,Math.toRadians(180)).build());
        //TODO: Lift up arm

        //going to the duck carousel
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.currentPose)
                .splineToConstantHeading(new Vector2d(-60, -60 * Math.signum(startingPose.getY())),Math.toRadians(180 - 45 * Math.signum(startingPose.getY())))
                .build());
    }
    public void depositFirst(int capNum, Pose2d startingPose, Pose2d endPoint){
        double h = 16;
        double r = 4;
        switch (capNum) {
            case 0: r = 8; h = 6.25; break;
            case 1: r = 7; h = 12.13; break;
            case 2: r = 4; h = 16; break;
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
}