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
        Pose2d startingPose = new Pose2d(-36,66 * am,0);
        //TODO: Implement ML here
        int capNum = 2;
        waitForStart();
        depositFirst(capNum,startingPose,new Pose2d(-36, 64*am));
        //TODO: Finish auto
        /*
        1. go to duck
        2. Spin off duck
        3. grab duck
        4. deposit duck
        5. park in area
         */
        if (wearhouse){
            // go to the other end of the field to park
        }
        else {
            // park in area nearest to where currently are
        }

    }
    public void depositFirst(int capNum, Pose2d startingPose, Pose2d endPoint){
        switch (capNum) {
            case 0: drive.startDeposit(endPoint, new Pose2d(-23.3, 35.3 * Math.signum(endPoint.getY())), 6.25); break;
            case 1: drive.startDeposit(endPoint, new Pose2d(-21.9, 33.9 * Math.signum(endPoint.getY())), 12.13); break;
            case 2: drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())), 20); break;
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
}