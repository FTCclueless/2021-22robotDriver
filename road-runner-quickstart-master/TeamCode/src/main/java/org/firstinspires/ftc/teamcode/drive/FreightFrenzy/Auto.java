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
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startingPose = new Pose2d(12,66,0);
    @Override
    public void runOpMode() throws InterruptedException {
        //double y = 24 - endPoint.getY();
        //double x = y / Math.tan(Math.toRadians(57.5));
        //Pose2d depositPoint = new Pose2d(x,y);
        TrajectorySequence[] intake = new TrajectorySequence[5];
        Pose2d endPoint = new Pose2d(12,64,0);
        boolean robotFinished = false;
        for (int i = 0; i < 5; i ++) {
            intake[i] = drive.trajectorySequenceBuilder(endPoint)
                    .addTemporalMarker(() -> {
                        drive.startIntake(false);
                        if (robotFinished) {
                            drive.startDeposit(endPoint, new Pose2d(12,24),16);
                        }
                    })
                    .splineTo(new Vector2d(36.5, 64), 0)
                    .splineTo(new Vector2d(40 + (i / 3) * 4,64),Math.toRadians((i % 3) * -15))
                    .build();
                    //.splineToConstantHeading(new Vector2d(38, 63 - 3 * (i % 3)),0)
                    //.splineToConstantHeading(new Vector2d(40 + (i / 3) * 4, 63 - (i % 3) * 3),0)
        }
        waitForStart();
        setUp();


        long start = System.currentTimeMillis();
        int numMinerals = 0;


        while (numMinerals < 5 && System.currentTimeMillis() - start <= 22000){


            drive.followTrajectorySequence(intake[numMinerals]);


            double power = 0.3;
            double startHeading = drive.currentPose.getHeading();
            while(drive.intakeCase <= 2){
                double turn = drive.currentPose.getHeading() - startHeading;
                drive.pinMotorPowers(power+turn,power+turn,power-turn,power-turn);
                drive.update();
            }


            TrajectorySequence deposit = drive.trajectorySequenceBuilder(drive.currentPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(36.5,endPoint.getY()),Math.toRadians(180))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(endPoint.getX(),endPoint.getY()),Math.toRadians(180))
                    .turn(0)
                    .build();
            drive.followTrajectorySequence(deposit);


            drive.intakeCase = 0;
            drive.lastIntakeCase = 0;
            drive.deposit();


            while (System.currentTimeMillis() - drive.depositTime <= 1500) { //drive.slidesCase <= 4
                drive.update();
            }


            numMinerals ++;
        }
        while (opModeIsActive()){
            drive.update();
        }
    }
    public void setUp(){
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
}
