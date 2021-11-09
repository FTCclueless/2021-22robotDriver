package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        TrajectorySequence traj[] = new TrajectorySequence[5];
        Pose2d endPoint = new Pose2d(12,64,0);
        double y = 24 - endPoint.getY();
        double x = y / Math.tan(Math.toRadians(57.5));
        Pose2d depositPoint = new Pose2d(x,y);
        for (int i = 0; i < 5; i ++) {
            traj[i] = drive.trajectorySequenceBuilder(endPoint)
                    .addTemporalMarker(() -> {
                        drive.startIntake(false);
                        //drive.startDeposit(depositPoint,30);
                    })
                    .splineTo(new Vector2d(36.5, 64), 0)
                    .splineToConstantHeading(new Vector2d(38, 63 - (3 * i) % 9),0)
                    .splineToConstantHeading(new Vector2d(43 + (int)(i/3) * 4, 63 - (3 * i) % 9),0)
                    .addTemporalMarker(() -> {
                        while(drive.intakeCase <= 2){
                            double power = 0.3;
                            double turn = drive.currentPose.getHeading();
                            drive.pinMotorPowers(power+turn,power+turn,power-turn,power-turn);
                            drive.update();
                        }
                    })
                    .setReversed(true)
                    .splineTo(new Vector2d(endPoint.getX(),endPoint.getY()),Math.toRadians(180))
                    .turn(0)
                    .addTemporalMarker(() -> {
                        drive.intakeCase = 0;
                        drive.lastIntakeCase = 0;
                        while(System.currentTimeMillis() - drive.depositTime <= 1500){
                            drive.update();
                        }
                        /*
                        drive.deposit();
                        drive.setMotorPowers(0,0,0,0);
                        while(drive.slidesCase <= 4){
                            drive.update();
                        }
                        */
                    })
                    .build();
        }
        waitForStart();
        setUp();
        long start = System.currentTimeMillis();
        int numMinerals = 0;
        while (numMinerals < 5 && System.currentTimeMillis() - start <= 22000){
            drive.followTrajectorySequence(traj[numMinerals]);
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
