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
@Autonomous(group = "drive")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-72+9, 72-9, Math.toRadians(-90));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean isZoneA = true;
        boolean isZoneB = false;
        boolean isZoneC = false;

        Vector2d deposit = new Vector2d(0,0);

        Vector2d ZoneA = new Vector2d(40, 53);
        Vector2d ZoneB = new Vector2d(16, 53);
        Vector2d ZoneC = new Vector2d(-8, 53);

        if(isZoneA) {
            deposit = ZoneA;
        }

        if(isZoneB) {
            deposit = ZoneB;
        }

        if(isZoneC) {
            deposit = ZoneC;
        }

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-48, 12), Math.toRadians(-90))
                .waitSeconds(3)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajSeq1.end(),true)
                .splineTo(new Vector2d(-24, 53), Math.toRadians(0))
                .splineTo(deposit,Math.toRadians(0))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(2)
                .splineTo(new Vector2d(-24, 53), Math.toRadians(180))
                .splineTo(new Vector2d(-36, 12), Math.toRadians(-135))
                .waitSeconds(3)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),true)
                .splineTo(new Vector2d(12,36), Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq1);
            drive.followTrajectory(traj1);
            drive.followTrajectorySequence(trajSeq2);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

        sleep(2000);

    }

}
