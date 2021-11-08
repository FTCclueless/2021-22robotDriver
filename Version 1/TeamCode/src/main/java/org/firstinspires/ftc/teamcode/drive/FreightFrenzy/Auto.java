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
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d(12,66,0);

        TrajectorySequence traj[] = new TrajectorySequence[5];
        for (int i = 0; i < 5; i ++) {
            traj[i] = drive.trajectorySequenceBuilder(new Pose2d(12,64,0))
                    .addTemporalMarker(() -> {
                        drive.intakeCase = 0;
                        drive.lastIntakeCase = 0;
                        drive.startIntake(false);
                        //drive.startDeposit(new Pose2d(),30);
                    })
                    .splineTo(new Vector2d(36, 64), 0)
                    .splineToConstantHeading(new Vector2d(43 + (int)(i/3) * 4, 63 - (3 * i) % 9),0)
                    .addTemporalMarker(() -> {
                        // this would make it keep driving until it intakes something

                    })
                    .setReversed(true)
                    .splineTo(new Vector2d(12, 64), Math.toRadians(180))
                    .build();
        }
        waitForStart();
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
        /*
        drive.followTrajectorySequence( drive.trajectorySequenceBuilder(startingPose)
                .strafeTo(new Vector2d(12, 64))
                .build()
        );
         */
        for (int i = 0; i < 5; i ++){
            drive.followTrajectorySequence(traj[i]);
        }
        while (opModeIsActive()){
            drive.update();
        }
    }
}
