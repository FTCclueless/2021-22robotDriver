package org.firstinspires.ftc.teamcode.drive.opmode;

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
@Autonomous(group = "rr")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,true,true);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        int i = 0;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj.end(),true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        drive.update();
        drive.localizer.setPoseEstimate(new Pose2d(0,0,0));
        drive.update();

        while(opModeIsActive() && i < 10) {
            drive.followTrajectorySequence(traj);
            drive.updateWait(500);
            drive.followTrajectorySequence(traj1);
            drive.updateWait(500);
            i ++;
        }
    }
}
