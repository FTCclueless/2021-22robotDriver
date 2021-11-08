package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "SetUp")
public class motorPIDTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetAssemblies();
        waitForStart();
        drive.v4bar.setPower(1.0);
        drive.v4bar.setTargetPosition(500);
        while (!isStopRequested()) {
        }
    }
}
