package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "SetUp")
public class EncoderTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            double forward = gamepad1.left_stick_y * -0.4;
            double left = gamepad1.left_stick_x * 0.6;
            double turn = gamepad1.right_stick_x * 0.35;
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.setMotorPowers(p1, p2, p3, p4);
            drive.getEncoders();
            drive.localizer.updateEncoders(drive.encoders);
            telemetry.addData("right Encoder",drive.localizer.encoders[0].getCurrentDist());
            telemetry.addData("left Encoder",drive.localizer.encoders[1].getCurrentDist());
            telemetry.addData("back Encoder",drive.localizer.encoders[2].getCurrentDist());
            if (drive.localizer.encoders.length == 4) {
                telemetry.addData("front Encoder", drive.localizer.encoders[3].getCurrentDist());
            }
            telemetry.update();
            drive.slides.setPower(1.0);
            drive.slides2.setPower(1.0);
        }
    }
}
