package org.firstinspires.ftc.teamcode.drive.SetUp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "SetUp")
public class DuckSpinTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        ButtonToggle a = new ButtonToggle();
        ButtonToggle b = new ButtonToggle();
        ButtonToggle c = new ButtonToggle();
        ButtonToggle d = new ButtonToggle();
        double power = 0;
        boolean last = false;
        double startingSpeed = 0.27;
        long start = System.currentTimeMillis();
        long timer = 950;
        long totalTime = 0;
        boolean last1 = false;
        while (!isStopRequested()) {
            if (gamepad1.dpad_up){
                startingSpeed += 0.0001;
            }
            if (gamepad1.dpad_down){
                startingSpeed -= 0.0001;
            }
            if (gamepad1.dpad_left){
                timer += 1;
            }
            if (gamepad1.dpad_right){
                timer -= 1;
            }
            drive.servos.get(7).setPosition(0.467);
            a.update(gamepad1.left_bumper);
            b.update(gamepad1.right_bumper);
            c.update(gamepad1.a);
            d.update(gamepad1.b);
            boolean update = gamepad1.a || gamepad1.b || gamepad1.right_bumper;
            boolean running = b.getToggleState() || c.getToggleState() || d.getToggleState();
            if (update && !last){
                power = startingSpeed;
                start = System.currentTimeMillis();
            }
            if (!running && !a.getToggleState() && last1){
                totalTime = System.currentTimeMillis() - start;
            }
            last = update;
            last1 = running || a.getToggleState();
            if (a.getToggleState()){
               power = startingSpeed;
            }
            else if (running){
                if (System.currentTimeMillis() - start < timer) {
                    if (b.getToggleState()) {
                        power += 0.001;
                    } else if (b.getToggleState()) {
                        power += 0.005;
                    } else if (c.getToggleState()) {
                        power += 0.0001;
                    } else if (d.getToggleState()) {
                        power += 0.000075;
                    }
                    power = Math.min(power,0.4);
                }
                else {
                    power = 1.0;
                }
            }
            else {
                power = 0;
            }
            drive.duckSpin.setPower(power);
            drive.duckSpin2.setPower(power * -1.0);
            sleep(6);
            telemetry.addData("speed", power);
            telemetry.addData("startingSpeed", startingSpeed);
            telemetry.addData("time", timer);
            telemetry.addData("time to deposit", totalTime);
            telemetry.update();
        }
    }
}
