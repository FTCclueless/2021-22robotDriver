package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.widget.ToggleButton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Comp Drive")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(12,66,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        int hub = 0;

        Pose2d endPoint = new Pose2d();
        Pose2d hubLocation = new Pose2d();
        double height = 0;
        double r = 0;

        boolean intake = true;

        int level = 3;

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        boolean lastToggleHub = false;

        ButtonToggle endgame = new ButtonToggle();
        ButtonToggle spin = new ButtonToggle();
        ButtonToggle odo = new ButtonToggle();
        ButtonToggle resetLoc = new ButtonToggle();

        long start = System.currentTimeMillis();

        double armInPos = 0;
        double armOutPos = 0.5;
        double armOutGrabPos = 1;
        boolean first = true;
        boolean lastIn = false;
        boolean lastOut = false;
        boolean armIn = true;

        while (!isStopRequested()) {
            drive.update();
            switch(hub) {
                case 0: endPoint = new Pose2d(64, 12, Math.toRadians(90));
                    hubLocation = new Pose2d(48, 0);
                    intake = true;
                    height = 6;
                        break;
                case 1: case 2:
                    if (hub == 1) {
                        endPoint = new Pose2d(12, 64, Math.toRadians(0));
                    }
                    else {
                        endPoint = new Pose2d(-56, 60, Math.toRadians(180));
                    }
                    boolean dpadUp = gamepad1.dpad_up;
                    boolean dpadDown = gamepad1.dpad_down;
                    if (!lastDpadUp && dpadUp){
                        level = Math.min(level + 1,3);
                    }
                    else if (!lastDpadDown &&dpadDown){
                        level = Math.max(level - 1,1);
                    }
                    lastDpadUp = dpadUp;
                    lastDpadDown = dpadDown;
                    switch (level){
                        case 1: r = 8; height = 8; break;
                        case 2: r = 7; height = 12.13; break;
                        case 3: r = 0; height = 20; break;
                    }
                    double d = Math.sqrt(Math.pow(endPoint.getX(),2) + Math.pow(endPoint.getY(),2));
                    double x1 = r * (12-endPoint.getX())/d;
                    double y1 = r * (-24-endPoint.getY())/d;
                    hubLocation = new Pose2d(-12.0 + x1, (24.0 + y1));
                    intake = false;
                        break;
            }
            if(gamepad1.right_bumper) {
                drive.startDeposit(endPoint, hubLocation, height);
            }
            if(gamepad1.right_trigger >= 0.5) {
                drive.startIntake(intake);
            }
            endgame.update(gamepad1.left_bumper);
            if (System.currentTimeMillis() - start >= 90000){
                endgame.toggleState = true;
            }
            if (endgame.getToggleState()){
                spin.update(gamepad2.y);
                if (spin.getToggleState()){
                    drive.servos.get(7).setPosition(0.5);
                    if (gamepad2.b){
                        drive.duckSpin.setPower(-1);
                    }
                }
                else{
                    drive.servos.get(7).setPosition(1.0);
                    drive.duckSpin.setPower(0);
                }
            }
            boolean toggleHub = gamepad1.y;
            if (toggleHub && !lastToggleHub){
                if (!endgame.getToggleState()) {
                    hub = (hub + 1) % 2;
                }
                else {
                    hub = (hub + 1) % 3;
                }
            }
            lastToggleHub = toggleHub;
            odo.update(gamepad2.b);
            if (odo.getToggleState()){
                drive.servos.get(3).setPosition(0.668);
                drive.isKnownY = false;
                drive.isKnownX = false;
            }
            else {
                drive.servos.get(3).setPosition(0.48);
            }
            if (gamepad2.right_trigger >= 0.5){
                drive.deposit();
            }
            resetLoc.update(gamepad2.left_bumper);
            if (resetLoc.getToggleState()){
                //Reset Loc
            }
            else{
                boolean in = gamepad2.dpad_down;
                boolean out = gamepad2.dpad_up;
                if (in && !lastIn){
                    if (!armIn){
                        first = false;
                    }
                    armIn = true;
                }
                else if (out && !lastOut){
                    armIn = false;
                }
                lastIn = in;
                lastOut = out;
                if (armIn){
                    if (gamepad2.dpad_left){
                        armInPos += 0.001;
                    }
                    else if (gamepad2.dpad_right){
                        armInPos -= 0.001;
                    }
                    drive.servos.get(5).setPosition(armInPos);
                }
                else {
                    if (first){
                        if (gamepad2.dpad_left){
                            armOutGrabPos += 0.001;
                        }
                        else if (gamepad2.dpad_right){
                            armOutGrabPos -= 0.001;
                        }
                        drive.servos.get(5).setPosition(armOutGrabPos);
                    }
                    else {
                        if (gamepad2.dpad_left){
                            armOutPos += 0.001;
                        }
                        else if (gamepad2.dpad_right){
                            armOutPos -= 0.001;
                        }
                        drive.servos.get(5).setPosition(armOutPos);
                    }
                }
            }


            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * 0.35;
            if (!gamepad1.left_stick_button){
                forward *= 0.4;
                left *= 0.5;
            }
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);
        }
    }
}
