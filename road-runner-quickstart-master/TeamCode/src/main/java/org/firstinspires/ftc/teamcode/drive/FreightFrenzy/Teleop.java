package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.widget.ToggleButton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Comp Drive")
public class Teleop extends LinearOpMode {

    ButtonToggle auto = new ButtonToggle();

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

        int lastLocVal = 0;
        int locVal = 0;

        boolean deposit = false;
        int flag = 0;

        int turretOffset = 0;
        int slidesOffset = 0;

        double lastIntakeCase = 0;

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
                flag = 0;
            }
            resetLoc.update(gamepad2.left_bumper);
            if (resetLoc.getToggleState()){
                int a = 0;
                if (gamepad2.dpad_right){
                    a = 1;
                }
                if (gamepad2.dpad_left){
                    a = 2;
                }
                if (gamepad2.dpad_up){
                    a = 3;
                }
                if (gamepad2.dpad_down){
                    a = 4;
                }
                if (a != lastLocVal && a != 0){
                    if ((a == 1 && locVal == 1) || (a == 1 && locVal == 4) || (a == 4 && locVal == 1)){ //right
                        drive.localizer.setPoseEstimate(new Pose2d(24,-65,Math.toRadians(0)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 2 && locVal == 2) || (a == 2 && locVal == 4) || (a == 4 && locVal == 2)){ //left
                        drive.localizer.setPoseEstimate(new Pose2d(24,65,Math.toRadians(0)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 1 && locVal == 3) || (a == 3 && locVal == 1)){ //top right
                        drive.localizer.setPoseEstimate(new Pose2d(65,-24,Math.toRadians(-90)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 2 && locVal == 3) || (a == 3 && locVal == 2)){ //top left
                        drive.localizer.setPoseEstimate(new Pose2d(65,24,Math.toRadians(90)));
                        resetLoc.toggleState = false;
                    }
                    locVal = a;
                }
                lastLocVal = a;
            }
            else{
                locVal = 0;
                lastLocVal = 0;
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
                    armInPos = Math.min(Math.max(armInPos,1.0),0);
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
                        armOutGrabPos = Math.min(Math.max(armOutGrabPos,1.0),0);
                        drive.servos.get(5).setPosition(armOutGrabPos);
                    }
                    else {
                        if (gamepad2.dpad_left){
                            armOutPos += 0.001;
                        }
                        else if (gamepad2.dpad_right){
                            armOutPos -= 0.001;
                        }
                        armOutPos = Math.min(Math.max(armOutPos,1.0),0);
                        drive.servos.get(5).setPosition(armOutPos);
                    }
                }
            }
            auto.update(gamepad1.a);
            //TODO: allow the drivers to auto cancel at any time
            if (auto.toggleState){
                if (deposit && flag == 0) {
                    flag ++;
                    deposit = false;
                    drive.startIntake(intake);
                    driveIn(hub,endPoint,drive);
                }
                else if (!deposit && flag == 0) {
                    flag ++;
                    deposit = true;
                    drive.startDeposit(endPoint, hubLocation, height);
                    if (hub == 1) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(drive.currentPose.getX(), drive.currentPose.getY(), 0))
                                .setReversed(true)
                                .splineTo(new Vector2d(36.5, endPoint.getY()), Math.toRadians(180))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(endPoint.getX(), endPoint.getY()), Math.toRadians(180))
                                .build());
                    }
                    if (hub == 0) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(drive.currentPose.getX(), drive.currentPose.getY(), 0))
                                .setReversed(true)
                                .splineTo(new Vector2d(endPoint.getX(), 36.5), Math.toRadians(180 * Math.signum(endPoint.getHeading())) - endPoint.getHeading())
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(endPoint.getX(), endPoint.getY()), Math.toRadians(180 * Math.signum(endPoint.getHeading())) - endPoint.getHeading())
                                .build());
                    }
                }
            }
            if (drive.intakeCase == 3 && lastIntakeCase == 2){
                flag = 0;
            }
            lastIntakeCase = drive.intakeCase;


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

            if(Math.abs(gamepad2.left_stick_x) > 0.25) {
                drive.setTurretTarget(drive.targetTurretHeading += gamepad2.right_stick_x * -0.1);
                turretOffset = drive.targetTurretHeading - drive.turretHeading;
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) {
                drive.setSlidesLength(drive.targetSlideExtensionLength += gamepad2.right_stick_y * -0.1);
                slidesOffset = drive.targetSlideExtensionLength - drive.slideExtensionLength;
            }
        }
    }
    public void driveIn(int hub, Pose2d endPoint, SampleMecanumDrive drive){
        Vector2d p1 = new Vector2d();
        Vector2d p2 = new Vector2d();
        if (hub == 1) {
            p1 = new Vector2d(16.5, endPoint.getY());
            p2 = new Vector2d(36.5, endPoint.getY());
        }
        if (hub == 2){
            p1 = new Vector2d(endPoint.getX(), 16.5);
            p2 = new Vector2d(endPoint.getX(), 36.5);
        }
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(endPoint)
                .splineTo(p1, endPoint.getHeading())
                .splineTo(p2, endPoint.getHeading())
                .addTemporalMarker(0.5, 0, () -> {
                    if (gamepad1.a) {
                        auto.toggleState = false;
                        drive.trajectorySequenceRunner.remainingMarkers.clear();
                    }
                })
                .addTemporalMarker(0.6, 0, () -> {
                    if (gamepad1.a) {
                        auto.toggleState = false;
                        drive.trajectorySequenceRunner.remainingMarkers.clear();
                    }
                })
                .addTemporalMarker(0.7, 0, () -> {
                    if (gamepad1.a) {
                        auto.toggleState = false;
                        drive.trajectorySequenceRunner.remainingMarkers.clear();
                    }
                })
                .addTemporalMarker(0.8, 0, () -> {
                    if (gamepad1.a) {
                        auto.toggleState = false;
                        drive.trajectorySequenceRunner.remainingMarkers.clear();
                    }
                })
                .addTemporalMarker(0.9, 0, () -> {
                    drive.trajectorySequenceRunner.remainingMarkers.clear();
                })
                .build());
    }
}
