package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.widget.ToggleButton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.servos.get(5).setPosition(0.237);

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(12,65.25,0);
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

        double armInPos = 0.237;
        double armOutPos = 0.403;
        double armOutGrabPos = 0.803;
        boolean first = true;
        boolean lastIn = false;
        boolean lastOut = false;
        boolean armIn = true;

        int lastLocVal = 0;
        int locVal = 0;

        boolean done = false;
        int flag = 0;

        int turretOffset = 0;
        int slidesOffset = 0;

        double lastIntakeCase = 0;
        double lastSlidesCase = 0;

        double radius = 0;

        while (!isStopRequested()) {
            drive.update();
            switch(hub) {
                case 0: endPoint = new Pose2d(65.25, 16, Math.toRadians(90));
                    hubLocation = new Pose2d(48, 0);
                    intake = true;
                    height = 6;
                    radius = 2;
                        break;
                case 1: case 2:
                    if (hub == 1) {
                        endPoint = new Pose2d(12, 65.25, Math.toRadians(0));
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
                        case 1: radius = 8.5; height = 6; break;
                        case 2: radius = 7; height = 12.13; break;
                        case 3: radius = 5; height = 17; break;
                    }
                    hubLocation = new Pose2d(-12.0, 24.0);
                    intake = false;
                        break;
            }
            if(gamepad1.right_bumper) {
                drive.startDeposit(endPoint, hubLocation, height, radius);
            }
            if(gamepad1.right_trigger >= 0.5) {
                //drive.startDeposit(endPoint, hubLocation, height, radius);
                drive.startIntake(intake);
            }
            endgame.update(gamepad1.left_bumper);
            if (System.currentTimeMillis() - start >= 90000){
                endgame.toggleState = true;
            }

            // Endgame controls do not line up with diagram. gamepad1.left_bumper starts endgame and extends duckSpinSpin servo. gamepad2.b moves odo servo.  gamepad2.y spins flywheel
            // Fine adjustments for slides and turret on gamepad2 not working
            if (endgame.getToggleState()){
                spin.update(gamepad2.y);
                if (spin.getToggleState()){
                    //drive.servos.get(7).setPosition(0.5);
                    if (gamepad2.b){
                        drive.duckSpin.setPower(-1);
                        drive.duckSpin2.setPower(-1);
                    }
                }
                else{
                    //drive.servos.get(7).setPosition(1.0);
                    drive.duckSpin.setPower(0);
                    drive.duckSpin2.setPower(0);
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
                        drive.localizer.setPoseEstimate(new Pose2d(24,-65.25,Math.toRadians(0)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 2 && locVal == 2) || (a == 2 && locVal == 4) || (a == 4 && locVal == 2)){ //left
                        drive.localizer.setPoseEstimate(new Pose2d(24,65.25,Math.toRadians(0)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 1 && locVal == 3) || (a == 3 && locVal == 1)){ //top right
                        drive.localizer.setPoseEstimate(new Pose2d(65.25,-24,Math.toRadians(-90)));
                        resetLoc.toggleState = false;
                    }
                    if ((a == 2 && locVal == 3) || (a == 3 && locVal == 2)){ //top left
                        drive.localizer.setPoseEstimate(new Pose2d(65.25,24,Math.toRadians(90)));
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
                    armInPos = Math.max(Math.min(armInPos,1.0),0);
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
                        armOutGrabPos = Math.max(Math.min(armOutGrabPos,1.0),0);
                        drive.servos.get(5).setPosition(armOutGrabPos);
                    }
                    else {
                        if (gamepad2.dpad_left){
                            armOutPos += 0.001;
                        }
                        else if (gamepad2.dpad_right){
                            armOutPos -= 0.001;
                        }
                        armOutPos = Math.max(Math.min(armOutPos,1.0),0);
                        drive.servos.get(5).setPosition(armOutPos);
                    }
                }
            }
            auto.update(gamepad1.a);
            //TODO: allow the drivers to auto cancel at any time
            if (auto.toggleState && !done){
                done = true;
                if (flag == 0) {
                    if (drive.intakeCase == 0) {
                        drive.startIntake(intake);
                    }
                    drive.startDeposit(endPoint,hubLocation,height,radius);
                    boolean inside = Math.abs(drive.currentPose.getY()) > 43.5 && drive.currentPose.getX() > 43.5;
                    if (! inside) {
                        if (hub == 1) {
                            driveToPoint(new Pose2d(16.5, endPoint.getY(), endPoint.getHeading()));
                            driveToPoint(new Pose2d(36.5, endPoint.getY(), endPoint.getHeading()));
                        }
                        if (hub == 0) {
                            driveToPoint(new Pose2d(endPoint.getX(), 16.5, endPoint.getHeading()));
                            driveToPoint(new Pose2d(endPoint.getX(), 36.5, endPoint.getHeading()));
                        }
                    }
                }
                else {
                    if (drive.slidesCase == 0) {
                        drive.startDeposit(endPoint, hubLocation, height, radius);
                    }
                    if (hub == 1) {
                        driveToPoint(new Pose2d(36.5,endPoint.getY(),endPoint.getHeading()));
                        driveToPoint(endPoint);
                    }
                    if (hub == 0) {
                        driveToPoint(new Pose2d(endPoint.getX(),36.5,endPoint.getHeading()));
                        driveToPoint(endPoint);
                    }
                }
            }
            if (drive.intakeCase == 3 && lastIntakeCase == 2){ // Just took in a block
                flag = 1;
                done = false;
            }
            if (drive.slidesCase == 5 && lastSlidesCase == 4){ // Just finished the deposit
                flag = 0;
                done = false;
            }
            lastIntakeCase = drive.intakeCase;
            lastSlidesCase = drive.slidesCase;

            double lY1 = gamepad1.left_stick_y * -1;
            double lX1 = gamepad1.left_stick_x;
            double forward = lY1;
            double left = lX1;
            if (false){
                //Field-centric-controls
                forward = lY1 * Math.cos(drive.currentPose.getHeading()) + lX1 * Math.sin(drive.currentPose.getHeading());
                left = lX1 * Math.cos(drive.currentPose.getHeading()) - lY1 * Math.sin(drive.currentPose.getHeading());
            }
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
                drive.turretOffset -= Math.toRadians(gamepad2.left_stick_y) * 0.1;
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) {
                drive.slidesOffset -= gamepad2.right_stick_y * 0.01;
            }
        }
    }
    public void driveToPoint(Pose2d target){
        double maxPowerForward = 0.8;
        double maxPowerSide = 0.8;
        double maxPowerTurn = 0.4;
        double slowDownDist = 4;
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = 2;
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > 2 || Math.abs(drive.currentPose.getY()-target.getY()) > 2) && auto.getToggleState()){
            auto.update(gamepad1.a);
            drive.update();
            Pose2d relError = new Pose2d(
                    Math.cos(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.sin(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()),
                    Math.cos(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()) - Math.sin(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()),
                    target.getHeading()-drive.currentPose.getHeading()
            );
            double forward = Math.min(Math.max(relError.getX()*maxPowerForward/slowDownDist,-maxPowerForward),maxPowerForward);
            double left = Math.min(Math.max(relError.getY()*maxPowerSide/slowDownDist,-maxPowerSide),maxPowerSide);
            double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/Math.toRadians(slowTurnAngle),-maxPowerTurn),maxPowerTurn);
            double p1 = forward-left-turn;
            double p2 = forward+left-turn;
            double p3 = forward-left+turn;
            double p4 = forward+left+turn;
            double max = Math.max(Math.max(Math.max(Math.max(Math.abs(p1),Math.abs(p2)),Math.abs(p3)),Math.abs(p4)),1);
            max *= 1.0/(1.0 - DriveConstants.kStatic);
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            p1 += DriveConstants.kStatic * Math.signum(p1);
            p2 += DriveConstants.kStatic * Math.signum(p2);
            p3 += DriveConstants.kStatic * Math.signum(p3);
            p4 += DriveConstants.kStatic * Math.signum(p4);
            drive.pinMotorPowers(p1, p2, p3, p4);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
}
