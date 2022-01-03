package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;
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

        double side = 1;
        boolean intake = false;

        drive.servos.get(5).setPosition(0.237);

        while (!isStarted()){
            if (gamepad1.dpad_up){
                side = -1;
                intake = true;
            }
            if (gamepad1.dpad_down){
                side = 1;
                intake = false;
            }
        }

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(36.5,65.25 * side,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        int hub = 0;

        Pose2d endPoint = new Pose2d();
        Pose2d hubLocation = new Pose2d();
        double height = 0;
        double r = 0;

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

        long startDuckSpin = System.currentTimeMillis();
        double duckSpinPower = 0.35;

        while (!isStopRequested()) {
            drive.update();
            switch(hub) {
                case 0: endPoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
                    hubLocation = new Pose2d(48, 0);
                    intake = side == 1;
                    height = 2;
                    radius = 2;
                        break;
                case 1: case 2:
                    if (hub == 1) {
                        endPoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));
                    }
                    else {
                        endPoint = new Pose2d(-56, 60 * side, Math.toRadians(180));
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
                        case 3: radius = 3; height = 14; break;
                    }
                    hubLocation = new Pose2d(-12.0, 24.0*side);
                    intake = side == -1;
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
                auto.toggleState = false;
                drive.servos.get(7).setPosition(0.467);
                spin.update(gamepad2.y);
                if (spin.getToggleState()){
                    long a = System.currentTimeMillis() - startDuckSpin;
                    if (a < 900){
                        drive.duckSpin.setPower(duckSpinPower * side);
                        drive.duckSpin2.setPower(-duckSpinPower * side);
                        duckSpinPower += 0.00012;

                    }
                    else{
                        drive.duckSpin.setPower(1 * side);
                        drive.duckSpin2.setPower(-1 * side);
                    }
                    if (a > 1500){ //TODO: tune this value
                        spin.toggleState = false;
                    }
                }
                else{
                    //TODO: find retracted location
                    drive.duckSpin.setPower(0);
                    drive.duckSpin2.setPower(0);
                    startDuckSpin = System.currentTimeMillis();
                    duckSpinPower = 0.35;
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
                    boolean inside = Math.abs(drive.currentPose.getY()) > 72-43.5 && drive.currentPose.getX() > 72-43.5;
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

            if (gamepad1.x){
                drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                drive.intakeCase = 6;
            }

            double lY1 = gamepad1.left_stick_y * -1;
            double lX1 = gamepad1.left_stick_x;
            double forward = lY1;
            double left = lX1;
            if (false){
                //Field-centric-controls
                forward = lY1 * Math.cos(drive.currentPose.getHeading()) + lX1 * Math.sin(drive.currentPose.getHeading());
                left = lX1 * Math.cos(drive.currentPose.getHeading()) - lY1 * Math.sin(drive.currentPose.getHeading());
            }
            double turn = gamepad1.right_stick_x * 0.4;
            if (!gamepad1.left_stick_button){
                forward *= 0.65;
                left *= 0.65;
            }
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            if(Math.abs(gamepad2.left_stick_x) > 0.25) {
                Log.e("turret offset", drive.turretOffset + "");
                drive.turretOffset -= Math.toRadians(gamepad2.left_stick_x) * 0.25;

                if(drive.turretOffset > 0.07) {
                    drive.turretOffset = 0.07;
                }

                if(drive.turretOffset < -2) {
                    drive.turretOffset = -2;
                }
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) {
                Log.e("slides offset", drive.slidesOffset + "");
                drive.slidesOffset -= gamepad2.right_stick_y * 0.4;

                if(drive.slidesOffset > 17) {
                    drive.slidesOffset = 17;
                }
            }
        }
    }
    public void driveToPoint(Pose2d target){
        double maxPowerForward = 0.8;
        double maxPowerTurn = 0.4;
        double slowDownDist = 4;
        double slowTurnAngle = Math.toRadians(8);
        drive.targetPose = target;
        drive.targetRadius = 2;
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > 2 || Math.abs(drive.currentPose.getY()-target.getY()) > 2) && auto.getToggleState()){
            auto.update(gamepad1.a);
            drive.update();
            updateMotors(getRelError(target),DriveConstants.kStatic,maxPowerForward,maxPowerTurn,slowDownDist,slowTurnAngle,2);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public Pose2d getRelError(Pose2d target){
        return new Pose2d(
                Math.cos(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()) + Math.sin(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()),
                Math.cos(drive.currentPose.getHeading()) * (target.getY()-drive.currentPose.getY()) - Math.sin(drive.currentPose.getHeading()) * (target.getX()-drive.currentPose.getX()),
                target.getHeading()-drive.currentPose.getHeading()
        );
    }
    public void updateMotors(Pose2d relError, double kStatic, double power, double maxPowerTurn, double slowDownDist, double slowTurnAngle, double error){
        double powerAdjust = power-kStatic;
        double turnAdjust = maxPowerTurn-kStatic;
        double forward = Math.min(Math.max(relError.getX()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getX()) * kStatic * Math.max(Math.signum(Math.abs(relError.getX()) - error),0);
        double left = Math.min(Math.max(relError.getY()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getY()) * kStatic * Math.max(Math.signum(Math.abs(relError.getY()) - error),0);
        double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/slowTurnAngle,-turnAdjust),turnAdjust) + Math.signum(relError.getHeading()) * kStatic * Math.max(Math.signum(Math.abs(relError.getHeading()) - slowTurnAngle),0);
        double [] p = new double[4];
        p[0] = forward-left-turn;
        p[1] = forward+left-turn;
        p[2] = forward-left+turn;
        p[3] = forward+left+turn;
        double max = (1.0 - kStatic);
        for (int i = 0; i < p.length; i ++){
            max = Math.max(Math.abs(p[i]),max) * (1.0 - kStatic);
        }
        for (int i = 0; i < p.length; i ++){
            p[i] *= max;
            p[i] += kStatic * Math.signum(p[i]);
        }
        drive.pinMotorPowers(p[0], p[1], p[2], p[3]);
    }
}
