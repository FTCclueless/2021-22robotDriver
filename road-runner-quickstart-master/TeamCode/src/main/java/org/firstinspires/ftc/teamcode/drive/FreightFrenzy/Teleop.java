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

    int hub = 0;
    Pose2d endPoint = new Pose2d();
    Pose2d hubLocation = new Pose2d();
    double height = 0;
    double radius = 0;
    int level = 3;

    double side = 0;
    boolean intake = false;

    boolean done = false;
    int flag = 0;

    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastToggleHub = false;

    ButtonToggle endgame = new ButtonToggle();
    ButtonToggle spin = new ButtonToggle();
    ButtonToggle odo = new ButtonToggle();

    long startReposition = System.currentTimeMillis();
    int lastLocVal = 0;
    int locVal = 0;

    double armInPos = 0.237;
    double armOutPos = 0.403;
    double armOutGrabPos = 0.803;
    boolean first = true;
    boolean lastIn = false;
    boolean lastOut = false;
    boolean armIn = true;

    long startDuckSpin = System.currentTimeMillis();
    double duckSpinPower = 0.35;
    long start;

    Pose2d sharedHubEndpoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
    Pose2d allianceHubEndpoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.servos.get(5).setPosition(0.237); //Capstone servo

        while (!isStarted()){
            if (gamepad1.dpad_up){
                side = -1;
                intake = true;
            }
            if (gamepad1.dpad_down){
                side = 1;
                intake = false;
            }
            telemetry.addData("side", side);
            telemetry.update();
            drive.update();
        }

        waitForStart();
        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(36.5,65.25 * side,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        start = System.currentTimeMillis();

        while (!isStopRequested()) {
            drive.update();

            if (gamepad1.b){ //Stops everything and makes turret face forward
                drive.slidesCase = 0;
                drive.intakeCase = 0;
                drive.currentIntake = 0;
            }
            if (gamepad1.x){ //Finish intaking
                drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                drive.intakeCase = 6;
            }

            updateEndgame();

            if(gamepad1.right_bumper) {//Starts the deposit sequence
                drive.startDeposit(endPoint, hubLocation, height, radius);
            }
            if(gamepad1.right_trigger >= 0.5) {//Starts the intake sequence
                drive.startIntake(intake);
            }
            if (gamepad2.right_trigger >= 0.5){//Makes it deposit
                drive.deposit();
                //Updates the target location for auto movement upon deposit
                if (auto.getToggleState()) {
                    if (hub == 1) {
                        allianceHubEndpoint = new Pose2d(drive.currentPose.getX(), drive.currentPose.getY(), drive.currentPose.getHeading());
                    }
                    if (hub == 0) {
                        sharedHubEndpoint = new Pose2d(drive.currentPose.getX(), drive.currentPose.getY(), drive.currentPose.getHeading());
                    }
                }
            }

            capstone();
            updatePoseLock();
            updateHub();

            odo.update(gamepad2.b);//updates the lifting of the odometry
            if (odo.getToggleState()){ //Lifts up odo and tells robot we no longer know where we are
                drive.servos.get(3).setPosition(0.668);
                drive.isKnownY = false;
                drive.isKnownX = false;
                auto.toggleState = false; //Since we don't know where we are, we cannot run auto
            }
            else { //drops down odo
                drive.servos.get(3).setPosition(0.48);
            }

            updateAuto();

            if(Math.abs(gamepad2.left_stick_x) > 0.25) { // Updates the turret & forces it to not have too high of a target angle that it would be impossible to reach
                drive.turretOffset -= Math.toRadians(gamepad2.left_stick_x) * 0.25;
                double maxPossibleTurretAngle = Math.toRadians(100);//Todo: fix value
                if (Math.abs(drive.targetTurretHeading + drive.turretOffset) > maxPossibleTurretAngle){
                    drive.turretOffset = Math.abs(maxPossibleTurretAngle - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) { // Updates the slide length
                drive.slidesOffset -= gamepad2.right_stick_y * 0.4;
                double maxPossibleSlideExtension = 52;//Todo: fix value
                if(drive.targetSlideExtensionLength + drive.slidesOffset > maxPossibleSlideExtension) {
                    drive.slidesOffset = maxPossibleSlideExtension - drive.targetSlideExtensionLength;
                }
            }

            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * 0.4;
            if (!gamepad1.left_stick_button){ //Normal mode (press button to sprint)
                forward *= 0.65;
                left *= 0.65;
            }
            //Sets power for all of the drive motors
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);
        }
    }
    public void updateEndgame(){
        endgame.update(gamepad2.left_bumper);
        if (System.currentTimeMillis() - start >= 90000){
            endgame.toggleState = true;
        }
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
                if (a > 1500){
                    spin.toggleState = false;
                }
            }
            else{
                drive.servos.get(7).setPosition(0.938);
                drive.duckSpin.setPower(0);
                drive.duckSpin2.setPower(0);
                startDuckSpin = System.currentTimeMillis();
                duckSpinPower = 0.35;
            }
        }
    }
    public void capstone(){
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
    public void updatePoseLock(){
        if (startReposition - System.currentTimeMillis() > 1000){ //If over a second since last value input then we stop the ability to add to it
            locVal = 0;
        }
        int a = 0;
        boolean[] gamepad1Dpad = {gamepad1.dpad_right,gamepad1.dpad_left,gamepad1.dpad_up,gamepad1.dpad_down};
        for(int i = 0; i < gamepad1Dpad.length; i ++){ //Finds the current input
            if (gamepad1Dpad[i]){
                a = i + 1;
                startReposition = System.currentTimeMillis();
            }
        }
        if (a != lastLocVal && a != 0){ //Testing for new input
            if ((a == 1 && locVal == 1) || (a == 1 && locVal == 4) || (a == 4 && locVal == 1)){ //right
                drive.localizer.setPoseEstimate(new Pose2d(24,-65.25,Math.toRadians(0)));
            }
            if ((a == 2 && locVal == 2) || (a == 2 && locVal == 4) || (a == 4 && locVal == 2)){ //left
                drive.localizer.setPoseEstimate(new Pose2d(24,65.25,Math.toRadians(0)));
            }
            if ((a == 1 && locVal == 3) || (a == 3 && locVal == 1)){ //top right
                drive.localizer.setPoseEstimate(new Pose2d(65.25,-24,Math.toRadians(-90)));
            }
            if ((a == 2 && locVal == 3) || (a == 3 && locVal == 2)){ //top left
                drive.localizer.setPoseEstimate(new Pose2d(65.25,24,Math.toRadians(90)));
            }
            locVal = a;
        }
        lastLocVal = a;
    }
    public void updateAuto(){
        auto.update(gamepad1.a);
        if (auto.toggleState && !done){
            done = true;
            if (flag == 0) {
                if (drive.intakeCase == 0) {
                    drive.startIntake(intake);
                }
                if (! (Math.abs(drive.currentPose.getY()) > 72-43.5 && drive.currentPose.getX() > 72-43.5)) { // Don't need to drive into the area if we are already inside
                    if (hub == 1) {
                        driveToPoint(new Pose2d(16.5, endPoint.getY(), endPoint.getHeading()),1000);
                        driveToPoint(new Pose2d(36.5, endPoint.getY(), endPoint.getHeading()),1000);
                    }
                    if (hub == 0) {
                        driveToPoint(new Pose2d(endPoint.getX(), 16.5 * side, endPoint.getHeading()),1000);
                        driveToPoint(new Pose2d(endPoint.getX(), 36.5 * side, endPoint.getHeading()),1000);
                    }
                }
            }
            else { //This is for going toward deposit area
                if (drive.slidesCase == 0) {
                    drive.startDeposit(endPoint, hubLocation, height, radius);
                }
                if (hub == 1) {
                    driveToPoint(new Pose2d(36.5,allianceHubEndpoint.getY(),endPoint.getHeading()),1000);
                    driveToPoint(allianceHubEndpoint,1000);
                }
                if (hub == 0) {
                    driveToPoint(new Pose2d(sharedHubEndpoint.getX(),36.5*side,endPoint.getHeading()),1000);
                    driveToPoint(sharedHubEndpoint,1000);
                }
            }
        }
        if (drive.intakeCase == 3 && drive.lastIntakeCase == 2){ // Just took in a block
            flag = 1;
            done = false;
        }
        if (drive.slidesCase == 5 && drive.lastSlidesCase == 4){ // Just finished the deposit
            flag = 0;
            done = false;
        }
    }
    public void updateHub(){
        boolean toggleHub = gamepad1.y;
        if (toggleHub && !lastToggleHub){
            if (!endgame.getToggleState()) { //TODO: Think about removing endgame (especially since we aren't really using it)
                hub = (hub + 1) % 2;
            }
            else {
                hub = (hub + 1) % 3;
            }
        }
        lastToggleHub = toggleHub;
        switch(hub) {
            case 0:
                endPoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
                hubLocation = new Pose2d(48, 0);
                drive.currentIntake = -side;
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
                boolean up = gamepad1.left_bumper;
                boolean down = gamepad1.left_trigger > 0.5;
                if (!lastDown && up){
                    level = Math.min(level + 1,3);
                }
                else if (!lastUp && down){
                    level = Math.max(level - 1,1);
                }
                lastUp = up;
                lastUp = down;

                switch (level){
                    case 1: radius = 8.5; height = 6; break;
                    case 2: radius = 7; height = 12.13; break;
                    case 3: radius = 3; height = 14; break;
                }
                hubLocation = new Pose2d(-12.0, 24.0*side);

                intake = side == -1;
                drive.currentIntake = side;
                break;
        }
    }
    public void driveToPoint(Pose2d target, long maxTime){
        double maxPowerForward = 0.8;
        double maxPowerTurn = 0.4;
        double slowDownDist = 4;
        double slowTurnAngle = Math.toRadians(8);
        drive.targetPose = target;
        drive.targetRadius = 2;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > 2 || Math.abs(drive.currentPose.getY()-target.getY()) > 2) && auto.getToggleState() && System.currentTimeMillis() - start < maxTime){
            auto.update(gamepad1.a);
            drive.update();
            drive.updateMotors(drive.getRelError(target),maxPowerForward,maxPowerTurn,slowDownDist,slowTurnAngle,2);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
}
