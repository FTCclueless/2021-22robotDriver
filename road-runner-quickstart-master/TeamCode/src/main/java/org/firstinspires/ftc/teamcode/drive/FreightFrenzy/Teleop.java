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
import org.firstinspires.ftc.teamcode.drive.Logger;
import org.firstinspires.ftc.teamcode.drive.Reader;
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
    SampleMecanumDrive drive;

    int hub = 1;

    boolean firstUpdate = false;

    Pose2d endPoint = new Pose2d();
    Pose2d hubLocation = new Pose2d();
    double height = 0;
    double radius = 0;
    int level = 3;

    double side = 1;
    boolean intake = false;

    boolean done = false;
    int flag = 0;

    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastToggleHub = false;

    ButtonToggle auto = new ButtonToggle();
    boolean firstAutoUpdate = false;

    ButtonToggle endgame = new ButtonToggle();
    ButtonToggle spin = new ButtonToggle();
    ButtonToggle odo = new ButtonToggle();

    long startReposition = System.currentTimeMillis();
    int lastLocVal = 0;
    int locVal = 0;

    double armInPos = 0.0;
    double armOutPos = 0.172;
    double armOutGrabPos = 0.52;
    boolean first = false; //true;
    boolean lastIn = false;
    boolean lastOut = false;
    boolean armIn = true;

    boolean lastIntake = false;

    long startDuckSpin = System.currentTimeMillis();
    double duckSpinPower = 0.35;
    long start;

    boolean lastTrigger = false;

    long lastArmDown = System.currentTimeMillis();

    Pose2d sharedHubEndpoint = new Pose2d(65.125, 16 * side, Math.toRadians(90) * side);
    Pose2d allianceHubEndpoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.servos.get(5).setPosition(0.237); //Capstone servo

        Reader r = new Reader();
        String info = r.readFile("Alliance");

        switch (info){
            case "blue":
                side = 1;
                intake = false;
                break;
            case "red":
                side = -1;
                intake = true;
                break;
        }

        while (!isStarted() && !isStopRequested()){
            if (gamepad1.dpad_up){
                side = -1;
                intake = true;
            }
            if (gamepad1.dpad_down){
                side = 1;
                intake = false;
            }
            telemetry.addData("side", side);
            telemetry.addData("info from file", info);
            telemetry.update();
            drive.update();
        }

        Pose2d sharedHubEndpoint = new Pose2d(65.125, 16 * side, Math.toRadians(90) * side);
        Pose2d allianceHubEndpoint = new Pose2d(12, 65.125 * side, Math.toRadians(0));

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
                firstUpdate = false;
            }
            if (gamepad1.x){ //Finish intaking
                drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                drive.intakeCase = 6;
            }

            updateEndgame();

            if(gamepad1.right_bumper) {//Starts the deposit sequence
                firstUpdate = true;
                drive.startDeposit(endPoint, hubLocation, height, radius);
            }
            boolean currentIntake = gamepad1.right_trigger >= 0.5;
            if(currentIntake && !lastIntake) {//Starts the intake sequence
                firstUpdate = true;
                if (drive.intakeCase == 1 || drive.intakeCase == 2) { //Can cancel intake if it hasn't intaked yet
                    drive.intakeCase = 0;
                    drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                    drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                }
                else{ //If you are not canceling the intake, then buffer an intake
                    drive.startIntake(intake);
                }
            }
            lastIntake = currentIntake;
            if (gamepad2.right_trigger >= 0.5){//Makes it deposit
                firstUpdate = true;
                drive.deposit();
                Log.e("deposit", "depositCalled");
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
                double maxPossibleTurretAngle = Math.toRadians(100); //Todo: fix value
                if (Math.abs(drive.targetTurretHeading + drive.turretOffset) > maxPossibleTurretAngle){
                    drive.turretOffset = Math.abs(maxPossibleTurretAngle - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) { // Updates the slide length
                drive.slidesOffset -= gamepad2.right_stick_y * 0.2;
                double maxPossibleSlideExtension = 52;//Todo: fix value
                if(drive.targetSlideExtensionLength + drive.slidesOffset > maxPossibleSlideExtension) {
                    drive.slidesOffset = maxPossibleSlideExtension - drive.targetSlideExtensionLength;
                }
            }

            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * 0.4;
            if (!gamepad1.left_stick_button){ //Normal mode (press button to sprint)
                forward *= 0.6;
                left *= 0.6;
            }
            //Sets power for all of the drive motors
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            telemetry.addData("hub", hub);
            telemetry.addData("level", level);
            telemetry.update();
        }
    }
    public void updateEndgame(){
        endgame.update(gamepad2.left_bumper);
        //if (System.currentTimeMillis() - start >= 90000){
            //endgame.toggleState = true;
        //}
        if (endgame.getToggleState()){
            drive.servos.get(7).setPosition(0.467);
            spin.update(gamepad2.y);
            if (spin.getToggleState()){
                long a = System.currentTimeMillis() - startDuckSpin;
                if (a < 1100){ //900
                    drive.duckSpin.setPower(duckSpinPower * side);
                    drive.duckSpin2.setPower(-duckSpinPower * side);
                    duckSpinPower += drive.loopSpeed * 0.25; //0.00012

                }
                else{
                    drive.duckSpin.setPower(1 * side);
                    drive.duckSpin2.setPower(-1 * side);
                }
                if (a > 1600){//1500
                    spin.toggleState = false;
                }
            }
            else{
                drive.duckSpin.setPower(0);
                drive.duckSpin2.setPower(0);
                startDuckSpin = System.currentTimeMillis();
                duckSpinPower = 0.30;
            }
        }
        else{
            drive.servos.get(7).setPosition(0.938);
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
            if (System.currentTimeMillis() - lastArmDown <= 500){
                first = true;
            }
            lastArmDown = System.currentTimeMillis();
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
        boolean a = gamepad1.a;
        auto.update(a);
        if (a){
            firstUpdate = true;
            if (!firstAutoUpdate){ // Only updates on first time
                if (drive.currentPose.getX() < 56){ //checks to see if you actually want to go for the alliance hub instead of the shared hub
                    hub = 1;
                }
                else {
                    hub = 0;
                }
                updateHub();
            }
            firstAutoUpdate = true; // after auto has started we will assume that they do not need this anymore
        }
        if (auto.toggleState && !done){
            done = true;
            if (flag == 0) {
                if (drive.intakeCase == 0) {
                    drive.startIntake(intake);
                }
                if (! (Math.abs(drive.currentPose.getY()) > 72-43.5 && drive.currentPose.getX() > 72-43.5)) { // Don't need to drive into the area if we are already inside
                    if (hub == 1) {
                        driveToPoint(new Pose2d(16.5, endPoint.getY(), endPoint.getHeading()),new Pose2d(18.5, endPoint.getY(), endPoint.getHeading()),3000,false);
                        driveToPoint(new Pose2d(38.5, endPoint.getY(), endPoint.getHeading()),3000,false);
                    }
                    if (hub == 0) {
                        driveToPoint(new Pose2d(endPoint.getX(), 16.5 * side, endPoint.getHeading()),new Pose2d(endPoint.getX(), 18.5 * side, endPoint.getHeading()),3000,true);
                        driveToPoint(new Pose2d(endPoint.getX(), 38.5 * side, endPoint.getHeading()),3000,true);
                    }
                }
            }
            else { //This is for going toward deposit area
                drive.startDeposit(endPoint, hubLocation, height, radius);
                if (hub == 1) {
                    driveToPoint(new Pose2d(38.5,allianceHubEndpoint.getY(),endPoint.getHeading()),3000,false);
                    driveToPoint(allianceHubEndpoint,1000,false);
                }
                if (hub == 0) {
                    Log.e("here","leaving area");
                    driveToPoint(new Pose2d(sharedHubEndpoint.getX(),38.5*side,endPoint.getHeading()),3000,true);
                    driveToPoint(sharedHubEndpoint,1000,true);
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
            firstAutoUpdate = true; // if you intentionally click the button it thinks you remembered to click the button
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
                if (firstUpdate) {
                    drive.currentIntake = -side;
                }
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
                if (firstUpdate) {
                    drive.currentIntake = side;
                }
                break;
        }
    }
    public void driveToPoint(Pose2d target, long maxTime, boolean shared){
        driveToPoint(target,target,maxTime,shared);
    }
    public void driveToPoint(Pose2d target, Pose2d target2, long maxTime, boolean shared){
        double error = 2;
        double power = 0.8;
        double slowDownDist = 4;
        boolean hugWall = true;
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/1.6,kStatic * 1.5);
        double slowTurnAngle = Math.toRadians(5);
        double m = 1;
        if (shared){
            m = -1;
        }
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.3;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(3)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double sideKStatic = 0;
            if (hugWall){
                sideKStatic = 0.4 * side * m;
            }
            if (x || y){
                if (Math.abs(relError.getY()) < sideError) {
                    relError = new Pose2d(relError.getX(), 0, relError.getHeading());
                    sideKStatic = 0;
                }
                else if (Math.abs(relError.getX()) < error){
                    relError = new Pose2d(0, relError.getY(), relError.getHeading());
                }
            }
            drive.updateMotors(relError,power,maxPowerTurn,slowDownDist,slowTurnAngle,sideError,sideKStatic);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
}
