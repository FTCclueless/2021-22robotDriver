package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "Comp Drive")
public class Teleop extends LinearOpMode {
    SampleMecanumDrive drive;

    int hub = 1;

    public static double duckSpinSpeed = 0.2;
    public static int duckSpinTime = 1900;
    public static int duckIntakeTime = 1870;
    long duckRegrabTime = System.currentTimeMillis();
    public static double duckInPos = 0;
    public static double duckOutPos = 0.801;
    public static double duckIntakeReceiving = 0.32;
    public static double duckIntakeReceivingOut = 0.52;
    public static double duckIntakeMaxIn = 0.18;

    boolean firstUpdate = false;

    Pose2d endPoint = new Pose2d();
    Pose2d hubLocation = new Pose2d();
    double height = 0;
    double radius = 0;

    double side = 1;
    boolean intake = false;

    boolean done = false;

    boolean duck = false;
    int flag = 0;

    ButtonToggle auto = new ButtonToggle();
    boolean firstAutoUpdate = false;

    ButtonToggle endgame = new ButtonToggle();
    ButtonToggle spin = new ButtonToggle();
    ButtonToggle odo = new ButtonToggle();
    ButtonToggle extendSlides = new ButtonToggle();
    ButtonToggle secondGamepadLevel = new ButtonToggle();
    ButtonToggle secondHubLevel = new ButtonToggle();
    ButtonToggle roboctopi = new ButtonToggle();

    double armInPosRight = 0.0;
    double armOutPosRight = 0.172;
    double armOutGrabPosRight = 0.514;
    double armInPosLeft = 0.89;
    double armOutPosLeft = 0.685;
    double armOutGrabPosLeft = 0.356;
    double armInPos = armInPosRight;
    double armOutPos = armOutPosRight;
    double armOutGrabPos = armOutGrabPosRight;
    boolean first = false; //true;
    boolean lastIn = false;
    boolean lastOut = false;
    boolean armIn = true;

    boolean endgameRumble = false;
    boolean parkRumble = false;
    boolean lastRightTrigger = false;

    int matchTime = 122000;
    int teleopTime = 92000;
    int parkTime = 4500;

    boolean lastIntake = false;

    long startDuckSpin = System.currentTimeMillis();
    double duckSpinPower = 0.25;
    long start;

    long lastArmDown = System.currentTimeMillis();

    double speedSlowMultiplier = 1;

    boolean firstShared = false;
    boolean firstAlliance = false;

    Pose2d sharedHubEndpoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
    Pose2d allianceHubEndpoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));

    ElapsedTime runtime = new ElapsedTime();

    boolean startDuck = false;
    long startDuckTime = System.currentTimeMillis();

    boolean lastY = false;

    boolean duckOut = false;

    long duckFixTime = System.currentTimeMillis();
    long duckWaitTime = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        extendSlides.toggleState = true;
        secondGamepadLevel.toggleState = true;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.servos.get(5).setPosition(armInPosRight);

        drive.v4barOffset = Math.toRadians(-10);

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

        runtime.reset();

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
        if (side == -1){
            duckInPos = 1.0 - 0.0;
            duckOutPos = 1.0 - 0.801;
            duckIntakeReceiving = 1.0 - 0.32;
            duckIntakeReceivingOut = 1.0 - 0.52;
            duckIntakeMaxIn = 1.0 - 0.18;
            armInPos = armInPosLeft;
            armOutPos = armOutPosLeft;
            armOutGrabPos = armOutGrabPosLeft;
        }

        drive.servos.get(6).setPosition(duckInPos);

        sharedHubEndpoint = new Pose2d(65.125, 16 * side, Math.toRadians(90) * side);
        allianceHubEndpoint = new Pose2d(12, 65.125 * side, Math.toRadians(0));

        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(45,65.25 * side,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        start = System.currentTimeMillis();

        Gamepad.RumbleEffect customRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)
                .addStep(0.0, 0.0, 250)
                .addStep(1.0, 0.0, 500)
                .build();

        while (!isStopRequested()) {
            updateEndgame();

            roboctopi.update(gamepad1.y);

            secondGamepadLevel.update(gamepad2.x);

            if (gamepad2.a) {
                if (!secondHubLevel.getToggleState()) {
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                }
                else {
                    drive.v4barOffset = Math.toRadians(40);
                    drive.slidesOffset = -5;
                }
                drive.turretOffset = 0;
            }
            boolean y = gamepad2.y;
            if (secondGamepadLevel.getToggleState()) {
                extendSlides.update(gamepad2.b);
                secondHubLevel.update(y);
                if (y && !lastY && secondHubLevel.getToggleState()){
                    drive.v4barOffset = Math.toRadians(40);
                    drive.slidesOffset = -5;
                    drive.turretOffset = 0;
                }
                else if (!y && lastY && !secondHubLevel.getToggleState()){
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                    drive.turretOffset = 0;
                }
                if (gamepad2.right_bumper){
                    drive.startDeposit(endPoint, hubLocation, height, radius);
                }
            }
            else {
                odo.update(gamepad2.b);//updates the lifting of the odometry
                endgame.update(gamepad2.right_bumper);
            }
            if (!spin.getToggleState()) {
                spin.update(gamepad1.left_trigger >= 0.5);
            }
            lastY = y;

            drive.update();

            if (gamepad1.b){ //Stops everything and makes turret face forward
                drive.slidesCase = 0;
                drive.intakeCase = 0;
                drive.currentIntake = 0;
                drive.transferMineral = false;
                auto.toggleState = false;
                firstUpdate = false;
            }
            if (gamepad1.x){ //Finish intaking
                drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                drive.intakeCase = 3;
                firstUpdate = true;
            }
            if (extendSlides.getToggleState()) {
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
            boolean rightTrigger = gamepad2.right_trigger >= 0.5;
            if (rightTrigger && !lastRightTrigger){//Makes it deposit
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
            lastRightTrigger = rightTrigger;

            capstone();
            updatePoseLock();
            updateHub();

            if (odo.getToggleState()){ //Lifts up odo and tells robot we no longer know where we are
                drive.raiseOdo();
                drive.isKnownY = false;
                drive.isKnownX = false;
                auto.toggleState = false; //Since we don't know where we are, we cannot run auto
            }
            else { //drops down odo
                drive.dropOdo();
            }

            updateAuto();
            double leftStickX = gamepad2.left_stick_x;
            if(Math.abs(leftStickX) > 0.15) { // Updates the turret & forces it to not have too high of a target angle that it would be impossible to reach
                drive.turretOffset -= Math.toRadians(leftStickX) * 0.375;
                if (drive.targetTurretHeading + drive.turretOffset >  Math.toRadians(62.6)){ //2 degrees less than before = no more stalling
                    drive.turretOffset = Math.toRadians(62.6) - drive.targetTurretHeading;
                }
                else if (drive.targetTurretHeading + drive.turretOffset < Math.toRadians(-59.33)){
                    drive.turretOffset = Math.toRadians(-59.33) - drive.targetTurretHeading;
                }
            }

            double rightStickY = gamepad2.right_stick_y;
            if(Math.abs(rightStickY) > 0.25) { // Updates the slide length
                drive.slidesOffset -= rightStickY * 0.2;
                double maxPossibleSlideExtension = 52;
                if(drive.targetSlideExtensionLength + drive.slidesOffset > maxPossibleSlideExtension) {
                    drive.slidesOffset = maxPossibleSlideExtension - drive.targetSlideExtensionLength;
                }
            }

            if (gamepad2.left_bumper){ //1.2
                drive.v4barOffset -= Math.toRadians(2);
            }
            if (gamepad2.left_trigger >= 0.5){
                drive.v4barOffset += Math.toRadians(2);
            }

            double f = gamepad1.left_stick_y;
            double l = gamepad1.left_stick_x;
            double kStatic = 0.15;
            if (gamepad1.right_bumper){ //Normal mode (press button to sprint)
                f *= 0.25;
                l *= 0.5;
            }

            double forward = Math.pow(f,7) * -(0.85 - kStatic) * speedSlowMultiplier + Math.signum(-f) * Math.max(Math.signum(Math.abs(f) - 0.1),0) * kStatic;
            double left = Math.pow(l,7) * (1.0 - kStatic) * speedSlowMultiplier + Math.signum(l) * Math.max(Math.signum(Math.abs(l) - 0.1),0) * kStatic;
            double turn = gamepad1.right_stick_x * 0.45 * speedSlowMultiplier;

            if (gamepad1.left_bumper){
                double m1 = -1;
                if (hub == 0){
                    m1 = 1;
                }
                left = 0.6 * side * m1;
            }

            //Sets power for all of the drive motors
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            if(!endgameRumble){
                if (runtime.milliseconds() - teleopTime >= -500){
                    if(!gamepad1.isRumbling() && !gamepad2.isRumbling()){
                        gamepad1.runRumbleEffect(customRumble);
                        gamepad2.runRumbleEffect(customRumble);
                        endgameRumble = true;
                    }
                }
            }
            
            if(!parkRumble){
                if (matchTime - runtime.milliseconds() <= parkTime){
                    if(!gamepad1.isRumbling() && !gamepad2.isRumbling()){
                        gamepad1.rumbleBlips(4);
                        gamepad2.rumbleBlips(4);
                        parkRumble = true;
                    }
                }
            }


            telemetry.addData("hub", hub);
            telemetry.update();
        }
    }
    public void updateEndgame(){
        if (endgame.getToggleState()){
            hub = 2;
            drive.servos.get(7).setPosition(0.471);
            firstUpdate = true;
            drive.intakeLiftDelay = 100;
            if (drive.intakeCase == 1 || drive.intakeCase == 2 || drive.slidesCase >= 5){
                drive.intake.setPower(drive.intakePower);
                if (duck && System.currentTimeMillis() - startDuckTime <= duckIntakeTime - 600){
                    if (System.currentTimeMillis() - duckRegrabTime >= 600) {
                        drive.servos.get(6).setPosition(duckIntakeReceivingOut);
                        duck = false;
                    } else {
                        drive.servos.get(6).setPosition(duckIntakeMaxIn);
                    }
                }
                if (System.currentTimeMillis() - startDuckTime >= duckIntakeTime + 600) {
                    drive.servos.get(6).setPosition(duckIntakeReceivingOut);
                } else if (System.currentTimeMillis() - startDuckTime >= duckIntakeTime) {
                    drive.servos.get(6).setPosition(duckIntakeMaxIn);
                } else {
                    drive.servos.get(6).setPosition(duckIntakeReceiving);
                }
            }
            else {
                if (System.currentTimeMillis() - startDuckSpin >= duckIntakeTime){
                    duck = true;
                }
                duckRegrabTime = System.currentTimeMillis();
            }
            if (drive.intakeCase == 8){
                if (side == 1){
                    drive.servos.get(0).setPosition(drive.rightIntakeDrop);
                }
                else {
                    drive.servos.get(1).setPosition(drive.leftIntakeDrop);
                }
                drive.servos.get(6).setPosition(duckIntakeReceiving);
            }
            if (spin.getToggleState()){
                long a = System.currentTimeMillis() - startDuckSpin;
                if (!startDuck){
                    startDuck = true;
                    startDuckTime = System.currentTimeMillis();
                }
                drive.duckSpin.setPower(-duckSpinSpeed * side);
                drive.duckSpin2.setPower(-duckSpinSpeed * side);
                if (a > duckSpinTime){//1550
                    drive.duckSpin.setPower(0);
                    drive.duckSpin2.setPower(0);
                    startDuckSpin = System.currentTimeMillis();
                    spin.toggleState = false;
                }
            }
            else{
                startDuck = false;
                startDuckSpin = System.currentTimeMillis();
                duckSpinPower = 0.25;
            }
        }
        else{
            drive.intakeLiftDelay = 0;
        }
    }
    public void capstone(){
        boolean in = gamepad2.dpad_up;
        boolean out = gamepad2.dpad_down;
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
            else {
                first = false;
                if (side == 1){
                    armOutPos = armOutPosRight;
                }
                else{
                    armOutPos = armOutPosLeft;
                }
            }
            lastArmDown = System.currentTimeMillis();
        }
        lastIn = in;
        lastOut = out;
        if (armIn){
            if (gamepad2.dpad_right){
                armInPos += 0.001 * side;
            }
            else if (gamepad2.dpad_left){
                armInPos -= 0.001 * side;
            }
            armInPos = Math.max(Math.min(armInPos,1.0),0);
            drive.servos.get(5).setPosition(armInPos);
            speedSlowMultiplier = 1;
        }
        else {
            speedSlowMultiplier = 0.5;
            if (first){
                if (gamepad2.dpad_right){
                    armOutGrabPos += 0.001 * side;
                }
                else if (gamepad2.dpad_left){
                    armOutGrabPos -= 0.001 * side;
                }
                armOutGrabPos = Math.max(Math.min(armOutGrabPos,1.0),0);
                drive.servos.get(5).setPosition(armOutGrabPos);
            }
            else {
                if (gamepad2.dpad_right){
                    armOutPos += 0.002 * side;
                }
                else if (gamepad2.dpad_left){
                    armOutPos -= 0.002 * side;
                }
                armOutPos = Math.max(Math.min(armOutPos,1.0),0);
                drive.servos.get(5).setPosition(armOutPos);
            }
        }
    }

    public void updatePoseLock(){
        if (gamepad1.dpad_up) {
            firstUpdate = true;
            hub = 0;
            drive.localizer.setPoseEstimate(new Pose2d(65.25,24 * side,Math.toRadians(90)*side));
        }
        if (gamepad1.dpad_down){
            firstUpdate = true;
            hub = 1;
            drive.localizer.setPoseEstimate(new Pose2d(24,65.25 * side,Math.toRadians(0)));
        }
    }
    public void updateAuto(){
        boolean a = gamepad1.a;
        auto.update(a);
        
        if (roboctopi.getToggleState()){
            odo.toggleState = true;
            auto.toggleState = false;
            hub = 3;
            firstUpdate = true;
        }
        else {
            if (hub == 3){
                hub = 1;
            }
        }

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
            drive.updateImuHeading();
            if (flag == 0) {
                if (drive.intakeCase == 0) {
                    drive.startIntake(intake);
                }
                if (! (Math.abs(drive.currentPose.getY()) > 72-43.5 && drive.currentPose.getX() > 72-43.5)) { // Don't need to drive into the area if we are already inside
                    if (hub == 1) {
                        driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 3.5, endPoint.getY(), endPoint.getHeading()),new Pose2d(allianceHubEndpoint.getX() + 6.5, endPoint.getY(), endPoint.getHeading()),1000,false);
                        driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 26.5, endPoint.getY(), endPoint.getHeading()),new Pose2d(72, 48 * side, endPoint.getHeading()),1000,false);
                    }
                }
            }
            else { //This is for going toward deposit area
                if (extendSlides.getToggleState()) {
                    drive.startDeposit(endPoint, hubLocation, height, radius);
                }
                if (hub == 1) {
                    if (Math.abs(drive.clipHeading(drive.currentPose.getHeading())) >= Math.toRadians(45)){
                        driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 32,allianceHubEndpoint.getY() - 10 * side,endPoint.getHeading()), new Pose2d(allianceHubEndpoint.getX() + 28.5,allianceHubEndpoint.getY() - 8 * side,endPoint.getHeading()),1000,false);
                    }
                    driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 28.5,allianceHubEndpoint.getY(),endPoint.getHeading()), new Pose2d(allianceHubEndpoint.getX() + 25.5,allianceHubEndpoint.getY(),endPoint.getHeading()),1000,false);
                    driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 4,allianceHubEndpoint.getY(),allianceHubEndpoint.getHeading()), allianceHubEndpoint,1000,false);
                    waitForDeposit(allianceHubEndpoint);
                }
                if (hub == 0) {
                    if (Math.abs(drive.clipHeading(drive.currentPose.getHeading() + Math.toRadians(45) * side)) >= Math.toRadians(45)){
                        driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 32,allianceHubEndpoint.getY() - 10 * side,endPoint.getHeading()), new Pose2d(allianceHubEndpoint.getX() + 28.5,allianceHubEndpoint.getY() - 8 * side,endPoint.getHeading()),1000,false);
                    }
                    driveToPoint(new Pose2d(sharedHubEndpoint.getX(),38.5*side,endPoint.getHeading()),new Pose2d(sharedHubEndpoint.getX(),40.5*side,endPoint.getHeading()),1000,true);
                    driveToPoint(sharedHubEndpoint,1000,true);
                    waitForDeposit(sharedHubEndpoint);
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
            drive.startIntake(intake);
            if (secondHubLevel.getToggleState()) {
                drive.v4barOffset = Math.toRadians(40);
                drive.slidesOffset = -5;
            }
        }
    }
    public void updateHub(){
        switch(hub) {
            case 0:
                firstAlliance = true;
                endPoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
                hubLocation = new Pose2d(48, 0);
                if (firstUpdate) {
                    drive.currentIntake = -side;
                }
                if (firstShared) {
                    Log.e("Activate", "Shared");
                    if (side == -1) {
                        drive.turretOffset = Math.toRadians(3);
                    } else {
                        drive.turretOffset = -Math.toRadians(3);
                    }
                    drive.slidesOffset = 0;
                    drive.v4barOffset = 0;
                    odo.toggleState = false;
                }
                intake = side == 1;
                height = 2;
                radius = 2;
                firstShared = false;
                break;
            case 1: case 2:
                firstShared = true;
                if (hub == 1) {
                    endPoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));
                }
                else {
                    endPoint = new Pose2d(-44, 65.25 * side, Math.toRadians(180));
                }
                radius = 3;
                height = 14;
                hubLocation = new Pose2d(-12.0, 24.0*side);

                intake = (side == -1) ^ (hub == 2); //side == -1
                if (firstUpdate) {
                    double a = side;
                    if (hub == 2){
                        a *= -1;
                    }
                    drive.currentIntake = a;
                }
                if (firstAlliance) {
                    drive.intakeCase = 0;
                    if (hub == 1 && duckOut){
                        drive.intakeCase = -1;
                        duckFixTime = System.currentTimeMillis();
                    }
                    Log.e("Activate", "Alliance");
                    drive.turretOffset = 0;
                    drive.slidesOffset = 0;
                    drive.v4barOffset = 0;
                    odo.toggleState = false;
                }
                firstAlliance = false;
                break;
            case 3:
                endPoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));
                hubLocation = new Pose2d(-12, 65.25*side);
                radius = 3;
                height = 10;
                firstShared = true;
                firstAlliance = true;
                if (firstUpdate) {
                    drive.currentIntake = side;
                }
                break;
        }
        if (hub == 0 || hub == 2){
            if (drive.intakeCase == 1) {
                duckOut = true;
                if (System.currentTimeMillis() - duckWaitTime >= 75) {
                    if (hub == 0) {
                        drive.servos.get(6).setPosition(duckOutPos - 0.2 * side);
                    }
                    else {
                        drive.servos.get(6).setPosition(duckIntakeReceivingOut);
                    }
                }
            }
            else {
                duckWaitTime = System.currentTimeMillis();
            }
            if (drive.intakeCase >= 3 || drive.intakeCase == 0 && duckOut){
                drive.servos.get(6).setPosition(duckOutPos);
            }
        }
        if (hub == 1 || hub == 3) {
            if (duckOut && System.currentTimeMillis() - duckFixTime >= 500) {
                duckOut = false;
                drive.intakeCase = 0;
            } else if (duckOut) {
                drive.servos.get(6).setPosition(duckInPos);
                if (side == 1) {
                    drive.servos.get(0).setPosition(drive.rightIntakeDrop);
                }
                if (side == -1) {
                    drive.servos.get(1).setPosition(drive.leftIntakeDrop);
                }
            }
        }
    }
    public void driveToPoint(Pose2d target, long maxTime, boolean shared){
        driveToPoint(target,target,maxTime,shared);
    }
    public void driveToPoint(Pose2d target, Pose2d target2, long maxTime, boolean shared){
        double error = 4;
        double power = 0.85;
        double slowDownDist = 10;
        boolean hugWall = true;
        double maxPowerTurn = 0.35;
        double slowTurnAngle = Math.toRadians(15);
        double m = 1;
        if (shared){
            m = -1;
        }
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.4;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (auto.getToggleState() && opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(5)) && System.currentTimeMillis() - start < maxTime){
            auto.update(gamepad1.a);
            updateHub();
            if (gamepad1.b){ //Stops everything and makes turret face forward
                drive.slidesCase = 0;
                drive.intakeCase = 0;
                drive.currentIntake = 0;
                drive.transferMineral = false;
                auto.toggleState = false;
                firstUpdate = false;
            }
            extendSlides.update(gamepad2.b);
            if (gamepad2.right_bumper){
                drive.startDeposit(endPoint, hubLocation, height, radius);
            }
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double sideKStatic = 0;
            if (hugWall){
                sideKStatic = 0.4 * side * m;
            }
            if (x || y){
                if (Math.abs(relError.getY()) < sideError && hugWall) {
                    sideKStatic =  0.25 * side * m;
                    double heading = relError.getHeading();
                    if (Math.abs(relError.getY()) < sideError / 2.0){
                        sideKStatic = 0.1 * side * m;
                        heading = 0;
                    }
                    relError = new Pose2d(relError.getX(), 0, heading);
                }
            }
            drive.updateMotors(relError,power,maxPowerTurn,slowDownDist,slowTurnAngle,0.5,sideError,sideKStatic);

            if (drive.intakeCase == 3 && drive.lastIntakeCase == 2){ // Just took in a block
                flag = 1;
                done = false;
            }
            if (drive.slidesCase == 5 && drive.lastSlidesCase == 4){ // Just finished the deposit
                flag = 0;
                done = false;
            }
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }

    public void waitForDeposit(Pose2d target){
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        boolean stop = (
                gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y ||
                        Math.abs(gamepad1.left_stick_x) >= 0.5 || Math.abs(gamepad1.left_stick_y) >= 0.5 || Math.abs(gamepad1.right_stick_x) >= 0.5 || Math.abs(gamepad1.right_stick_y) >= 0.5 ||
                        gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.right_trigger >= 0.5 || gamepad1.left_trigger >= 0.5 ||
                        gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left
        );
        while (flag == 1 && !stop && opModeIsActive()) {
            stop = (
                    gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y ||
                            Math.abs(gamepad1.left_stick_x) >= 0.5 || Math.abs(gamepad1.left_stick_y) >= 0.5 || Math.abs(gamepad1.right_stick_x) >= 0.5 || Math.abs(gamepad1.right_stick_y) >= 0.5 ||
                            gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.right_trigger >= 0.5 || gamepad1.left_trigger >= 0.5 ||
                            gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left
            );
            double leftStickX = gamepad2.left_stick_x;
            if(Math.abs(leftStickX) > 0.15) { // Updates the turret & forces it to not have too high of a target angle that it would be impossible to reach
                drive.turretOffset -= Math.toRadians(leftStickX) * 0.375;
                if (drive.targetTurretHeading + drive.turretOffset > Math.toRadians(60.6)){
                    drive.turretOffset = Math.abs(Math.toRadians(60.6) - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
                else if (drive.targetTurretHeading + drive.turretOffset < Math.toRadians(-60.6)){
                    drive.turretOffset = Math.abs(Math.toRadians(-60.6) - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
            }
            double rightStickY = gamepad2.right_stick_y;
            if(Math.abs(rightStickY) > 0.25) { // Updates the slide length
                drive.slidesOffset -= rightStickY * 0.2;
                double maxPossibleSlideExtension = 52;
                if(drive.targetSlideExtensionLength + drive.slidesOffset > maxPossibleSlideExtension) {
                    drive.slidesOffset = maxPossibleSlideExtension - drive.targetSlideExtensionLength;
                }
                if(drive.targetSlideExtensionLength + drive.slidesOffset < 0){
                    drive.slidesOffset = -drive.targetSlideExtensionLength;
                }
            }

            if (gamepad2.right_bumper){
                drive.startDeposit(endPoint, hubLocation, height, radius);
            }
            if (gamepad2.right_trigger >= 0.5){//Makes it deposit
                //firstUpdate = true;
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

            updateHub();

            extendSlides.update(gamepad2.b);

            if (gamepad2.left_bumper){ //1.2
                drive.v4barOffset -= Math.toRadians(2);
            }
            if (gamepad2.left_trigger >= 0.5){
                drive.v4barOffset += Math.toRadians(2);
            }

            if (gamepad2.a){
                if (!secondHubLevel.getToggleState()) {
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                }
                else {
                    drive.v4barOffset = Math.toRadians(10);
                    drive.slidesOffset = -10;
                }
                drive.turretOffset = 0;
            }
            drive.update();
            Pose2d error = drive.getRelError(target);
            if (Math.abs(error.getY()) <= 0.5){
                error = new Pose2d(error.getX(), 0, 0);
            }
            drive.updateMotors(error, 0.35, 0.25,5, Math.toRadians(8), 0.25, 0.5, 0);
            if (drive.slidesCase == 5 && drive.lastSlidesCase == 4){ // Just finished the deposit
                flag = 0;
                done = false;
            }
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
}
