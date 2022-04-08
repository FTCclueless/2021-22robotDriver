package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;
import android.widget.ToggleButton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

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

    boolean endgame = false;
    ButtonToggle extendSlides = new ButtonToggle();
    ButtonToggle secondHubLevel = new ButtonToggle();
    ButtonToggle barrier = new ButtonToggle();
    ButtonToggle a = new ButtonToggle();

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

    int cap = 0;
    boolean lastCap = false;

    boolean lastY = false;

    boolean duckOut = false;

    boolean firstEndgame = false;

    long duckFixTime = System.currentTimeMillis();
    long duckWaitTime = System.currentTimeMillis();

    long startB = System.currentTimeMillis();
    long startDepositCap = System.currentTimeMillis();
    boolean firstB = false;

    //TODO: Make sure the duck servo is working properly & make sure the defence button works
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        extendSlides.toggleState = true;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.servos.get(5).setPosition(armInPosRight);

        drive.v4barOffset = Math.toRadians(-10);

        //drive.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        //drive.blinkinLedDriver.setPattern(drive.pattern);

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
        else {
            duckInPos = 0;
            duckOutPos = 0.801;
            duckIntakeReceiving = 0.32;
            duckIntakeReceivingOut = 0.52;
            duckIntakeMaxIn = 0.18;
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

        boolean barrierStart = false;

        boolean lowShared = false;

        long sharedStartTime = System.currentTimeMillis();

        while (!isStopRequested()) {
            updateEndgame();

            if (endgame || barrier.getToggleState()){
                drive.raiseOdo();
            }
            else {
                drive.dropOdo();
            }

            barrier.update(gamepad1.y);

            if (barrier.getToggleState()){
                hub = 3;
                if (drive.intakeCase == 8 && drive.slidesCase == 0){
                    drive.setTurretTarget(0);
                }
                firstUpdate = true;
                extendSlides.toggleState = false;
                secondHubLevel.toggleState = false;
                a.update(gamepad1.a);
                if (barrierStart) {
                    barrierStart = false;
                    if (drive.slidesCase == 1 && !drive.transferMineral) {
                        drive.slidesCase = 0;
                    }
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                    drive.turretOffset = 0;
                }
            }
            else {
                if (hub == 3){
                    hub = 1;
                    extendSlides.toggleState = true;
                }
                barrierStart = true;
            }

            //Reset the offsets
            if (gamepad2.a) {
                if (hub == 0){
                    lowShared = false;
                    drive.v4barOffset = Math.toRadians(-196); //-166
                    drive.slidesOffset = 2;
                    drive.turretOffset = Math.toRadians(-3) * side;
                }
                else {
                    if (!secondHubLevel.getToggleState()) {
                        drive.v4barOffset = Math.toRadians(-10);
                        drive.slidesOffset = 0;
                    } else {
                        drive.v4barOffset = Math.toRadians(40);
                        drive.slidesOffset = -5;
                    }
                    drive.turretOffset = 0;
                }
            }
            //Start Endgame
            endgame = endgame || gamepad2.right_bumper;
            //Switch hub levels
            boolean y = gamepad2.y;
            secondHubLevel.update(y);
            if (hub != 0) {
                if (y && !lastY && secondHubLevel.getToggleState()) {
                    drive.v4barOffset = Math.toRadians(40);
                    drive.slidesOffset = -5;
                    drive.turretOffset = 0;
                } else if (!y && lastY && !secondHubLevel.getToggleState()) {
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                    drive.turretOffset = 0;
                }
            }
            lastY = y;
            //Slide extension code
            if (extendSlides.getToggleState() && cap == 0) {
                drive.startDeposit(endPoint, hubLocation, height, radius);
                if (drive.slidesCase == 5 && drive.lastSlidesCase == 4){
                    drive.startIntake(intake);
                }
            }
            else{
                drive.startSlides = false;
            }
            if (gamepad2.b){
                drive.startDeposit(endPoint, hubLocation, height, radius);
                drive.update();
                if (System.currentTimeMillis() - startB >= 1000){
                    if (drive.slidesCase == 1 && !drive.transferMineral){
                        drive.slidesCase = 0;
                    }
                    if (firstB){
                        firstB = false;
                        extendSlides.toggleState = ! extendSlides.getToggleState();
                    }
                }
            }
            else{
                startB = System.currentTimeMillis();
                firstB = true;
                drive.update();
            }
            //Duck Stuff
            if (intake != (side == -1)){//We are using the intake that has the duck flipper on it # hub == 0 || hub == 2
                if (drive.intakeCase == 1) { //We are using the intake but just dropping it
                    if (!duckOut && System.currentTimeMillis() - duckWaitTime >= 150) { //75; 380 is the max number
                        drive.servos.get(6).setPosition(duckOutPos);
                        if (System.currentTimeMillis() - duckWaitTime >= 300) {
                            duckOut = true; //It has officially come out
                        }
                    }
                    else if (duckOut){ // If it is already out then we can put it to the correct position
                        if (hub == 2) {
                            drive.servos.get(6).setPosition(duckIntakeReceivingOut);
                        }
                        else {
                            drive.servos.get(6).setPosition(duckOutPos - 0.2 * side);
                        }
                    }
                }
                else {
                    if (!duckOut){
                        drive.intakeCase = 0;
                    }
                    duckWaitTime = System.currentTimeMillis(); //Timer from when we start dropping the intake
                }
                if (duckOut) {
                    if (drive.intakeCase == 0){
                        drive.servos.get(6).setPosition(duckOutPos);
                    }
                    else if (drive.intakeCase == 8){
                        if (endgame){
                            drive.servos.get(6).setPosition(duckIntakeReceiving);
                        }
                        else{
                            drive.servos.get(6).setPosition(duckOutPos);
                        }
                    }
                    else if (drive.intakeCase >= 3) { //raising intake for transfer and duck is out
                        drive.servos.get(6).setPosition(duckOutPos); //Need the duck to be out to not interfere with transfer
                    } else if (endgame) {
                        if (gamepad1.left_trigger >= 0.5 || gamepad2.right_bumper) {
                            drive.servos.get(6).setPosition(duckIntakeMaxIn);
                        } else {
                            drive.servos.get(6).setPosition(duckIntakeReceiving);
                        }
                    }
                }
                duckFixTime = System.currentTimeMillis();
            }
            else { //This makes the duck servo come back in # if (hub == 1 || hub == 3)
                if (duckOut && System.currentTimeMillis() - duckFixTime >= 500) { // Stops the process after the robot has successfully pulled everything up
                    duckOut = false; //have successfully returned it
                    drive.intakeCase = 0;
                } else if (duckOut) {
                    drive.intakeCase = -1;
                    drive.servos.get(6).setPosition(duckInPos); //moves it in
                    if (side == 1) { //drops the correct intake down so the servo can move in
                        drive.servos.get(0).setPosition(drive.rightIntakeDrop);
                    }
                    else {
                        drive.servos.get(1).setPosition(drive.leftIntakeDrop);
                    }
                }
            }


            if (gamepad1.b){ //Stops everything and makes turret face forward
                drive.slidesCase = 0;
                drive.intakeCase = 0;
                drive.currentIntake = 0;
                drive.transferMineral = false;
                firstUpdate = false;
            }
            if (gamepad1.x){ //Finish intaking
                drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                drive.intakeCase = 3;
                firstUpdate = true;
            }
            boolean currentIntake = gamepad1.right_trigger >= 0.5;
            if(currentIntake && !lastIntake) {//Starts the intake sequence
                firstUpdate = true;
                if (drive.intakeCase == 1 || drive.intakeCase == 2) { //Can cancel intake if it hasn't intaked yet
                    drive.intakeCase = 0;
                    drive.servos.get(0).setPosition(drive.rightIntakeRaise);
                    drive.servos.get(1).setPosition(drive.leftIntakeRaise);
                }
                else if (drive.intakeCase == 3) {
                    //drive.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    //drive.blinkinLedDriver.setPattern(drive.pattern);
                }
                else{ //If you are not canceling the intake, then buffer an intake
                    drive.startIntake(intake);
                }
            }
            lastIntake = currentIntake;

            if (gamepad2.x && !lowShared){
                lowShared = true;
                drive.v4barOffset = 0;
                drive.slidesOffset = 0;
                drive.turretOffset = Math.toRadians(-3) * side;
            }

            boolean rightTrigger = gamepad2.right_trigger >= 0.5;
            if (rightTrigger && !lastRightTrigger){//Makes it deposit
                if (hub == 0 && !lowShared){
                    drive.effectiveDepositTime = 600;
                    sharedStartTime = System.currentTimeMillis();
                }
                else {
                    drive.effectiveDepositTime = 250;
                }
                drive.deposit();
            }
            if (System.currentTimeMillis() - sharedStartTime >= 200 && !lowShared && drive.slidesCase == 4 && hub == 0){
                drive.v4barOffset = Math.toRadians(-166); //156
            }
            if (drive.slidesCase == 6 && drive.lastSlidesCase == 5){
                if (hub == 0 && lowShared){
                    lowShared = false;
                    drive.v4barOffset = Math.toRadians(-196); //-166
                    drive.slidesOffset = 2;
                    drive.turretOffset = Math.toRadians(-3) * side;
                }
                else if (hub == 0){
                    lowShared = false;
                    drive.v4barOffset = Math.toRadians(-196); //-166
                }
                else if (barrier.getToggleState()){
                    drive.v4barOffset = Math.toRadians(-10);
                    drive.slidesOffset = 0;
                    drive.turretOffset = 0;
                }
                else if (secondHubLevel.getToggleState()){
                    drive.v4barOffset = Math.toRadians(40);
                    drive.slidesOffset = -5;
                    drive.turretOffset = 0;
                }
            }
            lastRightTrigger = rightTrigger;

            capstone();
            updatePoseLock();
            updateHub();


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
            if(Math.abs(rightStickY) > 0.15) { // Updates the slide length
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
                if (hub == 0 || hub == 3){
                    m1 = 1;
                }
                if (endgame){
                    m1 *= -1;
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
        if (endgame){
            barrier.toggleState = false;
            hub = 2;
            //first time
            if (!firstEndgame){
                firstEndgame = true;
                firstUpdate = false;
                extendSlides.toggleState = true;
                secondHubLevel.toggleState = false;
                drive.v4barOffset = Math.toRadians(-10);
                drive.slidesOffset = 0;
                drive.turretOffset = 0;
            }
            drive.servos.get(7).setPosition(0.471);
            //firstUpdate = true;
            drive.transfer1Time = 200;
            drive.transfer2Time = 250;
            drive.intakeLiftDelay = 100;
            /*
            if (drive.intakeCase == 2 || drive.slidesCase >= 5) {
                drive.intake.setPower(drive.intakePower);
            }
             */
            if (gamepad1.dpad_left){
                drive.duckSpin.setPower(-duckSpinSpeed * side);
                drive.duckSpin2.setPower(-duckSpinSpeed * side);
            }
            else {
                drive.duckSpin.setPower(0);
                drive.duckSpin2.setPower(0);
            }
            if (drive.intakeCase == 8){
                if (side == 1){
                    drive.servos.get(0).setPosition(drive.rightIntakeDrop);
                }
                else {
                    drive.servos.get(1).setPosition(drive.leftIntakeDrop);
                }
            }
        }
        else{
            firstEndgame = false;
            drive.intakeLiftDelay = 0;
            drive.transfer1Time = 300;
            drive.transfer2Time = 350;
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
            barrier.toggleState = false;
            drive.localizer.setPoseEstimate(new Pose2d(65.25,24 * side,Math.toRadians(90)*side));
        }
        if (gamepad1.dpad_down){
            firstUpdate = true;
            hub = 1;
            drive.localizer.setPoseEstimate(new Pose2d(24,65.25 * side,Math.toRadians(0)));
        }
    }
    public void updateHub(){
        if (hub == 0){
            drive.effectiveDepositAngle = Math.toRadians(-75);
        }
        else{
            drive.effectiveDepositAngle = Math.toRadians(-45);
        }
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
                    drive.turretOffset = Math.toRadians(-3) * side;
                    drive.slidesOffset = 2;
                    drive.v4barOffset = Math.toRadians(-196); //-166
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

                intake = (side == -1) ^ (hub == 2);

                boolean x = gamepad2.x;
                if (x && !lastCap && endgame){
                    cap ++;
                    switch (cap){
                        case 1:
                            drive.slidesOffset = 1;
                            drive.turretOffset = 0;
                            drive.v4barOffset = Math.toRadians(52); //56
                            break;
                        case 2:
                            drive.slidesOffset = 0;
                            drive.turretOffset = 0;
                            drive.v4barOffset = 0;
                            break;
                        case 4:
                            startDepositCap = System.currentTimeMillis();
                            break;
                    }
                }
                lastCap = x;

                if (cap > 0){
                    //drive.slidesCase = 3;
                    drive.transferMineral = true;
                    double h = 0;
                    drive.setV4barOrientation(drive.targetV4barOrientation + drive.v4barOffset);
                    drive.setTurretTarget(drive.targetTurretHeading + drive.turretOffset);
                    drive.setSlidesLength(drive.targetSlideExtensionLength + drive.slidesOffset);
                    switch (cap){
                        case 1:
                            drive.setDepositAngle(Math.toRadians(180));
                            drive.slidesCase = -1;
                            hubLocation = new Pose2d(-60.0, 35.25*side);
                            radius = 1;
                            height = 6; //Micheal thinks this is about 5
                            break;
                        case 2:
                            drive.setDepositAngle(Math.toRadians(90));
                            drive.slidesCase = -1;
                            hubLocation = new Pose2d(-20.0, 65.25*side);
                            radius = 1;
                            height = 10;
                            h = Math.toRadians(180);
                            break;
                        case 3:
                        case 4:
                            drive.slidesCase = -1;
                            hubLocation = new Pose2d(-12.0, 24*side);
                            radius = 5;
                            if (cap == 4){
                                if (System.currentTimeMillis() - startDepositCap >= 1500){
                                    drive.slidesOffset = 0;
                                    drive.turretOffset = 0;
                                    drive.v4barOffset = 0;
                                    drive.slidesCase = 5;
                                    cap = 0;
                                }
                                else if (System.currentTimeMillis() - startDepositCap >= 250){
                                    drive.slidesOffset -= 6 * drive.loopSpeed;
                                    drive.v4barOffset += Math.toRadians(40) * drive.loopSpeed;
                                    drive.setDepositAngle(Math.toRadians(180 - 45 * (System.currentTimeMillis() - startDepositCap - 500) / 1250.0));
                                }
                                else{
                                    drive.setDepositAngle(Math.toRadians(180));
                                }
                            }
                            else {
                                drive.setDepositAngle(Math.toRadians(115));
                            }
                            height = 16; //14
                            h = Math.toRadians(180);
                            break;
                    }
                    endPoint = new Pose2d(-44, 65.25 * side, h);
                    drive.startDeposit(endPoint,hubLocation,height,radius);
                }

                if (firstUpdate) {
                    double a = side;
                    if (hub == 2){
                        a *= -1;
                    }
                    drive.currentIntake = a;
                }
                if (firstAlliance) {
                    drive.intakeCase = 0;
                    Log.e("Activate", "Alliance");
                    drive.turretOffset = 0;
                    drive.slidesOffset = 0;
                    drive.v4barOffset = 0;
                }
                firstAlliance = false;
                break;
            case 3:
                endPoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));
                hubLocation = new Pose2d(-12, 65.25 * side);
                radius = 3;
                height = 10;
                firstShared = true;
                firstAlliance = true;

                intake = side == -1;
                if (a.getToggleState()){
                    intake = !intake;
                }

                if (firstUpdate) {
                    double m1 = 1;
                    if (a.getToggleState()){
                        m1 = -1;
                    }
                    drive.currentIntake = side * m1;
                }
                break;
        }
    }
}
