package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Reader;
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
    SampleMecanumDrive drive;

    int hub = 1;

    boolean firstUpdate = false;

    Pose2d endPoint = new Pose2d();
    Pose2d hubLocation = new Pose2d();
    double height = 0;
    double radius = 0;

    double side = 1;
    boolean intake = false;

    boolean done = false;
    int flag = 0;

    boolean lastToggleHub = false;

    ButtonToggle auto = new ButtonToggle();
    boolean firstAutoUpdate = false;

    ButtonToggle endgame = new ButtonToggle();
    ButtonToggle spin = new ButtonToggle();
    ButtonToggle odo = new ButtonToggle();

    double armInPosRight = 0.0;
    double armOutPosRight = 0.172;
    double armOutGrabPosRight = 0.521;
    double armInPosLeft = 0.89;
    double armOutPosLeft = 0.685;
    double armOutGrabPosLeft = 0.356;
    boolean first = false; //true;
    boolean lastIn = false;
    boolean lastOut = false;
    boolean armIn = true;

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

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      drive.servos.get(5).setPosition(armInPosRight);
//      drive.servos.get(6).setPosition(armInPosLeft);
        drive.servos.get(5).setPosition(armOutPosRight);
        drive.servos.get(6).setPosition(armOutPosLeft);

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

        sharedHubEndpoint = new Pose2d(65.125, 16 * side, Math.toRadians(90) * side);
        allianceHubEndpoint = new Pose2d(12, 65.125 * side, Math.toRadians(0));

        drive.intakeCase = 0;
        drive.lastIntakeCase = 0;
        drive.update();
        Pose2d startingPose = new Pose2d(45,65.25 * side,0);
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();

        start = System.currentTimeMillis();

        while (!isStopRequested()) {
            updateEndgame();
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
                drive.intakeCase = 6;
            }

            drive.startDeposit(endPoint, hubLocation, height, radius);
            //if(gamepad2.x) {//Starts the deposit sequence
                //firstUpdate = true;
            //}
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
                drive.raiseOdo();
                drive.isKnownY = false;
                drive.isKnownX = false;
                auto.toggleState = false; //Since we don't know where we are, we cannot run auto
            }
            else { //drops down odo
                drive.dropOdo();
            }

            updateAuto();

            if(Math.abs(gamepad2.left_stick_x) > 0.15) { // Updates the turret & forces it to not have too high of a target angle that it would be impossible to reach
                drive.turretOffset -= Math.toRadians(gamepad2.left_stick_x) * 0.5; // 0.15
                if (drive.targetTurretHeading + drive.turretOffset >  Math.toRadians(62.5953)){ //2 degrees less than before = no more stalling
                    drive.turretOffset = Math.abs(Math.toRadians(62.5953) - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
                else if (drive.targetTurretHeading + drive.turretOffset < Math.toRadians(-59.3259)){
                    drive.turretOffset = Math.abs(Math.toRadians(-59.3259) - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
            }

            if(Math.abs(gamepad2.right_stick_y) > 0.25) { // Updates the slide length
                drive.slidesOffset -= gamepad2.right_stick_y * 0.2;
                double maxPossibleSlideExtension = 52;//Todo: fix value
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

            if (gamepad2.a){
                drive.v4barOffset = Math.toRadians(-10);
                drive.slidesOffset = 0;
                drive.turretOffset = 0;
            }
            double f = gamepad1.left_stick_y;
            double l = gamepad1.left_stick_x;
            double kStatic = 0.15;
            double forward = Math.pow(f,7) * -(0.85 - kStatic) * speedSlowMultiplier + Math.signum(-f) * Math.max(Math.signum(Math.abs(f) - 0.1),0) * kStatic;
            double left = Math.pow(l,7) * (1.0 - kStatic) * speedSlowMultiplier + Math.signum(l) * Math.max(Math.signum(Math.abs(l) - 0.1),0) * kStatic;
            double turn = gamepad1.right_stick_x * 0.35 * speedSlowMultiplier;//4
            if (gamepad1.right_bumper){ //Normal mode (press button to sprint)
                forward *= 0.6;
                left *= 0.6;
            }
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

            telemetry.addData("hub", hub);
            telemetry.update();

            //drive.duckSpin.setPower(0);
        }
    }
    public void updateEndgame(){
        endgame.update(gamepad2.right_bumper);
        if (endgame.getToggleState()){
            drive.servos.get(7).setPosition(0.06);
            spin.update(gamepad2.y);
            if (spin.getToggleState()){
                long a = System.currentTimeMillis() - startDuckSpin;
                if (a < 900){ //900
                    drive.duckSpin.setPower(-duckSpinPower * side);//1.1
                    drive.duckSpin2.setPower(-duckSpinPower * side);
                    duckSpinPower += drive.loopSpeed * 0.2;//0.13

                }
                else {
                    drive.duckSpin.setPower(-1 * side);
                    drive.duckSpin2.setPower(-1 * side);
                }
                if (a > 1500){//1500
                    drive.duckSpin.setPower(0);
                    drive.duckSpin2.setPower(0);
                    startDuckSpin = System.currentTimeMillis();
                    spin.toggleState = false;
                }
            }
            else{
                startDuckSpin = System.currentTimeMillis();
                duckSpinPower = 0.25;
            }
        }
        else{
            drive.servos.get(7).setPosition(0.97);
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
            }
            lastArmDown = System.currentTimeMillis();
        }
        lastIn = in;
        lastOut = out;
        if (armIn){
            if (gamepad2.dpad_left){
                armInPosRight += 0.001;
                armInPosLeft -= 0.001;
            }
            else if (gamepad2.dpad_right){
                armInPosRight -= 0.001;
                armInPosLeft += 0.001;
            }
            armInPosRight = Math.max(Math.min(armInPosRight,1.0),0);
            drive.servos.get(5).setPosition(armInPosRight);
            armInPosLeft = Math.max(Math.min(armInPosLeft,1.0),0);
            drive.servos.get(6).setPosition(armInPosLeft);
            speedSlowMultiplier = 1;
        }
        else {
            speedSlowMultiplier = 0.5;
            if (first){
                if (gamepad2.dpad_left){
                    armOutGrabPosRight += 0.001;
                    armOutGrabPosLeft -= 0.001;
                }
                else if (gamepad2.dpad_right){
                    armOutGrabPosRight -= 0.001;
                    armOutGrabPosLeft += 0.001;
                }
                armOutGrabPosRight = Math.max(Math.min(armOutGrabPosRight,1.0),0);
                drive.servos.get(5).setPosition(armOutGrabPosRight);
                armOutGrabPosLeft = Math.max(Math.min(armOutGrabPosLeft,1.0),0);
                drive.servos.get(6).setPosition(armOutGrabPosLeft);
            }
            else {
                if (gamepad2.dpad_left){
                    armOutPosRight += 0.001;
                    armOutPosLeft -= 0.001;
                }
                else if (gamepad2.dpad_right){
                    armOutPosRight -= 0.001;
                    armOutPosLeft += 0.001;
                }
                armOutPosRight = Math.max(Math.min(armOutPosRight,1.0),0);
                drive.servos.get(5).setPosition(armOutPosRight);
                armOutPosLeft = Math.max(Math.min(armOutPosLeft,1.0),0);
                drive.servos.get(6).setPosition(armOutPosLeft);
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
                    if (hub == 0) {
                        driveToPoint(new Pose2d(endPoint.getX(), sharedHubEndpoint.getY() + 0.5 * side, endPoint.getHeading()),new Pose2d(endPoint.getX(), sharedHubEndpoint.getY() + 2.5 * side, endPoint.getHeading()),1000,true);
                        driveToPoint(new Pose2d(endPoint.getX(), sharedHubEndpoint.getY() + 22.5 * side, endPoint.getHeading()),new Pose2d(48, 72 * side, endPoint.getHeading()),1000,true);
                    }
                }
            }
            else { //This is for going toward deposit area
                drive.startDeposit(endPoint, hubLocation, height, radius);
                if (hub == 1) {
                    driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 28.5,allianceHubEndpoint.getY(),endPoint.getHeading()), new Pose2d(allianceHubEndpoint.getX() + 25.5,allianceHubEndpoint.getY(),endPoint.getHeading()),1000,false);
                    driveToPoint(new Pose2d(allianceHubEndpoint.getX() + 4,allianceHubEndpoint.getY(),allianceHubEndpoint.getHeading()), allianceHubEndpoint,1000,false);
                    waitForDeposit(allianceHubEndpoint);
                }
                if (hub == 0) {
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
        }
    }
    public void waitForDeposit(Pose2d target){
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        boolean stop = (
                gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y ||
                Math.abs(gamepad1.left_stick_x) >= 0.5 || Math.abs(gamepad1.left_stick_y) >= 0.5 || Math.abs(gamepad1.right_stick_x) >= 0.5 || Math.abs(gamepad1.right_stick_y) >= 0.5 ||
                gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.right_trigger >= 0.5 || gamepad1.left_trigger >= 0.5 ||
                gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left ||
                gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y
        );
        while (flag == 1 && !stop && opModeIsActive()) {
            stop = (
                    gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y ||
                    Math.abs(gamepad1.left_stick_x) >= 0.5 || Math.abs(gamepad1.left_stick_y) >= 0.5 || Math.abs(gamepad1.right_stick_x) >= 0.5 || Math.abs(gamepad1.right_stick_y) >= 0.5 ||
                    gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.right_trigger >= 0.5 || gamepad1.left_trigger >= 0.5 ||
                    gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left
            );
            if(Math.abs(gamepad2.left_stick_x) > 0.15) { // Updates the turret & forces it to not have too high of a target angle that it would be impossible to reach
                drive.turretOffset -= Math.toRadians(gamepad2.left_stick_x) * 0.25; // 0.15
                if (drive.targetTurretHeading + drive.turretOffset > 1.1274009793517894){
                    drive.turretOffset = Math.abs(1.1274009793517894 - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
                else if (drive.targetTurretHeading + drive.turretOffset < -1.0703392733416528){
                    drive.turretOffset = Math.abs(-1.0703392733416528 - drive.targetTurretHeading) * Math.signum(drive.turretOffset);
                }
            }
            if (gamepad2.right_trigger >= 0.5){//Makes it deposit
                firstUpdate = true;
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

            if(Math.abs(gamepad2.right_stick_y) > 0.25) { // Updates the slide length
                drive.slidesOffset -= gamepad2.right_stick_y * 0.2;
                double maxPossibleSlideExtension = 52;//Todo: fix value
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

            if (gamepad2.a){
                drive.v4barOffset = Math.toRadians(-10);
                drive.slidesOffset = 0;
                drive.turretOffset = 0;
            }
            drive.update();
            Pose2d error = drive.getRelError(target);
            double dist = Math.pow(error.getX()*error.getX() + error.getX()*error.getX(),0.5);
            if (dist > 1) {
                drive.updateMotors(error, 0.25, 0.25,4, Math.toRadians(8), 1, 0);
            }
            else {
                drive.setMotorPowers(0,0,0,0);
            }
            if (drive.slidesCase == 5 && drive.lastSlidesCase == 4){ // Just finished the deposit
                flag = 0;
                done = false;
            }
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
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
                firstAlliance = true;
                endPoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
                hubLocation = new Pose2d(48, 0);
                if (firstUpdate) {
                    drive.currentIntake = -side;
                }
                if (firstShared) {
                    Log.e("Activate", "Shared");
                    if (side == -1) {
                        drive.turretOffset = Math.toRadians(64.5953 - 3);
                    } else {
                        drive.turretOffset = -Math.toRadians(61.3259 - 3);
                    }
                    drive.slidesOffset = 0;
                    drive.v4barOffset = 0;
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
                    endPoint = new Pose2d(-56, 60 * side, Math.toRadians(180));
                }
                radius = 3;
                height = 14;
                hubLocation = new Pose2d(-12.0, 24.0*side);

                intake = side == -1;
                if (firstUpdate) {
                    drive.currentIntake = side;
                }
                if (firstAlliance) {
                    Log.e("Activate", "Alliance");
                    drive.turretOffset = 0;
                    drive.slidesOffset = 0;
                    drive.v4barOffset = 0;
                }
                firstAlliance = false;
                break;
        }
    }
    public void driveToPoint(Pose2d target, long maxTime, boolean shared){
        driveToPoint(target,target,maxTime,shared);
    }
    public void driveToPoint(Pose2d target, Pose2d target2, long maxTime, boolean shared){
        double error = 2;//4
        double power = 0.95;//0.8
        double slowDownDist = 2;
        boolean hugWall = true;
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/1.6,kStatic * 1.5);
        double slowTurnAngle = Math.toRadians(7);//5
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
        while (auto.getToggleState() && opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(3)) && System.currentTimeMillis() - start < maxTime){
            auto.update(gamepad1.a);
            if (gamepad1.b){ //Stops everything and makes turret face forward
                drive.slidesCase = 0;
                drive.intakeCase = 0;
                drive.currentIntake = 0;
                drive.transferMineral = false;
                auto.toggleState = false;
                firstUpdate = false;
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
                if (Math.abs(relError.getY()) < sideError) {
                    relError = new Pose2d(relError.getX(), 0, relError.getHeading());
                    sideKStatic =  0.2 * side * m;
                }
                else if (Math.abs(relError.getX()) < error){
                    relError = new Pose2d(0, relError.getY(), relError.getHeading());
                }
            }
            drive.updateMotors(relError,power,maxPowerTurn,slowDownDist,slowTurnAngle,sideError,sideKStatic);

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
}
