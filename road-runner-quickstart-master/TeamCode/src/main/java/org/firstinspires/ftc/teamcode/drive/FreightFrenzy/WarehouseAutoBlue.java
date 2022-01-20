package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Logger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class WarehouseAutoBlue extends LinearOpMode {
    SampleMecanumDrive drive;
    double side = 1;

    /* START CAMERA PARAMETERS */
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics (UNITS ARE PIXELS). Calibration for C270 webcam at 640x480
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.0762;    //0.166

    int ID_TAG_OF_INTEREST = 17; // Tag ID 17 from the 36h11 family
    AprilTagDetection tagOfInterest = null;
    AprilTagDetection previousTag = null;
    int previousTagCounter = 0;
    /* END CAMERA PARAMETERS */

    double lastIntakeX = 42;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.resetAssemblies();
        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.25 * side,0);

        int capNum = 2;

        drive.currentIntake = side;
        drive.transferMineral = true;
        drive.setV4barDeposit(drive.depositTransferAngle,Math.toRadians(-5));
        drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);

        /* START CAMERA INITIALIZATION */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleep(1000);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) {

            }
        });
        /* END CAMERA INITIALIZATION */

        // The INIT-loop: This REPLACES waitForStart()!
        while (!isStarted() && !isStopRequested()) {
            //Detecting AprilTags
            tagOfInterest = null;
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    previousTag = tag;
                    previousTagCounter = 0;
                    break;
                }
            }
            if (previousTagCounter > 25) {  //If a tag was detected within the last 25 frames, count it
                previousTag = null;
            }
            previousTagCounter++;

            if (previousTag != null){
                if (previousTag.pose.x  > 0) {
                    telemetry.addLine("Center Level \n");
                    capNum = 1;
                }
                else {
                    telemetry.addLine("Low Level \n");
                    capNum = 0;
                }
            }
            else {
                telemetry.addLine("Top Level \n");
                capNum = 2;
            }

            if (tagOfInterest != null) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else if (previousTag != null) {
                telemetry.addLine("Tag of interest sighted in the last few frames.\n\nLocation data:");
                tagToTelemetry(previousTag);
            }
            else {
                telemetry.addLine("No Tag.");
            }
            telemetry.update();
        }

        setUp(startingPose);

        Logger a = new Logger("Alliance",false);
        String b = "blue";
        a.addData(b);
        a.update();
        a.close();

        long start = System.currentTimeMillis();
        drive.slidesOffset = 0;
        depositFirst(capNum, endPoint);
        int numMinerals = 0;
        while (System.currentTimeMillis() - start <= 30000 - 3270 && opModeIsActive()){
            long lastCycleStart = System.currentTimeMillis();
            driveIn(endPoint,numMinerals);
            driveOut(endPoint,numMinerals);
            while (System.currentTimeMillis() - lastCycleStart < 1000){
                drive.update();
            }
            numMinerals ++;
        }
        drive.slidesOffset = 0;
        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 1, 1000, 3, true);
        drive.setMotorPowers( 0 , 0, 0, 0);
        drive.slides.setPower(0);
        drive.slides2.setPower(0);
    }
    public void driveIn(Pose2d endPoint, int numMinerals){
        Pose2d intakePoint = new Pose2d(lastIntakeX,endPoint.getY(),0);
        driveToPoint(new Pose2d(16.5, endPoint.getY(),0), new Pose2d(38.5, endPoint.getY(),0), false,3, 0.9,500,0.5, true);
        driveToPoint(new Pose2d(38.5, endPoint.getY(),0),intakePoint, false,1, 0.8,300,1, true);
        driveToPoint(intakePoint, true,1, 0.75,500,3, false);
        intakeMineral(0.5,2000);
        if (drive.intakeCase == 2){
            Log.e("intake", "never found anything");
            drive.intakeCase ++;
        }
        lastIntakeX = Math.max(drive.currentPose.getX(),lastIntakeX);
    }
    /* this is old code if we want to revert to it
    public void driveIn(Pose2d endPoint, int numMinerals){
        drive.startIntake(side == -1);
        int a = 3;
        //TODO: Values here changed
        int b = (numMinerals/(a-1));
        double angle = ((numMinerals % a) * Math.toRadians(-10)) * Math.signum(endPoint.getY());
        double x = 45 + b * 4;
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * Math.signum(endPoint.getY());
        driveToPoint(new Pose2d(16.5, endPoint.getY(),0), new Pose2d(38.5, endPoint.getY(),0), false,3, 0.9,500,0.5, true);
        driveToPoint(new Pose2d(38.5, endPoint.getY(),0), new Pose2d(x,y,angle), false,1, 0.8,300,1, true);
        driveToPoint(new Pose2d(x,y,angle), new Pose2d(72,24 * Math.signum(endPoint.getY()),angle + Math.signum(endPoint.getY()) * Math.toRadians(5)), true,1, 0.75,500,3, false); //0.65
        for (int i = 0; i < 10; i ++){
            double power = 0.65 - i * 0.05;
            intakeMineral(power,40);
        }
        intakeMineral(0.35,500);
        intakeMineralTurn(0.35,500);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
    }
     */
    public void driveOut(Pose2d endPoint, int numMinerals){
        Pose2d newEnd = new Pose2d(endPoint.getX(), endPoint.getY(), endPoint.getHeading());
        double i = 0;
        switch (numMinerals){
            case 2: case 3:
                newEnd = new Pose2d(endPoint.getX() - 1.5, endPoint.getY(), endPoint.getHeading());
                i = 0.5;
                drive.slidesOffset = 4.5;
                break;
            case 4: case 5: case 6: case 7:
                newEnd = new Pose2d(endPoint.getX() + 1, endPoint.getY(), endPoint.getHeading());
                i = -2;
                drive.slidesOffset = 5.75;
                break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0 + i, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(new Pose2d(38.5, newEnd.getY(),0), new Pose2d(36.5, newEnd.getY(),0), false,1, 0.8,1000,1,true);
        drive.deposit();
        driveToPoint(newEnd, false,2, 0.65,1000,3, true);
        waitForDeposit(newEnd);
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        switch (capNum) {
            case 0: r = 9.25; h = 2.5; drive.slidesOffset = -1.5; break;
            case 1: r = 7; h = 8.125; break;
            case 2: r = 3; h = 13.5; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        if (capNum == 2) {
            waitForDeposit();
        }
        else {
            waitForDepositSlow();
        }
        drive.slidesOffset = 0;
    }
    public void waitForDepositSlow(){
        while (drive.slidesCase < 3 && opModeIsActive()){
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.deposit();
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }

    }
    public void waitForDeposit(){
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.deposit();
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }
    }
    public void waitForDeposit(Pose2d target){
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        while (drive.slidesCase > 4 && opModeIsActive()) {
            drive.update();
            drive.startDeposit(target, new Pose2d(-12.0, 24.0 * Math.signum(target.getY())),13.5,3);
        }
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.deposit();
            drive.update();
            Pose2d error = drive.getRelError(target);
            double dist = Math.pow(error.getX()*error.getX() + error.getX()*error.getX(),0.5);
            if (dist > 1) {
                drive.updateMotors(error, 0.25, 0.25,4, Math.toRadians(8), 1, 0);
            }
            else {
                drive.setMotorPowers(0,0,0,0);
            }
            if (drive.intakeCase == 9){
                if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
                if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
            }
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall){
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/2.0,kStatic * 1.5);
        double slowTurnAngle = Math.toRadians(16);
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.3;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double sideKStatic = 0;
            if (hugWall){
                sideKStatic = 0.3 * side;
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
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall){
        driveToPoint(target,target,intake,error,power,maxTime,slowDownDist,hugWall);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        double maxPower = power;
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double currentPower = maxPower;
            if (drive.intakeHit){
                currentPower *= 0.3;
                currentPower = Math.min(currentPower,0.25);
            }
            double turn = 0;
            double multiplier = Math.min(1.0/(Math.abs(currentPower) + Math.abs(turn)),1);
            drive.pinMotorPowers((currentPower+turn)*multiplier,(currentPower+turn)*multiplier,(currentPower-turn)*multiplier,(currentPower-turn)*multiplier);
            drive.update();
        }
        drive.setMotorPowers(0,0,0,0);
    }
    public void intakeMineralTurn(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double turn = 0.4 * side;
            if (Math.abs(Math.toRadians(drive.currentPose.getHeading())) >= 40){
                turn = -0.4 * side;
            }
            double multiplier = Math.min(1.0/(Math.abs(power) + Math.abs(turn)),1);
            drive.pinMotorPowers((power+turn)*multiplier,(power+turn)*multiplier,(power-turn)*multiplier,(power-turn)*multiplier);
            drive.update();
        }
        drive.setMotorPowers(0,0,0,0);
    }
    public void setUp(Pose2d startingPose){
        drive.update();
        drive.localizer.setPoseEstimate(startingPose);
        drive.update();
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meters", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meters", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meters", detection.pose.z));
    }
}