package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

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
public class WarehouseAutoRed extends LinearOpMode {
    SampleMecanumDrive drive;
    double side = -1;

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

    double lastIntakeX = 39; //42

    long start = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.resetAssemblies();
        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.125 * side,0);

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

        setUp(startingPose);

        Logger a = new Logger("Alliance",false);
        String l = "red";
        a.addData(l);
        a.update();
        a.close();

        // The INIT-loop: This REPLACES waitForStart()!
        boolean seenMarker = false;
        while (!isStarted() && !isStopRequested()) {
            //Detecting AprilTags
            tagOfInterest = null;
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    previousTag = tag;
                    previousTagCounter = 0;
                    seenMarker = true;
                    break;
                }
            }
            if (previousTagCounter > 25) {  //If a tag was detected within the last 25 frames, count it
                previousTag = null;
            }
            previousTagCounter++;

            if (previousTag != null){
                if (previousTag.pose.x  > 0) {
                    telemetry.addLine("Top Level \n");
                    capNum = 2;
                }
                else {
                    telemetry.addLine("Center Level \n");
                    capNum = 1;
                }
            }
            else {
                if (seenMarker) {
                    telemetry.addLine("Low Level \n");
                    capNum = 0;
                }
                else {
                    telemetry.addLine("Never Saw Capstone . . . High Level \n");
                    capNum = 2;
                }
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
            drive.update();
        }

        setUp(startingPose);

        start = System.currentTimeMillis();
        drive.servos.get(5).setPosition(0);
        drive.servos.get(6).setPosition(0.89);
        drive.v4barOffset = Math.toRadians(-4);
        depositFirst(capNum, endPoint);
        int numMinerals = 0;
        drive.intakeLiftDelay = 0;
        while (System.currentTimeMillis() - start <= 30000 - 3270 && opModeIsActive()){
            long lastCycleStart = System.currentTimeMillis();
            driveIn(endPoint,numMinerals);
            if (System.currentTimeMillis() - start >= 30000 - 2270){
                break;
            }
            driveOut(endPoint,numMinerals);
            while (System.currentTimeMillis() - lastCycleStart < 1000){
                drive.update();
            }
            numMinerals ++;
        }
        driveToPoint(new Pose2d(45, endPoint.getY(), 0), false, 1, 1, 1000, 13, true);
        while (opModeIsActive() && System.currentTimeMillis() - start <= 29900) {
            driveToPoint(new Pose2d(45, endPoint.getY(), 0), false, 1, 0.3, 1000, 3, true);
        }
        drive.setMotorPowers( 0 , 0, 0, 0);
        drive.slides.setPower(0);
        drive.slides2.setPower(0);
        drive.turret.setPower(0);

        Logger b = new Logger("Pose",false);
        b.addData(drive.currentPose.getX());
        b.addData(drive.currentPose.getY());
        b.addData(drive.currentPose.getHeading());
        b.update();
        b.close();
    }
    public void driveIn(Pose2d endPoint, int numMinerals){
        //drive.startIntake(side == -1);
        int a = 3;
        double angle = ((numMinerals % a) * Math.toRadians(-15)) * Math.signum(endPoint.getY());
        double x = lastIntakeX;
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        driveToPoint(new Pose2d(16.5, endPoint.getY(),0), new Pose2d(38.5, endPoint.getY(),0), false,3, 0.65,500,6, true);
        drive.startIntake(side == -1);
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), new Pose2d(x,y,0), false,2, 0.75,500,8, true);
        driveToPoint(new Pose2d(38.5, endPoint.getY(),0), new Pose2d(x,y,0), false,2, 0.65,800,8, true);
        driveToPoint(new Pose2d(Math.max(x - 6,39), y, 0), new Pose2d(72, 24 * side, angle), true, 2, 0.45, 600, 12, false);
        driveToPoint(new Pose2d(x,y,angle), new Pose2d(72,24 * side,angle), true,3, 0.30,2000,8,false);
        intakeMineral(0.35,4000);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
        lastIntakeX = Math.max(drive.currentPose.getX(),lastIntakeX);
        lastIntakeX = Math.min(54.0, lastIntakeX);
    }
    public void driveOut(Pose2d endPoint, int numMinerals){
        double i = 0;
        double offset = -3.5;
        drive.v4barOffset = Math.toRadians(-4); drive.slidesOffset = 0; drive.turretOffset = 0;
        /*
        if (numMinerals >= 2){
            drive.slidesOffset = 3; //drive.turretOffset = Math.toRadians(-2.3 - (int)(numMinerals/2) * 1.4) * side;
            offset = Math.min((numMinerals-4)/2.0 * 2.0 - 1,1.75);
        }
         */
        Pose2d newEnd = new Pose2d(endPoint.getX() + offset, endPoint.getY(), endPoint.getHeading());
        drive.effectiveDepositAngle = Math.toRadians(-30);
        drive.startDeposit(endPoint, new Pose2d(-12.0 + i, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(new Pose2d(39.5, newEnd.getY(), 0), new Pose2d(38.5, newEnd.getY(), 0), false,4, 0.8,500,7,true);
        driveToPoint(new Pose2d(newEnd.getX() + 3,newEnd.getY() + 0.1 * side, 0), false,4, 0.5,1000,12, true);
        driveToPoint(newEnd, false,2, 0.35,1000,8, false);
        waitForDeposit(newEnd);
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        double offset = 0;
        switch (capNum) {
            case 0: r = 9.25; h = 2.5; drive.slidesOffset = -3.75; offset = -3; break; //-1.5
            case 1: r = 7; h = 7.125; break;
            case 2: r = 3; h = 13.5; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        waitForDepositSlow(offset);
        drive.slidesOffset = 0;
    }
    public void waitForDepositSlow(double offset){
        while (drive.slidesCase < 3 && opModeIsActive()){
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.slidesOffset = offset;
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
        long a = System.currentTimeMillis();
        long b = System.currentTimeMillis();
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            Pose2d error = drive.getRelError(target);
            double dist = error.getX();
            if (dist > 1 && System.currentTimeMillis() - a <= 500) {
                if (Math.abs(error.getY()) <= 0.5){
                    error = new Pose2d(error.getX(), 0, 0);
                }
                drive.updateMotors(error, 0.35, 0.35,4, Math.toRadians(3), 1, 0.5, 0);
                b = System.currentTimeMillis();
            }
            else {
                //if (System.currentTimeMillis() - b >= 150) {
                    drive.deposit();
                    drive.setMotorPowers(0, 0, 0, 0);
                //}
            }
        }
        //if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
        //if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall){
        double maxPowerTurn = 0.45;
        double slowTurnAngle = Math.toRadians(15);
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.5;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (opModeIsActive() && System.currentTimeMillis() - start <= 29900 && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(3)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double sideKStatic = 0;
            if (hugWall){
                sideKStatic = 0.3 * side;
            }
            if (x || y){
                if (Math.abs(relError.getY()) < sideError && hugWall) {
                    sideKStatic = 0.2 * side;
                    double heading = relError.getHeading();
                    if (Math.abs(relError.getY()) < sideError / 2.0){
                        sideKStatic = 0;
                        heading = 0;
                    }
                    relError = new Pose2d(relError.getX(), 0, heading);
                }
                else if (Math.abs(relError.getX()) < error){
                    relError = new Pose2d(0, relError.getY(), relError.getHeading());
                }
            }
            drive.updateMotors(relError,power,maxPowerTurn,slowDownDist,slowTurnAngle,error,sideError,sideKStatic);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        //drive.setMotorPowers(0,0,0,0);
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall){
        driveToPoint(target,target,intake,error,power,maxTime,slowDownDist,hugWall);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        double maxPower = power;
        Pose2d lastPose = drive.currentPose;
        Long lastGoodIntake = System.currentTimeMillis();
        boolean first = true;
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double currentPower = maxPower;
            drive.intake.setPower(-1);
            double sidePower = 0;
            if (Math.abs(drive.relCurrentVelocity.getX()) <= 3){
                //currentPower = maxPower/2.0;
                if (first && System.currentTimeMillis() - lastGoodIntake >= 100) {
                    first = false;
                    driveToPoint(new Pose2d(lastPose.getX() - 4, lastPose.getY(), lastPose.getHeading()), true, 1, 0.5, 1000, 2, false);
                }
            }
            else{
                lastGoodIntake = System.currentTimeMillis();
                lastPose = drive.currentPose;
            }
            double turn = 0;
            double multiplier = Math.min(1.0/(Math.abs(currentPower) + Math.abs(turn) + Math.abs(sidePower)),1);
            drive.pinMotorPowers((currentPower+turn-sidePower)*multiplier,(currentPower+turn+sidePower)*multiplier,(currentPower-turn-sidePower)*multiplier,(currentPower-turn+sidePower)*multiplier);
            drive.update();
        }
        drive.intake.setPower(-1);
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