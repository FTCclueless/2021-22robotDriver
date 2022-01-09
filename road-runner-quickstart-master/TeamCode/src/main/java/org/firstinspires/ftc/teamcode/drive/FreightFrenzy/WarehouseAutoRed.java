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
    public int RANDOMIZATION = 3;  //default set to 3, which is the highest level of the shipping hub

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
    boolean tagOfInterestFound;
    AprilTagDetection tagOfInterest = null;
    /* END CAMERA PARAMETERS */

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.resetAssemblies();
        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.25 * side,0);

        int capNum = 2;
        //TODO: Implement ML here

        drive.currentIntake = side;
        drive.transferMineral = true;
        drive.setV4barDeposit(drive.depositTransferAngle,Math.toRadians(-5));
        drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);

        /* START CAMERA INITIALIZATION */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        /* END CAMERA INITIALIZATION */

        // The INIT-loop: This REPLACES waitForStart()!
        while (!isStarted() && !isStopRequested()) {
            //Detecting AprilTags
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            tagOfInterestFound = false;
            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    tagOfInterestFound = true;
                    break;
                }
            }

            if (tagOfInterestFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                tagToTelemetry(tagOfInterest);
            }
            else if (tagOfInterest != null) {
                telemetry.addLine("Tag no longer in view; previously found at:");
//                tagToTelemetry(tagOfInterest);
            }
            else {
                telemetry.addLine("No Tag.");
            }

            telemetry.update();
        }

        // BELOW STUFF HAPPENS AFTER START IS PRESSED
        /* Update the telemetry with tag data */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag detected.");
            telemetry.update();
        }
        /* Set the randomization variable. This is based on the location of the last sighting of the tag. */
        //TODO: figure out the correct thresholds (123 and 234 are fillers)
        if(tagOfInterest.pose.x <= 123) {
            RANDOMIZATION = 1;
        }
        else if(tagOfInterest.pose.x >= 123 && tagOfInterest.pose.x <= 234) {
            RANDOMIZATION = 2;
        }
        else {  //Note: the tag is out of the FoV when the randomization is 3
            RANDOMIZATION = 3;
        }

        setUp(startingPose);

        Logger a = new Logger("Alliance",false);
        a.addData("red");
        a.update();
        a.close();

        Long start = System.currentTimeMillis();
        depositFirst(capNum, endPoint);
        int numMinerals = 0;
        while (System.currentTimeMillis() - start <= 30000 - 3270 && opModeIsActive()){
            driveIn(endPoint,numMinerals);
            driveOut(endPoint,numMinerals);
            numMinerals ++;
        }
        driveToPoint(new Pose2d(45, endPoint.getY(),0), false,1, 1, 1000, 3);
        drive.setMotorPowers( 0 , 0, 0, 0);
    }
    public void driveIn(Pose2d endPoint, int numMinerals){
        drive.startIntake(side == -1);
        int a = 3;
        int b = (numMinerals/(a - 1));
        double angle = (numMinerals % a) * Math.toRadians(-20) * Math.signum(endPoint.getY());
        double x = 42 + b * 4;
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * Math.signum(endPoint.getY());
        driveToPoint(new Pose2d(18.5, endPoint.getY(),0), new Pose2d(36.5, endPoint.getY(),0), true,1, 0.8,500,1);
        driveToPoint(new Pose2d(36.5, endPoint.getY(),0), new Pose2d(x,y,angle), true,1, 0.8,500,1);
        driveToPoint(new Pose2d(x,y,angle), new Pose2d(72,24 * Math.signum(endPoint.getY()),angle), true,1, 0.5,500,3); // 0.5
        intakeMineral(0.35,1500);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
    }
    public void driveOut(Pose2d endPoint, int numMinerals){
        double i = -1;
        if (numMinerals < 3){
            i = 0;
        }
        else if (numMinerals < 6){
            i = -2;
        }
        Pose2d newEnd = new Pose2d(endPoint.getX() + i, endPoint.getY(), endPoint.getHeading());
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(new Pose2d(36.5, newEnd.getY(),0), newEnd, false,1, 0.8,1000,1);
        driveToPoint(newEnd, false,2, 0.5,1000,3);
        waitForDeposit(newEnd);
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        switch (capNum) {
            case 0: r = 8.5; h = 6; break;
            case 1: r = 7; h = 12.125; break;
            case 2: r = 3; h = 13.5; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        waitForDeposit();
    }
    public void waitForDeposit(){
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.deposit();
            drive.update();
            if (drive.intakeCase == 9){
                if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
                if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
            }
        }
    }
    public void waitForDeposit(Pose2d target){
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.deposit();
            drive.update();
            Pose2d error = drive.getRelError(target);
            double dist = Math.pow(error.getX()*error.getX() + error.getX()*error.getX(),0.5);
            if (dist > 1) {
                drive.updateMotors(error, 0.25, 0.25, 4, Math.toRadians(8), 1);
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
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist){
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/2.0,kStatic * 1.5);
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        while (opModeIsActive() && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            if (x){
                relError = new Pose2d(0,relError.getY(),relError.getHeading());
            }
            if (y){
                relError = new Pose2d(relError.getX(),0,relError.getHeading());
            }
            drive.updateMotors(relError, power, maxPowerTurn, slowDownDist, Math.toRadians(slowTurnAngle), error);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime, double slowDownDist){
        double kStatic = DriveConstants.kStatic;
        double maxPowerTurn = Math.max(power/2.0,kStatic * 1.5);
        double slowTurnAngle = 8;
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (Math.abs(drive.currentPose.getX()-target.getX()) > error || Math.abs(drive.currentPose.getY()-target.getY()) > error || Math.abs(drive.currentPose.getHeading() - target.getHeading()) > Math.toRadians(5)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            drive.updateMotors(drive.getRelError(target), power, maxPowerTurn, slowDownDist, Math.toRadians(slowTurnAngle), error);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
        drive.setMotorPowers(0,0,0,0);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive()){
            double turn = 0;
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