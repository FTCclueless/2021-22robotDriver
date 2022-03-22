package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Logger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
public class WarehouseAutoRedFaster extends LinearOpMode {
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
    long cutoff = 3000;

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
        drive.updateSlideLength = false;

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
        drive.servos.get(6).setPosition(1.0);
        drive.v4barOffset = Math.toRadians(-4);
        drive.updateSlideLength = true;
        depositFirst(capNum, endPoint);
        int numMinerals = 0;
        drive.intakeLiftDelay = 50;
        while (System.currentTimeMillis() - start <= 30000 - cutoff - 500 && opModeIsActive()){
            driveIn(endPoint,numMinerals);
            if (System.currentTimeMillis() - start >= 30000 - cutoff){
                break;
            }
            driveOut(endPoint);
            numMinerals ++;
        }
        if (drive.intakeCase <= 2) {
            drive.intakeCase = 0;
        }

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveToPoint(new Pose2d(36, endPoint.getY(), 0), false, 1, 0.75, 1000, 13, true,100);
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
        int a = 3;
        double angle = ((numMinerals % a) * Math.toRadians(-17.5)) * Math.signum(endPoint.getY());
        double x = lastIntakeX;
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        drive.startIntake(side == -1);
        driveToPoint(new Pose2d(Math.max(x - 8,30), endPoint.getY(), 0), new Pose2d(72, 24 * side, angle), true, 4, 0.95, 600, 10, false,cutoff);
        driveToPoint(new Pose2d(x + 1 + 12 * (1 - Math.cos(angle)),y,angle), new Pose2d(72,24 * side,angle), true,2.5, 0.45,300,3.5,false,cutoff);
        intakeMineral(0.35,2000);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
        lastIntakeX = Math.max(drive.currentPose.getX(),lastIntakeX);
        lastIntakeX = Math.min(50.0, lastIntakeX);
    }
    public void driveOut(Pose2d endPoint){
        double i = 0;
        double offset = 2;
        drive.v4barOffset = Math.toRadians(-4); drive.slidesOffset = 2; drive.turretOffset = 0;
        Pose2d newEnd = new Pose2d(endPoint.getX() + offset, endPoint.getY(), endPoint.getHeading());
        drive.effectiveDepositAngle = Math.toRadians(-30);
        drive.startDeposit(endPoint, new Pose2d(-12.0 + i, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(new Pose2d(40.5, newEnd.getY(), 0), new Pose2d(33.5, newEnd.getY() - side, 0), false,4, 0.90,500,4,true,cutoff);
        driveToPoint(new Pose2d(newEnd.getX() + 5,newEnd.getY() + 0.1 * side, 0), false,4, 0.95,1000,10, true,cutoff);
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
    public void waitForDeposit(Pose2d target){
        drive.targetPose = target;
        drive.targetRadius = 0.25;
        while (drive.slidesCase > 4 && opModeIsActive()) {
            drive.update();
            drive.startDeposit(target, new Pose2d(-12.0, 24.0 * Math.signum(target.getY())),13.5,3);
        }
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            Pose2d error = drive.getRelError(target);
            drive.deposit();
            if (Math.abs(error.getY()) <= 0.5){
                error = new Pose2d(error.getX(), 0, 0);
            }
            drive.updateMotors(error, 0.45, 0.25,5, Math.toRadians(8), 0.25, 0.5, 0);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall, long cutoff){
        double maxPowerTurn = 0.35;
        double slowTurnAngle = Math.toRadians(15);
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.4;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        double kI = 0;
        while (opModeIsActive() && System.currentTimeMillis() - start <= 30000 - cutoff && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(error * 2)) && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime){
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double targetSpeed = power * 30 * Math.pow(Math.min(Math.abs(relError.getX()/slowDownDist),1),2);
            double currentSpeed = Math.abs(drive.relCurrentVelocity.getX());
            double speedError = targetSpeed-currentSpeed;
            double kP = speedError * 0.02;
            kI += speedError * drive.loopSpeed * 0.0005;
            double sideKStatic = 0;
            if (hugWall){
                sideKStatic = 0.4 * side;
            }
            if (x || y){
                if (Math.abs(relError.getY()) < sideError && hugWall) {
                    sideKStatic = 0.25 * side;
                    double heading = relError.getHeading();
                    if (Math.abs(relError.getY()) < sideError / 2.0 && Math.abs(drive.relCurrentVelocity.getY()) < 4){
                        sideKStatic = 0;
                        heading = 0;
                    }
                    relError = new Pose2d(relError.getX(), 0, heading);
                }
            }
            drive.updateMotors(relError,power*0.75 + kP + kI,maxPowerTurn,slowDownDist,slowTurnAngle,0.5,sideError,sideKStatic);
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall,long cutoff){
        driveToPoint(target,target,intake,error,power,maxTime,slowDownDist,hugWall,cutoff);
    }
    public void intakeMineral(double power, long maxTime){
        long startingTime = System.currentTimeMillis();
        double maxPower = power;
        Pose2d lastPose = drive.currentPose;
        Long lastGoodIntake = System.currentTimeMillis();
        boolean first = true;
        double kI = 0;
        while(drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && opModeIsActive() && System.currentTimeMillis() - start <= 30000 - cutoff){
            double currentPower = maxPower;
            drive.intake.setPower(-1);
            double sidePower = 0;
            if (Math.abs(drive.relCurrentVelocity.getX()) <= 3 || drive.intakeHit){
                if (first && System.currentTimeMillis() - lastGoodIntake >= 500) {
                    //drive.intake.setPower(0);
                    first = false;
                    //driveToPoint(new Pose2d(lastPose.getX() - 4, lastPose.getY(), lastPose.getHeading()), true, 1, 0.35, 1000, 6, false, cutoff);
                }
            }
            else{
                //drive.intake.setPower(drive.intakePower);
                lastGoodIntake = System.currentTimeMillis();
                lastPose = drive.currentPose;
            }
            double turn = 0;
            double targetSpeed = power * 30;
            double currentSpeed = Math.abs(drive.relCurrentVelocity.getX());
            double speedError = targetSpeed-currentSpeed;
            double kP = speedError * 0.02;
            kI += speedError * drive.loopSpeed * 0.0005;
            double multiplier = Math.min(1.0/(Math.abs(currentPower) + Math.abs(turn) + Math.abs(sidePower)),1);
            double f = kP + kI + currentPower * 0.75;
            drive.pinMotorPowers((f+turn-sidePower)*multiplier,(f+turn+sidePower)*multiplier,(f-turn-sidePower)*multiplier,(f-turn+sidePower)*multiplier);
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