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
public class WarehouseAutoRed extends LinearOpMode {
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

    SampleMecanumDrive drive;
    double side = -1;

    double lastIntakeX = 36; //39

    long start = System.currentTimeMillis();
    long cutoff = 1300;

    int numMinerals = 0;

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
        drive.intakeLiftDelay = 0; //50
        while (System.currentTimeMillis() - start <= 30000 - cutoff - 500 && opModeIsActive()){
            driveIn(endPoint);
            if (System.currentTimeMillis() - start >= 30000 - cutoff){
                break;
            }
            driveOut(endPoint);
        }
        if (drive.intakeCase <= 2) {
            drive.intakeCase = 0;
        }

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveToPoint(new Pose2d(40, endPoint.getY(), 0),new Pose2d(72, 24, 0), false, 1, 0.85, 1000, 13, true,100);
        drive.setMotorPowers( 0.2, 0.2, 0.2, 0.2);
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
    public void driveIn(Pose2d endPoint){
        drive.startIntake(side == -1);
        double angle = Math.toRadians(-10) * side * (numMinerals % 3);//-12 => -15 => -13.5
        double x =  -2 + lastIntakeX + 12 * (1 - Math.cos(angle)); // -2, (-)
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        driveToPoint(
                new Pose2d(x - 6, endPoint.getY() + 0.3 * side, 0), //8
                new Pose2d(72, 24 * side, angle),
                true, 3, 0.95, 2000, 5, true, cutoff,true //6
        );
        //0.35, 7
        if (angle != 0) {
            driveToPoint(
                    new Pose2d(x, y, angle),
                    new Pose2d(72, 24 * side, angle),
                    true, 1.5, 0.405, 500, 1.5, false, cutoff //0.405, 800
            );
        }
        else {
            int k = 6; //5 => 4
            if (numMinerals == 0){
                k = 2;
            }
            intakeMineral(0.5,k * 150, false); //500
        }

        if (Math.abs(drive.currentPose.getHeading()) >= Math.toRadians(5) || angle == 0) { //make it so that if it intakes something while coming in that it then keeps going at the same angle the next cycle through
            numMinerals++;
        }

        /*
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 50 && drive.intakeCase == 2){ //250 => 150
            drive.update();
            drive.setMotorPowers(0,0,0,0);
        }
         */
        intakeMineral(0.285,2000, true); //345
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }

        lastIntakeX = Math.max(drive.currentPose.getX(),lastIntakeX + 2);
        lastIntakeX = Math.min(54.0, lastIntakeX);
    }
    public void driveOut(Pose2d endPoint){
        double i = -1;
        double offset = 1; //1.5
        drive.effectiveDepositAngle = Math.toRadians(-35);//-35
        drive.v4barOffset = Math.toRadians(-6); drive.slidesOffset = -0.5; drive.turretOffset = Math.toRadians(1 * side); // 1
        Pose2d newEnd = new Pose2d(endPoint.getX() + offset, endPoint.getY(), endPoint.getHeading());
        drive.startDeposit(endPoint, new Pose2d(-12.0 + i, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(
                new Pose2d(36.5, newEnd.getY() + side * 0.2, 0),
                new Pose2d(33.5, newEnd.getY() + side * -0.5, 0),
                false,3, 0.85,750,4,true, cutoff //0.75
        );
        //drive.deposit();
        driveToPoint(
                new Pose2d(newEnd.getX() + 5,newEnd.getY() + 0.25 * side, 0),
                false,2, 0.95,1500,10, true, cutoff //4
        );
        //long start = System.currentTimeMillis(); System.currentTimeMillis() - start < 1000
        drive.targetPose = newEnd;
        drive.targetRadius = 1;
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            Pose2d error = drive.getRelError(newEnd);
            drive.deposit();
            double a = 0.15 * side;
            if (Math.abs(error.getY()) <= 0.5){
                error = new Pose2d(error.getX(), 0, error.getHeading());
                a *= 3;
            }
            if (drive.slidesCase == 4){
                drive.startIntake(side == -1);
            }
            drive.updateMotors(error, 0.215, 0.45,10, Math.toRadians(8), 0.35, 0.5, a);//14,0.15
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall, long cutoff) {
        driveToPoint(target,target2,intake,error,power,maxTime,slowDownDist,hugWall,cutoff, false);
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall, long cutoff, boolean forward){
        double maxPowerTurn = 0.35; //0.55 => 0.65
        if (hugWall){
            maxPowerTurn = 0.65;
        }
        double slowTurnAngle = Math.toRadians(7);//15 => 10
        drive.targetPose = target;
        drive.targetRadius = error;
        long start = System.currentTimeMillis();
        double sideError = 0.4;
        drive.update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
        double kI = 0;
        while (
                opModeIsActive() && System.currentTimeMillis() - start <= 30000 - cutoff
                        && (!forward || drive.currentPose.getX() <= target.getX() - error)
                        && !(x && y &&  Math.abs(drive.currentPose.getHeading() - target.getHeading()) < Math.toRadians(error * 2))
                        && ((drive.intakeCase <= 2 || drive.intakeCase == 8) || !intake) && System.currentTimeMillis() - start < maxTime
        ) {
            drive.update();
            x = (Math.max(target.getX(),target2.getX()) + error > drive.currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < drive.currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > drive.currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < drive.currentPose.getY());
            Pose2d relError = drive.getRelError(target);
            double targetSpeed = power * 35 * (Math.pow(Math.min(Math.abs(relError.getX()/slowDownDist),1),2) * 0.5 + 0.5);
            double currentSpeed = Math.abs(drive.relCurrentVelocity.getX());
            double speedError = targetSpeed-currentSpeed;
            double kP = speedError * 0.02;
            kI += speedError * drive.loopSpeed * 0.0005;
            double sideKStatic = 0;
            if (hugWall && (drive.currentPose.getX() <= 40 || Math.min(target.getX(),target2.getX()) <= 40)){ //36
                sideKStatic = 0.5 * side;
                if (x || y){
                    sideKStatic = 0.35 * side; //0.3
                    if (Math.abs(relError.getY()) < sideError){
                        sideKStatic = 0.15 * side;
                    }
                    relError = new Pose2d(relError.getX(), 0, relError.getHeading());
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
    public void intakeMineral(double power, long maxTime, boolean goBackIfStall){
        long startingTime = System.currentTimeMillis();
        double maxPower = power;
        Pose2d lastPose = drive.currentPose;
        Long lastGoodIntake = System.currentTimeMillis();
        Long stall = System.currentTimeMillis();
        Long noStall = System.currentTimeMillis();
        Long a = System.currentTimeMillis();
        boolean b = true;
        boolean first = true;
        boolean c = true;
        double kI = 0;
        while(opModeIsActive() && drive.intakeCase <= 2 && a-startingTime <= maxTime && a - start <= 30000 - cutoff){
            double currentPower = maxPower;
            //drive.intake.setPower(-1);
            double sidePower = 0;
            a = System.currentTimeMillis();
            if (a - start >= 150 && Math.abs(drive.relCurrentVelocity.getX()) <= 2 && goBackIfStall){
                if (first && a - lastGoodIntake >= 300) {
                    first = false;
                    maxTime += 500;
                    driveToPoint(new Pose2d(lastPose.getX() - 4, lastPose.getY(), lastPose.getHeading()), true, 1, 0.35, 1000, 6, false, cutoff);
                }
            }
            else{
                lastGoodIntake = a;
                lastPose = drive.currentPose;
            }

            if (Math.abs(drive.intakeSpeed) <= 3 && a - noStall >= 200 && c){
                drive.intake.setPower(drive.transfer1Power);
                stall = a;
                c = false;
                first = false;
                maxTime += 500;
                while (System.currentTimeMillis() - a <= 100){
                    drive.update();
                }
                drive.intake.setPower(drive.intakePower);
                driveToPoint(new Pose2d(lastPose.getX() - 2, lastPose.getY() - 4 * side, lastPose.getHeading() - Math.toRadians(8 * side)), true, 1, 0.35, 1000, 6, false, cutoff);
            }

            if (a - stall >= 100 && !c){
                drive.intake.setPower(drive.intakePower);
                c = true;
                noStall = a;
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
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        double offset = 0;
        switch (capNum) {
            case 0: r = 9.25; h = 2.5; drive.slidesOffset = -3.75; offset = -1.5; drive.turretOffset = Math.toRadians(-7 * side); break;
            case 1: r = 7; h = 7.125; offset = 3.5; drive.turretOffset = Math.toRadians(-11 * side); break;
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
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meters", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meters", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meters", detection.pose.z));
    }
}