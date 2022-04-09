package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Logger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class TestAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    double side = -1;

    double lastIntakeX = 36; //39

    long start = System.currentTimeMillis();
    long cutoff = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.125 * side,0);

        int capNum = 2;

        drive.currentIntake = side;
        drive.transferMineral = true;
        drive.setV4barDeposit(drive.depositTransferAngle,Math.toRadians(-5));
        drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);
        drive.updateSlideLength = false;

        while (!isStarted() && !isStopRequested()){
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
        drive.intakeLiftDelay = 0; //50
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
        driveToPoint(new Pose2d(40, endPoint.getY(), 0),new Pose2d(72, 24, 0), false, 1, 0.85, 1000, 13, true,100);
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
        drive.startIntake(side == -1);
        double angle = Math.toRadians(-15) * Math.signum(endPoint.getY()) * (numMinerals % 3);//-12
        double x =  -2 + lastIntakeX - 12 * (1 - Math.cos(angle)); // 3
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        driveToPoint(
                new Pose2d(x - 6, endPoint.getY(), 0), //8
                new Pose2d(72, 24 * side, angle),
                true, 3, 0.95, 900, 5, true, cutoff,true //6
        );
        //0.35, 7
        if (angle != 0) {
            driveToPoint(
                    new Pose2d(x, y, angle),
                    new Pose2d(72, 24 * side, angle),
                    true, 1.5, 0.35, 800, 1.5, false, cutoff
            );
        }
        else {
            int k = 4; //5
            if (numMinerals == 0){
                k = 2;
            }
            intakeMineral(0.5,k * 150, false); //500
        }
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 150 && drive.intakeCase == 2){
            drive.update();
            drive.setMotorPowers(0,0,0,0);
        }
        intakeMineral(0.405,2000, true);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
        lastIntakeX = Math.max(drive.currentPose.getX(),lastIntakeX + 2);
        lastIntakeX = Math.min(54.0, lastIntakeX);
    }
    public void driveOut(Pose2d endPoint){
        double i = -1;
        double offset = 0; //-2
        drive.v4barOffset = Math.toRadians(0); drive.slidesOffset = 2; drive.turretOffset = 0; // -4
        Pose2d newEnd = new Pose2d(endPoint.getX() + offset, endPoint.getY(), endPoint.getHeading());
        drive.startDeposit(endPoint, new Pose2d(-12.0 + i, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        driveToPoint(
                new Pose2d(38.5, newEnd.getY() + side * 0.2, 0),
                new Pose2d(33.5, newEnd.getY() + side * 0.2, 0),
                false,3, 0.85,500,4,true, cutoff //0.75
        );
        drive.deposit();
        driveToPoint(
                new Pose2d(newEnd.getX() + 5,newEnd.getY() + 0.2 * side, 0),
                false,4, 0.85,1000,10, true, cutoff //0.95, 10
        );
        //long start = System.currentTimeMillis(); System.currentTimeMillis() - start < 1000
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.update();
            Pose2d error = drive.getRelError(endPoint);
            drive.deposit();
            if (Math.abs(error.getY()) <= 0.5){
                error = new Pose2d(error.getX(), 0, 0);
            }
            drive.updateMotors(error, 0.30, 0.45,14, Math.toRadians(8), 0.25, 0.5, 0.2 * side); //13 //0.2 -> 0.1
        }
        drive.targetPose = null;
        drive.targetRadius = 1;
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall, long cutoff) {
        driveToPoint(target,target2,intake,error,power,maxTime,slowDownDist,hugWall,cutoff, false);
    }
    public void driveToPoint(Pose2d target, Pose2d target2, boolean intake, double error, double power, long maxTime, double slowDownDist, boolean hugWall, long cutoff, boolean forward){
        double maxPowerTurn = 0.65; //0.55
        double slowTurnAngle = Math.toRadians(10);//15
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
                        && (drive.intakeCase <= 2 || !intake) && System.currentTimeMillis() - start < maxTime
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
            if (hugWall && drive.currentPose.getX() <= 40){ //36
                sideKStatic = 0.4 * side;
            }
            if (x || y){
                if (Math.abs(relError.getY()) < sideError && hugWall && drive.currentPose.getX() <= 36) {
                    sideKStatic = 0.25 * side;
                    double heading = relError.getHeading();
                    if (Math.abs(relError.getY()) < sideError && Math.abs(drive.relCurrentVelocity.getY()) < 4){
                        sideKStatic = 0.05; // 0
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
    public void intakeMineral(double power, long maxTime, boolean goBackIfStall){
        long startingTime = System.currentTimeMillis();
        double maxPower = power;
        Pose2d lastPose = drive.currentPose;
        Long lastGoodIntake = System.currentTimeMillis();
        boolean first = true;
        double kI = 0;
        while(opModeIsActive() && drive.intakeCase <= 2 && System.currentTimeMillis()-startingTime <= maxTime && System.currentTimeMillis() - start <= 30000 - cutoff){
            double currentPower = maxPower;
            drive.intake.setPower(-1);
            double sidePower = 0;
            if (System.currentTimeMillis() - start >= 550 && (Math.abs(drive.relCurrentVelocity.getX()) <= 3 || drive.intakeHit) && goBackIfStall){
                if (first && System.currentTimeMillis() - lastGoodIntake >= 300) {
                    first = false;
                    maxTime += 500;
                    driveToPoint(new Pose2d(lastPose.getX() - 4, lastPose.getY(), lastPose.getHeading()), true, 1, 0.35, 1000, 6, false, cutoff);
                }
            }
            else{
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
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        double offset = 0;
        switch (capNum) {
            case 0: r = 9.25; h = 2.5; drive.slidesOffset = -3.75; offset = -2.5; break; //-1.5
            case 1: r = 7; h = 7.125; offset = -1.5; break;
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