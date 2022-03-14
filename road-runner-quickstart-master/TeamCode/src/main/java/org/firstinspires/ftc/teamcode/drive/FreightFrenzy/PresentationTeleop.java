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
public class PresentationTeleop extends LinearOpMode {
    SampleMecanumDrive drive;

    int hub = 1;

    boolean firstUpdate = false;

    Pose2d endPoint = new Pose2d();
    Pose2d hubLocation = new Pose2d();
    double height = 0;
    double radius = 0;

    double side = 1;
    boolean intake = false;

    boolean lastToggleHub = false;

    boolean lastIntake = false;

    long start;

    boolean firstShared = false;
    boolean firstAlliance = false;

    Pose2d sharedHubEndpoint = new Pose2d(65.25, 16 * side, Math.toRadians(90) * side);
    Pose2d allianceHubEndpoint = new Pose2d(12, 65.25 * side, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            drive.update();

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
                drive.intakeCase = 6;
            }

            drive.startDeposit(endPoint, hubLocation, height, radius);
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
            if (gamepad1.right_bumper){
                drive.deposit();
            }

            updatePoseLock();
            updateHub();

            if (gamepad1.left_bumper){ //1.2
                drive.v4barOffset -= Math.toRadians(2);
            }
            if (gamepad1.left_trigger >= 0.5){
                drive.v4barOffset += Math.toRadians(2);
            }

            if (gamepad1.a){
                drive.v4barOffset = Math.toRadians(-10);
                drive.slidesOffset = 0;
                drive.turretOffset = 0;
            }

            telemetry.addData("hub", hub);
            telemetry.update();
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
    public void updateHub(){
        boolean toggleHub = gamepad1.y;
        if (toggleHub && !lastToggleHub){
            hub = (hub + 1) % 2;
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
                        drive.turretOffset = Math.toRadians(3);
                    } else {
                        drive.turretOffset = -Math.toRadians(3);
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
}
