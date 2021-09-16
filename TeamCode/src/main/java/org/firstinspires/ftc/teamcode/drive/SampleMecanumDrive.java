package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7, 0.075,0.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(100, 10, 0);
    private ArrayList<Pose2d> poseHistory;

    private PIDFController turnController;

    private String TAG = "SampleTankDrive";

    public static double LATERAL_MULTIPLIER = 1.75;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;

    static RevBulkData bulkData;
    static ExpansionHubEx expansionHub1, expansionHub2;
    static ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;

    private BNO055IMU imu;
    private boolean useIMU;

    private VoltageSensor batteryVoltageSensor;

    Localizer localizer;

    static int[] encoders;

    public int loops = 0;

    public Pose2d currentPose;
    public Pose2d currentVelocity;

    public static int slidesEncoder;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        encoders = new int[4];
        useIMU = false;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront =     (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftRear =      (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightRear =     (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront =    (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        // add more motors here

        //expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");


//        dropperServo.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE); //rightRear
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //rightFront

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }


        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = new Localizer();
        setLocalizer(localizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        poseHistory = new ArrayList<Pose2d>();
    }

    public static void getEncoders(){
        bulkData = expansionHub1.getBulkInputData();
        encoders[0] = bulkData.getMotorCurrentPosition(leftFront); // switched 0 and 1
        encoders[1] = bulkData.getMotorCurrentPosition(rightFront);
        encoders[2] = bulkData.getMotorCurrentPosition(rightRear);
        encoders[3] = bulkData.getMotorCurrentPosition(leftRear);
        // you can set the bulkData to the other expansion hub to get data from the other one
        //bulkData = expansionHub2.getBulkInputData();
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {return trajectorySequenceRunner.getLastPoseError();}

    public void updateEstimate(){
        getEncoders();
        if (loops%5 == 0 && useIMU){
            localizer.updateHeading(imu.getAngularOrientation().firstAngle);
        }
        localizer.updateEncoders(encoders);
        localizer.update();
        currentPose = getPoseEstimate();
        currentVelocity = getPoseVelocity();
    }

    public void update() {
        loops ++;
        updateEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(currentPose, currentVelocity);
        if (signal != null) {
            updateDriveMotors(signal);
        }
    }
    public void updateDriveMotors(DriveSignal signal){
        double forward =    (signal.component1().component1() * kV) + (signal.component2().component1() * kA);
        double left =       ((signal.component1().component2() * kV) + (signal.component2().component2() * kA)) * LATERAL_MULTIPLIER;
        double turn =       (signal.component1().component3() * kV) + (signal.component2().component3() * kA);
        double p1 = forward-left-turn;
        double p2 = forward+left-turn;
        double p3 = forward-left+turn;
        double p4 = forward+left+turn;
        double maxPow = getMax(p1,p2,p3,p4);
        if(maxPow >= 1.0 - kStatic) {
            p1 /=maxPow; p1 *= 1.0 - kStatic;
            p2 /=maxPow; p2 *= 1.0 - kStatic;
            p3 /=maxPow; p3 *= 1.0 - kStatic;
            p4 /=maxPow; p4 *= 1.0 - kStatic;
        }
        p1 += kStatic*Math.signum(p1);
        p2 += kStatic*Math.signum(p2);
        p3 += kStatic*Math.signum(p3);
        p4 += kStatic*Math.signum(p4);
        if (useIMU) {
            switch (loops % 5) { //%4 + 1 or 5
                case 1: leftFront.setPower(p1);break;
                case 2: leftRear.setPower(p2);break;
                case 3: rightRear.setPower(p3);break;
                case 4: rightFront.setPower(p4);break;
            }
        }
        else{
            switch (loops % 4) {
                case 0: leftFront.setPower(p1);break;
                case 1: leftRear.setPower(p2);break;
                case 2: rightRear.setPower(p3);break;
                case 3: rightFront.setPower(p4);break;
            }
        }
    }
    public double getMax(double d1, double d2, double d3, double d4){
        double absD1 = Math.abs(d1);
        double absD2 = Math.abs(d2);
        double absD3 = Math.abs(d3);
        double absD4 = Math.abs(d4);
        return Math.max(Math.max(absD1,absD2),Math.max(absD3,absD4));
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void pinMotorPowers(double v, double v1, double v2, double v3) {
        if (useIMU) {
            switch (loops % 5) { //%4 + 1 or 5
                case 1: leftFront.setPower(v);break;
                case 2: leftRear.setPower(v1);break;
                case 3: rightRear.setPower(v2);break;
                case 4: rightFront.setPower(v3);break;
            }
        }
        else{
            switch (loops % 4) {
                case 0: leftFront.setPower(v);break;
                case 1: leftRear.setPower(v1);break;
                case 2: rightRear.setPower(v2);break;
                case 3: rightFront.setPower(v3);break;
            }
        }
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}