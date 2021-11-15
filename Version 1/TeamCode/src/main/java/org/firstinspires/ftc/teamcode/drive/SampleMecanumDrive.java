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

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7, 0.075,0.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(100, 10, 0);

    private PIDFController turnController;

    public static double LATERAL_MULTIPLIER = 1.75;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public int staticHeading;

    public TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;

    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;
    public ExpansionHubMotor leftFront, leftRear, rightRear, rightFront, intake, turret, slides;
    public AnalogInput rightIntake, leftIntake, rightWall, leftWall;
    double rightIntakeVal, leftIntakeVal, rightWallVal, leftWallVal;
    long lastTouchPoll;
    long lastTiltPoll;

    public int intakeCase;
    public int lastIntakeCase;
    private long intakeTime;
    private long slideTime;
    boolean transferMineral = false;
    boolean startIntake = false;
    public int slidesCase;
    private int lastSlidesCase;
    boolean startSlides = false;

    public double slideExtensionLength = 0;
    public double turretHeading = 0;
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;
    public double slideTickToInch = 71.0953;
    public double turretTickToRadians = 578.3213;

    private boolean deposit = false;

    private double intakeTurretInterfaceHeading = 57.5;

    public ArrayList<Servo> servos;

    double currentIntake = 0;

    public static ColorSensor color;

    public BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    public Localizer localizer;

    public static int[] encoders;

    public int loops = 0;

    public Pose2d currentPose = new Pose2d(0,0,0);
    public Pose2d currentVelocity = new Pose2d(0,0,0);
    public Pose2d relCurrentVelocity = new Pose2d(0,0,0);

    boolean tiltForward = false;
    boolean tiltBackward = false;

    long tiltTime;

    public long depositTime = 0;

    boolean isKnownY = true;
    boolean isKnownX = true;
    boolean lastLightReading = false;

    Orientation imuAngle;
    boolean updateIMU = false;

    boolean firstOffBarrier = false;

    Vec3F finalTiltHeading;

    robotComponents r;

    long firstTiltTime;

    private boolean display3WheelOdo;

    public SampleMecanumDrive(HardwareMap hardwareMap){
        this(hardwareMap, false, false,false);
    }
    public SampleMecanumDrive(HardwareMap hardwareMap, boolean threeWheel, boolean t265){ this(hardwareMap, threeWheel, t265,false);}

    public SampleMecanumDrive(HardwareMap hardwareMap, boolean threeWheel, boolean t265, boolean mergeT265Odo) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        long currentTime = System.currentTimeMillis();
        lastTouchPoll = currentTime;
        firstTiltTime = currentTime;
        lastTiltPoll = currentTime;
        tiltTime = currentTime;
        intakeTime = currentTime;
        slideTime = currentTime;

        staticHeading = 0;
        r = new robotComponents(true);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(1, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        color = hardwareMap.colorSensor.get("cs"); //TODO: Replace this with an analog sensor
        color.enableLed(true);

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront =     (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftRear =      (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightRear =     (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront =    (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        // add more motors here
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        slides.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setTargetPosition(0);
        slides.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        servos = new ArrayList<>();
        for (int i = 0; i < 12; i ++) {
            switch (i){
                case 0: servos.add(hardwareMap.servo.get("rightIntake")); break;
                case 1: servos.add(hardwareMap.servo.get("leftIntake")); break;
                case 2: servos.add(hardwareMap.servo.get("deposit")); break;
                case 3: servos.add(hardwareMap.servo.get("odoLift")); break;
                case 4: servos.add(hardwareMap.servo.get("v4bar")); break;
                case 5: servos.add(hardwareMap.servo.get("rightCapstone")); break;
                case 6: servos.add(hardwareMap.servo.get("leftCapstone")); break;
            }
        }

        rightIntake = hardwareMap.analogInput.get("rightIntake");
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        rightWall = hardwareMap.analogInput.get("rightWall");
        leftWall = hardwareMap.analogInput.get("leftWall");
        rightIntakeVal = 0;
        leftIntakeVal = 0;
        rightWallVal = 0;
        leftWallVal = 0;



        // reverse any motors using DcMotor.setDirection()
        // if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = new Localizer();
        encoders = new int[localizer.encoders.length];
        setLocalizer(localizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);


        display3WheelOdo = threeWheel;
        display3WheelOdo = display3WheelOdo && localizer.encoders.length == 4;
        if (display3WheelOdo){
            trajectorySequenceRunner.initThreeWheelRobot();
        }
        if (t265 || mergeT265Odo){
            if (t265) {
                trajectorySequenceRunner.initT265Robot();
            }
            initT265(hardwareMap);
            if (mergeT265Odo){
                localizer.mergeT265Odo = true;
            }
        }

        intakeCase = 0;
        lastIntakeCase = 0;
        slidesCase = 0;
        lastSlidesCase = 0;
    }

    public void resetAssemblies(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initT265(HardwareMap hardwareMap){
        localizer.a = new T265();
        localizer.a.T265Init(new Pose2d(-8.3,-1.5),0.680755, hardwareMap.appContext);//was 1.8574
        localizer.a.start();
        localizer.useT265 = true;
        localizer.T265Start = System.currentTimeMillis();
    }

    public void getEncoders(){
        bulkData = expansionHub1.getBulkInputData();
        encoders[0] = bulkData.getMotorCurrentPosition(rightFront);
        encoders[1] = bulkData.getMotorCurrentPosition(leftFront);
        encoders[2] = bulkData.getMotorCurrentPosition(rightRear);
        if (encoders.length == 4) {
            encoders[3] = bulkData.getMotorCurrentPosition(leftRear);
        }
        //TODO: Get the analog input devices to all be on the control hub
        rightIntakeVal = bulkData.getAnalogInputValue(rightIntake);
        leftIntakeVal = bulkData.getAnalogInputValue(leftIntake);
        rightWallVal = bulkData.getAnalogInputValue(rightWall);
        leftWallVal = bulkData.getAnalogInputValue(leftWall);

        // you can set the bulkData to the other expansion hub to get data from the other one
        if (!(slidesCase == 0) || intakeCase == 4 ){ //|| (currentIntake == 1 && intakeCase == 2)) { // the only encoders on the second hub are for the the turret and the slides (all of these are in slides case and none are in the intake case)
            bulkData = expansionHub2.getBulkInputData();
            slideExtensionLength = bulkData.getMotorCurrentPosition(slides)/slideTickToInch;
            turretHeading = bulkData.getMotorCurrentPosition(turret)/turretTickToRadians;
            //leftIntakeVal = bulkData.getAnalogInputValue(leftIntake);
        }
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
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectorySequenceBuilder(
                startPose, null, reversed,
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
        if (!isKnownX || !isKnownY){
            updateImuAngle();
            localizer.updateHeading(imuAngle.firstAngle);
        }
        localizer.updateEncoders(encoders);
        localizer.updateOdo = !tiltBackward && !tiltForward;
        localizer.update();
        currentPose = getPoseEstimate();
        currentVelocity = getPoseVelocity();
        relCurrentVelocity = localizer.getRelPoseVelocity();
    }
    public void updateImuAngle(){
        if (!updateIMU) {
            updateIMU = true;
            imuAngle = imu.getAngularOrientation();
        }
    }
    public void updateVisualizer(){
        r.setOdoColor(isKnownX && isKnownY);
    }

    public void startIntake(boolean rightIntake){
        double targetIntake = 1;
        if (rightIntake){
            targetIntake = -1;
        }
        if (currentIntake != targetIntake){
            currentIntake = targetIntake;
            if (!transferMineral || slidesCase >= 6){
                turret.setTargetPosition((int)(Math.toRadians(intakeTurretInterfaceHeading)*currentIntake*turretTickToRadians));
                turret.setPower(1.0);
            }
        }
        startIntake = true;
    }

    public void startDeposit(Pose2d relTarget, double height){
        targetTurretHeading = Math.atan2(relTarget.getY() * -1,relTarget.getX() * -1);
        height -= 8.86;
        double length = Math.sqrt(Math.pow(relTarget.getY(),2) + Math.pow(relTarget.getX(),2));
        double v4BarLength = 8;
        double slope = 0.1228;
        double a = (slope*slope + 1);
        double b = -1.0*(2*length + 2*slope*height);
        double c = length*length - Math.pow(v4BarLength,2) + height * height;
        double slideExtension = (-1.0 * b - Math.sqrt(b*b - 4.0 * a * c)) / (2.0 * a);
        targetSlideExtensionLength = slideExtension * 1.0098;
        targetV4barOrientation = Math.atan2(height - slideExtension*slope,slideExtension - length);
        while (targetV4barOrientation < 0){
            targetV4barOrientation += Math.PI * 2;
        }
        targetV4barOrientation -= Math.toRadians(13.1);
        startSlides = true;
    }
    public void deposit(){
        deposit = true;
    }

    public void updateIntake(){
        if (lastIntakeCase != intakeCase) {
            switch (intakeCase) {
                case 1: if(currentIntake == 1){servos.get(1).setPosition(0.068);} if(currentIntake == -1){servos.get(0).setPosition(0.762);} break; // rotate the servo down
                case 2: intake.setPower(-0.85); break; // turn on the intake (forward)
                case 3: if(currentIntake == 1){servos.get(1).setPosition(0.747);} if(currentIntake == -1){servos.get(0).setPosition(0.113);} break; // lift up the servo
                case 4: turret.setTargetPosition((int)(Math.toRadians(intakeTurretInterfaceHeading)*currentIntake*turretTickToRadians)); turret.setPower(1.0); break; //send turret to the correct side
                case 5: intake.setPower(0.6); break; // rotate the servo backward
                case 6: transferMineral = true; intake.setPower(0); depositTime = System.currentTimeMillis(); break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis();
        }
        lastIntakeCase = intakeCase;
        int a = intakeCase;
        switch (a) {
            case 1: if (System.currentTimeMillis() - intakeTime >= 500){intakeCase ++;} break;  // waiting for the servo to drop
            case 2: if (currentIntake == -1 && rightIntakeVal <= 300){intakeCase ++;} if (currentIntake == 1 && leftIntakeVal <= 300){intakeCase ++;} break;
            case 3: if (System.currentTimeMillis() - intakeTime >= 600 && !transferMineral){intakeCase ++;} break;  // waiting for the servo to go up && slides to be back
            case 4: if(Math.abs(turretHeading - Math.toRadians(intakeTurretInterfaceHeading)*currentIntake) <= Math.toRadians(1)){intakeCase ++;} break;//wait for the slides to be in the correct orientation
            case 5: if (System.currentTimeMillis() - intakeTime >= 950){intakeCase ++;} break;  // waiting for mineral to leave the intake
        }
    }

    public void updateSlides(){
        //TODO: implement v4bar as a servo
        if (transferMineral) { // I have deposited into the area
            if (lastSlidesCase != slidesCase) {
                switch (slidesCase) {
                    case 1: // rotate turret
                        turret.setTargetPosition((int)(targetTurretHeading*turretTickToRadians)); turret.setPower(1.0);break;
                    case 2: // extend slides & v4bar & servo pre-tilt
                        slides.setTargetPosition((int)(targetSlideExtensionLength*slideTickToInch)); slides.setPower(1.0);
                        setV4barOrientation(targetV4barOrientation);
                        servos.get(2).setPosition(0.406); break;
                    case 4: //deposit
                        servos.get(2).setPosition(1); break;
                    case 5: // go back to start
                        slides.setTargetPosition(0); slides.setPower(1.0);
                        setV4barOrientation(0);
                        servos.get(2).setPosition(0.29); break;
                    case 6: // rotate turret back
                        turret.setTargetPosition((int)(Math.toRadians(intakeTurretInterfaceHeading)*currentIntake*turretTickToRadians)); turret.setPower(1.0); break;
                }
                slideTime = System.currentTimeMillis();
            }
            lastSlidesCase = slidesCase;
            int a = slidesCase;
            switch (a) {
                case 1: //wait for turret to get near to end
                    if (Math.abs(turretHeading - targetTurretHeading) <= Math.toRadians(15)){slidesCase ++;} break;
                case 2: //wait for arm to be over area
                    if (Math.abs(slideExtensionLength - targetSlideExtensionLength) <= 1 && System.currentTimeMillis() - slidesCase >= targetV4barOrientation * 238.7){slidesCase ++;} break;
                case 3: //wait for everything to get to the end and it wants to deposit
                    if (Math.abs(turretHeading - targetTurretHeading) <= Math.toRadians(5) && deposit){slidesCase ++;} break;
                case 4: //wait for the block to drop => reset the intakeCase
                    if (System.currentTimeMillis() - slideTime >= 200){slidesCase ++; intakeCase = 0; lastIntakeCase = 0;} break;
                case 5: //wait for the arm to be at start
                    if (Math.abs(slideExtensionLength) <= 10){slidesCase ++;} break;
                case 6: //wait for the intake to be facing correct direction
                    if (Math.abs(turretHeading - Math.toRadians(intakeTurretInterfaceHeading)*currentIntake) <= Math.toRadians(5)){slidesCase ++;} break;
                case 7: //wait for the arm to be at start
                    if (Math.abs(slideExtensionLength) <= 1){slidesCase ++;} break;
                case 9: //resets the slidesCase & officially says mineral has not been transfered
                    transferMineral = false; slidesCase = 0; lastSlidesCase = 0; deposit = false; break;
            }
        }
    }

    public void setV4barOrientation(double targetV4barOrientation){
        servos.get(4).setPosition(targetV4barOrientation); //TODO: Find the position to angle
    }

    public void updateScoring(){
        if (startIntake && intakeCase == 0){
            intakeCase = 1;
            intakeTime = System.currentTimeMillis();
            startIntake = false;
        }
        if (startSlides && slidesCase == 0){
            slidesCase = 1;
            slideTime = System.currentTimeMillis();
            startSlides = false;
        }

        updateIntake();
        updateSlides();
    }

    public void update() {
        loops ++;
        updateEstimate();

        if (display3WheelOdo){
            trajectorySequenceRunner.updateThreeWheelPose(localizer.currentThreeWheelPose);
        }
        if (localizer.useT265){
            trajectorySequenceRunner.t265Confidence = localizer.confidence;
            trajectorySequenceRunner.t265Velocity = localizer.relT265Vel;
            trajectorySequenceRunner.updateT265(localizer.T265Pose);
        }

        updateSensor();
        updateVisualizer();
        DriveSignal signal = trajectorySequenceRunner.update(currentPose, relCurrentVelocity, r);
        if (signal != null) {
            updateDriveMotors(signal);
        }

        updateScoring();
        updateIMU = false;
    }
    public void updateSensor(){
        updateLineDetection();
        updateWallDetection();
        updateOdoOverBarrier();
    }
    public void updateOdoOverBarrier(){
        boolean overTrackForward = Math.abs(currentPose.getY()) < 72-48-(12.0/2.0) && Math.abs(currentPose.getX()-24) < 16 + 2;
        boolean overTrackLR = Math.abs(Math.abs(currentPose.getY())-24) < 16 + 2 && Math.abs(currentPose.getX()-(72-43.5/2.0)) < 43.5/2.0 - 12.0/2.0;
        if ((!isKnownX || !isKnownY) || ((overTrackForward || overTrackLR) && System.currentTimeMillis() - lastTiltPoll > 100)){
            updateImuAngle();
            lastTiltPoll = System.currentTimeMillis();
            if (Math.abs(Math.toDegrees(imuAngle.thirdAngle))>1){
                if (!firstOffBarrier) {
                    firstTiltTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - firstTiltTime > 50) {
                    tiltTime = System.currentTimeMillis();
                    isKnownY = false;
                    isKnownX = false;
                    finalTiltHeading = new Vec3F((float)clipHeading(currentPose.getHeading()), imuAngle.secondAngle, imuAngle.thirdAngle);//imuAngle.firstAngle
                    tiltForward = imuAngle.secondAngle > 0;
                    tiltBackward = !tiltForward;
                    firstOffBarrier = true;
                    servos.get(3).setPosition(0.668);
                }
            }
            else{
                firstTiltTime = System.currentTimeMillis();
                if (System.currentTimeMillis()-tiltTime > 50){
                    //when this is first true then we update the pose
                    if (firstOffBarrier){
                        firstOffBarrier = false;
                        double heading = finalTiltHeading.getData()[0];
                        double m1 = 1.0;
                        if (finalTiltHeading.getData()[2] < 0){
                            m1 = -1.0; // facing upward ==> when you face upward at the end it means you went over backward
                        }
                        boolean forward = Math.abs(heading) < Math.toRadians(15);
                        boolean backward = Math.abs(heading) > Math.toRadians(180 - 15);
                        boolean left = Math.abs(heading - Math.toRadians(90)) < Math.toRadians(15);
                        boolean right =  Math.abs(heading + Math.toRadians(90)) < Math.toRadians(15);
                        if (forward || backward){
                            double m2 = 1.0;
                            if (backward){
                                m2 = -1.0;
                            }
                            localizer.setX(24 + (10)*m1*m2);
                        }
                        if (left || right){
                            double m2 = 1.0;
                            if (right){
                                m2 = -1.0;
                            }
                            localizer.setY((24 + (10)*m1*m2) * Math.signum(currentPose.getY()));
                        }
                    }
                    tiltForward = false;
                    tiltBackward = false;
                    servos.get(3).setPosition(0.48);
                }
            }
        }
    }
    public void updateWallDetection(){
        if ((!isKnownX || !isKnownY) || Math.abs(currentPose.getX()) >= 72 - 6 - 4 || Math.abs(currentPose.getY()) >= 72 - 6 - 4){
            double heading = clipHeading(currentPose.getHeading());
            //TODO: Get the actual value for the sensors when in idle
            boolean left = leftWallVal <= 300;
            boolean right = rightWallVal <= 300;
            if (right ^ left){ // this is XOR it means that this or this but not both this and this
                boolean forward = Math.abs(heading) < Math.toRadians(15);
                boolean backward = Math.abs(heading) > Math.toRadians(180 - 15);
                boolean leftRight = Math.abs(Math.abs(heading) - Math.toRadians(90)) < Math.toRadians(15);
                if (forward || backward){
                    double distance;
                    if (left){
                        distance = leftWallVal/100.0 - 3; //TODO: Find the function for light reflectance vs distance
                    }
                    else{
                        distance = rightWallVal/100.0 - 3; //TODO: Find the function for light reflectance vs distance
                    }
                    //TODO: implement kalman filter here
                    localizer.setY((72 - 6.0 - distance) * Math.signum(currentPose.getY()));
                    isKnownY = true;
                }
                else if (leftRight){
                    //TODO: implement kalman filter here
                    double distance;
                    if (left){
                        distance = leftWallVal/100.0 - 3; //TODO: Find the function for light reflectance vs distance
                    }
                    else{
                        distance = rightWallVal/100.0 - 3; //TODO: Find the function for light reflectance vs distance
                    }
                    //TODO: implement kalman filter here
                    localizer.setX((72 - 6.0 - distance) * Math.signum(currentPose.getX()));
                    isKnownX = true;
                }
            }
            lastTouchPoll = System.currentTimeMillis();
        }
    }
    public void updateLineDetection(){
        double robotWidth = 12.5;
        boolean detectLine = false;
        double colorX = 0.996;
        int threshold = 200;
        int sensorThreshold = 3;
        boolean leftRightEntrance = Math.abs(currentPose.getX()-(72-(43.5-1))) < sensorThreshold && Math.abs(Math.abs(currentPose.getY())-(72-robotWidth/2.0)) < sensorThreshold;
        boolean topLeftEntrance = Math.abs(currentPose.getX()-(72-robotWidth/2.0)) < sensorThreshold && Math.abs(currentPose.getY()-(72-(43.5-1))) < sensorThreshold;
        boolean topRightEntrance = Math.abs(currentPose.getX()-(72-robotWidth/2.0)) < sensorThreshold && Math.abs(currentPose.getY()+(72-(43.5-1))) < sensorThreshold;
        if ((!isKnownX || !isKnownY) || leftRightEntrance || topLeftEntrance || topRightEntrance){
            detectLine = color.alpha() > threshold;
        }
        boolean updatePose = lastLightReading != detectLine;
        double multiplier = -1;
        if (detectLine){
            multiplier = 1;
        }
        if (updatePose) {
            double heading = clipHeading(currentPose.getHeading());
            double var = Math.abs(heading)-Math.toRadians(90);
            if (!isKnownX || !isKnownY) {
                if (Math.abs(var) > Math.toRadians(90-15)) { // facing forward or backward (going over the left right line)
                    double speed = Math.signum(currentVelocity.getX()) * multiplier;
                    double m1 = Math.signum(var);
                    localizer.setX(72 - 43.5 + 1 - speed + m1*colorX);
                    isKnownX = true;
                } else if (Math.abs(var) < Math.toRadians(15)) {
                    double m1 = Math.signum(heading)*-1;
                    double speed = Math.signum(currentVelocity.getY()) * multiplier;
                    localizer.setY((72 - 43.5 + 1) * m1 - speed + colorX*m1);
                    isKnownY = true;
                }
            } else if (leftRightEntrance) {
                double speed = Math.signum(currentVelocity.getX()) * multiplier;
                double m1 = Math.signum(heading)*-1.0;
                localizer.setX(72 - 43.5 + 1 - speed + m1*colorX);
                isKnownX = true;
            } else if (topLeftEntrance || topRightEntrance) {
                double m1 = 1;
                if (topRightEntrance){
                    m1 = -1;
                }
                double m2 = Math.signum(heading);
                double speed = Math.signum(currentVelocity.getY()) * multiplier;
                localizer.setY((72 - 43.5 + 1) * m1 - speed - colorX*m2);
                isKnownY = true;
            }
        }
        lastLightReading = detectLine;
    }
    public double clipHeading (double heading){
        while (heading > Math.PI) {
            heading -= Math.PI * 2.0;
        }
        while (heading < -Math.PI) {
            heading += Math.PI * 2.0;
        }
        return heading;
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
        switch (loops % 4) {
            case 0: leftFront.setPower(p1);break;
            case 1: leftRear.setPower(p2);break;
            case 2: rightRear.setPower(p3);break;
            case 3: rightFront.setPower(p4);break;
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

    public void updateWait(long time){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            update();
        }
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
        switch (loops % 4) {
            case 0: leftFront.setPower(v);break;
            case 1: leftRear.setPower(v1);break;
            case 2: rightRear.setPower(v2);break;
            case 3: rightFront.setPower(v3);break;
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
        // This must be changed to match your configuration
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

    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }
}