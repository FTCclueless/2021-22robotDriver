package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
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
import com.qualcomm.robotcore.hardware.CRServo;
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
    private ArrayList<Pose2d> poseHistory;

    private PIDFController turnController;

    private String TAG = "SampleTankDrive";

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

    static RevBulkData bulkData;
    static ExpansionHubEx expansionHub1, expansionHub2;
    public static ExpansionHubMotor leftFront, leftRear, rightRear, rightFront, intake, v4bar, turret, slides;
    static TouchSensor lf, lb, rb, rf;
    long lastTouchPoll;
    long lastTiltPoll;

    public static int intakeCase = 0;
    private int lastIntakeCase = 0;
    private long intakeTime;
    private long slideTime;
    boolean transferMineral = false;
    boolean startIntake = true;
    public static int slidesCase = 0;
    private int lastSlidesCase = 0;
    boolean startSlides = true;

    static double slideExtensionLength = 0;
    static double turretHeading = 0;
    static double v4barOrientation = 0;
    static double targetSlideExtensionLength = 0;
    static double targetTurretHeading = 0;
    static double targetV4barOrientation = 0;
    double slideTickToInch = 0; // number of ticks in an inch
    double turretTickToRadians = 0;
    double v4barTickToRadians = 0;

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
    private boolean displayT265;

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
                new Pose2d(0.25, 0.25, Math.toRadians(1.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        color = hardwareMap.colorSensor.get("cs");
        color.enableLed(true);

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront =     (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftRear =      (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightRear =     (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront =    (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        // add more motors here
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        v4bar  = (ExpansionHubMotor) hardwareMap.dcMotor.get("v4bar");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");


//        dropperServo.setDirection(Servo.Direction.REVERSE);

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

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reverse any motors using DcMotor.setDirection()
        // if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = new Localizer();
        encoders = new int[localizer.encoders.length];
        setLocalizer(localizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        poseHistory = new ArrayList<Pose2d>();


        displayT265 = t265;
        display3WheelOdo = threeWheel;
        display3WheelOdo = display3WheelOdo && localizer.encoders.length == 4;
        if (display3WheelOdo){
            trajectorySequenceRunner.initThreeWheelRobot();
        }
        if (displayT265 || mergeT265Odo){
            if (displayT265) {
                trajectorySequenceRunner.initT265Robot();
            }
            initT265(hardwareMap);
            if (mergeT265Odo){
                localizer.mergeT265Odo = true;
            }
        }
    }

    public void resetAssemblies(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4bar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initT265(HardwareMap hardwareMap){
        localizer.a = new T265();
        localizer.a.T265Init(new Pose2d(-8.3,-1.5),0.05, hardwareMap.appContext);//was 1.8574
        localizer.a.start();
        localizer.useT265 = true;
        localizer.T265Start = System.currentTimeMillis();
    }

    public static void getEncoders(){
        bulkData = expansionHub1.getBulkInputData();
        encoders[0] = bulkData.getMotorCurrentPosition(rightFront);
        encoders[1] = bulkData.getMotorCurrentPosition(leftFront);
        encoders[2] = bulkData.getMotorCurrentPosition(rightRear);
        if (encoders.length == 4) {
            encoders[3] = bulkData.getMotorCurrentPosition(leftRear);
        }
        // you can set the bulkData to the other expansion hub to get data from the other one
        if (!(slidesCase == 0)) { // the only enoders on the second hub are for the v4bar, the turret, and the slides (all of these are in slides case and none are in the intake case)
            bulkData = expansionHub2.getBulkInputData();
            slideExtensionLength = bulkData.getMotorCurrentPosition(slides);
            turretHeading = bulkData.getMotorCurrentPosition(turret);
            v4barOrientation =  bulkData.getMotorCurrentPosition(v4bar);
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
        localizer.updatOdo = !tiltBackward && !tiltForward;
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
        startIntake = true;
    } // TODO: Make sure that the program knows which intake we want to use

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

    public void updateIntake(){
        if (lastIntakeCase != intakeCase) {
            switch (intakeCase) {
                case 1: break; // rotate the servo down
                case 2: intake.setPower(1); break; // turn on the intake (forward)
                case 3: break; // lift up the servo
                case 4: intake.setPower(-1); break; // rotate the servo backward
                case 5: transferMineral = true; intake.setPower(0);  break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis();
        }
        lastIntakeCase = intakeCase;
        int a = intakeCase;
        switch (a) {
            case 1: if (System.currentTimeMillis() - intakeTime >= 200){intakeCase ++;} break;  // waiting for the servo to drop
            case 2: if (System.currentTimeMillis() - intakeTime >= 1000){intakeCase ++;} break;  // waiting for a mineral in intake
            case 3: if (System.currentTimeMillis() - intakeTime >= 200 && slidesCase == 0){intakeCase ++;} break;  // waiting for the servo to go up &&
            case 4: if (System.currentTimeMillis() - intakeTime >= 100){intakeCase ++;} break;  // waiting for mineral to leave the intake
        }
    }

    public void updateSlides(){
        if (transferMineral) { // I have deposited into the area
            if (lastSlidesCase != slidesCase) {
                switch (slidesCase) {
                    case 1: // rotate turret
                        turret.setTargetPosition((int)(targetTurretHeading*turretTickToRadians)); turret.setPower(1.0);break;
                    case 2: // extend slides & v4bar & TODO: pre-tilt the deposit servo a little
                        slides.setTargetPosition((int)(targetSlideExtensionLength*slideTickToInch)); slides.setPower(1.0);
                        v4bar.setTargetPosition((int)(targetV4barOrientation*v4barTickToRadians)); v4bar.setPower(1.0); break;
                    case 3: break; // TODO: servo deposit
                    case 4: // go back to start TODO: reset servo to start pose
                        slides.setTargetPosition(0); slides.setPower(1.0); v4bar.setTargetPosition(0); v4bar.setPower(1.0); break;
                    case 5: // rotate turret back TODO: need to turn it toward the intake that it goes to
                        turret.setTargetPosition(0); turret.setPower(0); break;
                }
                slideTime = System.currentTimeMillis();
            }
            lastSlidesCase = slidesCase;
            int a = slidesCase;
            switch (a) {
                case 1: //wait for turret to rotate
                    if (Math.abs(turretHeading/turretTickToRadians - targetTurretHeading) <= Math.toRadians(1)){slidesCase ++;} break;
                case 2: //wait for arm to be over area TODO: check whether wants to drop (maybe an autoDrop variable vs a manual button input for mechanical)
                    if (Math.abs(slideExtensionLength/slideTickToInch - targetSlideExtensionLength) <= 1 &&
                            Math.abs(v4barOrientation/v4barTickToRadians - targetV4barOrientation) <= Math.toRadians(1)){slidesCase ++;} break;
                case 3: //wait for the block to drop => reset the intakeCase
                    if (System.currentTimeMillis() - slideTime >= 200){slidesCase ++; intakeCase = 0; lastIntakeCase = 0;} break;
                case 4: //wait for the arm to be at start
                    if (Math.abs(slideExtensionLength/slideTickToInch) <= 1 &&
                            Math.abs(v4barOrientation/v4barTickToRadians) <= Math.toRadians(1)){slidesCase ++;} break;
                case 5: //wait for the intake to be facing correct direction TODO: need to turn it toward the intake that it goes to
                    if (Math.abs(turretHeading/turretTickToRadians) <= Math.toRadians(1)){slidesCase ++;} break;
                case 6: //resets the slidesCase & officially says mineral has not been transfered
                    transferMineral = false; slidesCase = 0; lastSlidesCase = 0; break;
            }
        }
    }

    public void updateScoring(){
        if (startIntake && intakeCase == 0){
            intakeCase = 1;
            startIntake = false;
        }
        if (startSlides && slidesCase == 0){
            slidesCase = 1;
            startSlides = false;
        }

        updateIntake();
        updateSlides();
    }

    public void update() {
        loops ++;
        updateEstimate();
        updateSensor();
        updateVisualizer();
        DriveSignal signal = trajectorySequenceRunner.update(currentPose, relCurrentVelocity, r);
        if (signal != null) {
            updateDriveMotors(signal);
        }

        updateScoring();

        if (display3WheelOdo){
            trajectorySequenceRunner.updateThreeWheelPose(localizer.currentThreeWheelPose);
        }
        if (localizer.useT265){
            trajectorySequenceRunner.t265Confidence = localizer.confidence;
            trajectorySequenceRunner.t265Velocity = localizer.relT265Vel;
            trajectorySequenceRunner.updateT265(localizer.T265Pose);
        }
        updateIMU = false;
    }
    public void updateSensor(){
        updateColorSensor();
        //updateTouchSensor();
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
                    //Todo: lift up the odo pods
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
                            setPoseEstimate(new Pose2d(24 + (10)*m1*m2,currentPose.getY(),currentPose.getHeading()));
                            //localizer.x = 24 + (10)*m1*m2;
                        }
                        if (left || right){
                            double m2 = 1.0;
                            if (right){
                                m2 = -1.0;
                            }
                            setPoseEstimate(new Pose2d(currentPose.getX(),(24 + (10)*m1*m2) * Math.signum(currentPose.getY()),currentPose.getHeading()));
                            //localizer.y = (24 + (10)*m1*m2) * Math.signum(currentPose.getY());
                        }
                    }
                    tiltForward = false;
                    tiltBackward = false;
                    //Todo: drop odo pds
                }
            }
        }
    }
    public void updateTouchSensor(){
        if ((!isKnownX || !isKnownY) || System.currentTimeMillis() - lastTouchPoll > 250){
            boolean left = lf.isPressed() && lb.isPressed();
            boolean right = rf.isPressed() && rb.isPressed();
            double heading = clipHeading(currentPose.getHeading());
            if (right ^ left){ // this is XOR it means that this or this but not both this and this
                boolean forward = Math.abs(heading) < Math.toRadians(15);
                boolean backward = Math.abs(heading) > Math.toRadians(180 - 15);
                boolean leftRight = Math.abs(Math.abs(heading) - Math.toRadians(90)) < Math.toRadians(15);
                double multiplier = 1.0;
                if (right){
                    multiplier = -1.0;
                }
                if (forward || backward){
                    double m1 = 1;
                    if (backward){
                        m1 = -1;
                    }
                    if ((!isKnownX || !isKnownY)) {
                        setPoseEstimate(new Pose2d(currentPose.getX(), (72 - 12.0 / 2.0) * multiplier * m1, currentPose.getHeading()));
                    }
                    else if (Math.abs(currentPose.getY()) > 72 - 12.0 / 2.0 - 5){
                        setPoseEstimate(new Pose2d(currentPose.getX(), (72 - 12.0 / 2.0) * Math.signum(currentPose.getY()), currentPose.getHeading())); // * multiplier * m1
                    }
                    //localizer.y = (72-12.0/2.0)*multiplier*m1;
                    isKnownY = true;
                }
                else if (leftRight){
                    if ((!isKnownX || !isKnownY)) {
                        setPoseEstimate(new Pose2d(72-12.0/2.0,currentPose.getY(),currentPose.getHeading()));
                    }
                    else if (Math.abs(currentPose.getX()) > 72 - 12.0 / 2.0 - 5){
                        setPoseEstimate(new Pose2d((72-12.0/2.0)*Math.signum(currentPose.getX()),currentPose.getY(),currentPose.getHeading()));
                    }
                    //localizer.x = 72-12.0/2.0;
                    isKnownX = true;
                }
            }
            lastTouchPoll = System.currentTimeMillis();
        }
    }
    public void updateColorSensor(){
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
                    setPoseEstimate(new Pose2d(72 - 43.5 + 1 - speed + m1*colorX,currentPose.getY(),currentPose.getHeading()));
                    //localizer.x = 72 - 43.5 + 1 - speed + m1*colorX;
                    isKnownX = true;
                } else if (Math.abs(var) < Math.toRadians(15)) {
                    double m1 = Math.signum(heading)*-1;
                    double speed = Math.signum(currentVelocity.getY()) * multiplier;
                    setPoseEstimate(new Pose2d(currentPose.getX(),(72 - 43.5 + 1) * m1 - speed + colorX*m1,currentPose.getHeading()));
                    //localizer.y = (72 - 43.5 + 1) * m1 - speed + colorX*m1;
                    isKnownY = true;
                }
            } else if (leftRightEntrance) {
                double speed = Math.signum(currentVelocity.getX()) * multiplier;
                double m1 = Math.signum(heading)*-1.0;
                setPoseEstimate(new Pose2d(72 - 43.5 + 1 - speed + m1*colorX,currentPose.getY(),currentPose.getHeading()));
                //localizer.x = 72 - 43.5 + 1 - speed + m1*colorX;
                isKnownX = true;
            } else if (topLeftEntrance || topRightEntrance) {
                double m1 = 1;
                if (topRightEntrance){
                    m1 = -1;
                }
                double m2 = Math.signum(heading);
                double speed = Math.signum(currentVelocity.getY()) * multiplier;
                setPoseEstimate(new Pose2d(currentPose.getX(),(72 - 43.5 + 1) * m1 - speed - colorX*m2,currentPose.getHeading()));
                //localizer.y = (72 - 43.5 + 1) * m1 - speed - colorX*m2;
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

    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }
}