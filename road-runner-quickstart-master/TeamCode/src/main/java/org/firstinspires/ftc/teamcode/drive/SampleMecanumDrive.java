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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
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
    public static double LATERAL_MULTIPLIER = 1.75;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public int staticHeading;
    public TrajectorySequenceRunner trajectorySequenceRunner;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static double kPSlides = 0.2, kISlides = 0.001, slidesI = 0;

    private final List<DcMotorEx> motors;
    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;
    public ExpansionHubMotor leftFront, leftRear, rightRear, rightFront, intake, turret, slides, slides2;
    public AnalogInput rightIntake, leftIntake, depositSensor, dist, mag1;
    public CRServo duckSpin, duckSpin2;
    public ColorSensor color, leftWall, rightWall;
    public BNO055IMU imu;
    public ArrayList<Servo> servos;
    private final VoltageSensor batteryVoltageSensor;

    long lastTouchPoll, lastTiltPoll, tiltTime;
    public Orientation imuAngle;
    boolean updateIMU = false;
    boolean firstOffBarrier = false;
    Vec3F finalTiltHeading;
    long firstTiltTime;
    boolean tiltForward = false;
    boolean tiltBackward = false;

    ArrayList<Double> depositHistory, intakeHistory;
    public double currentIntake = 0;
    double rightIntakeVal, leftIntakeVal, depositVal, sumIntakeSensor, intakeSensorLoops;
    int intakeMinValRight = 800;//15
    int intakeMinValLeft = 75;//15
    public int intakeCase, lastIntakeCase;
    private long intakeTime, slideTime;
    public boolean transferMineral;
    boolean startIntake = false;
    public int slidesCase, lastSlidesCase;
    boolean startSlides = false;
    public boolean deposit = false;

    public boolean expansion2 = true; //makes expansion hub 2 always on

    public Localizer localizer;
    public boolean isKnownY = true, isKnownX = true;
    public int[] encoders;

    public int loops = 0;

    public Pose2d currentPose = new Pose2d(0,0,0);
    public Pose2d targetPose = null;
    public double targetRadius = 0;
    public Pose2d currentVelocity = new Pose2d(0,0,0);
    public Pose2d relCurrentVelocity = new Pose2d(0,0,0);

    boolean lastLightReading = false;

    robotComponents r;

    private boolean display3WheelOdo;

    public double intakeTurretInterfaceHeading = Math.toRadians(57.5);
    public double v4barInterfaceAngle = Math.toRadians(10);
    public double depositAngle = Math.toRadians(-45);
    public double depositInterfaceAngle = Math.toRadians(65);
    public double depositTransferAngle = Math.toRadians(115);//135

    double targetSlidesPose = 0, slidesSpeed = 0, targetTurretPose = 0, turretPower = 0;
    double currentTargetSlidesPose = 0;

    public int dropIntakeTime = 300;
    public double intakePower = -1;
    public int liftIntakeTime = 500;
    //TODO: Values here changed
    public int transfer1Time = 500; //300
    public int transfer2Time = 700; //300
    public double transfer1Power = 1.0;
    public double transfer2Power = 0.78; //0.85
    public int closeDepositTime = 250;
    public int openDepositTime = 400;
    public int intakeLiftDelay = 100;
    public int effectiveDepositTime = openDepositTime;
    public double returnSlideLength = 1.0; //0.75

    public double slideExtensionLength = 0;
    public double turretHeading = 0;
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;
    public double slideTickToInch = 25.1372713591;
    public double turretTickToRadians = 578.3213;
    double currentV4barAngle = 0;
    double targetV4barAngle = 0;

    public boolean intakeDepositTransfer = false, intakeHit = false;
    long startIntakeDepositTransfer, startIntakeHit;

    double targetDepositAngle = 0;

    double voltage = 0;
    long voltageStart = System.currentTimeMillis();

    double tF = 32767.0 / (1150.0 / 60.0 * 145.1);
    double tP = tF * 0.1 + 0.2;
    double tI = tF * 0.01;
    double tD = 0;
    double tPP = 15;

    public double leftIntakeDrop;
    public double leftIntakeRaise;
    public double rightIntakeDrop;
    public double rightIntakeRaise;
    public double leftIntakeMid;
    public double rightIntakeMid;

    public double turretOffset;
    public double slidesOffset;
    public double v4barOffset;

    double deleteLater = 0;

    double distVal = 0;
    double mag1Val = 0;

    ArrayList<Pose2d> poseHistory;
    private final FtcDashboard dashboard;

    long intakeDelay;
    long depositDelay;
    long slidesDelay;

    public double loopSpeed = 0;

    long lastLoopTime = System.nanoTime();

    public SampleMecanumDrive(HardwareMap hardwareMap){
        this(hardwareMap, false, false,false);
    }

    public SampleMecanumDrive(HardwareMap hardwareMap, boolean threeWheel, boolean t265){ this(hardwareMap, threeWheel, t265,false);}

    public SampleMecanumDrive(HardwareMap hardwareMap, boolean threeWheel, boolean t265, boolean mergeT265Odo) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        long currentTime = System.currentTimeMillis();
        startIntakeDepositTransfer = currentTime;
        lastTouchPoll = currentTime;
        firstTiltTime = currentTime;
        lastTiltPoll = currentTime;
        tiltTime = currentTime;
        intakeTime = currentTime;
        slideTime = currentTime;
        intakeDelay = currentTime;
        depositDelay = currentTime;
        slidesDelay = currentTime;

        staticHeading = 0;
        r = new robotComponents(true);

        PIDFController turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.25, 0.25, Math.toRadians(5.0)), 0.5);

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
        leftWall = hardwareMap.colorSensor.get("leftWall");
        rightWall = hardwareMap.colorSensor.get("rightWall");
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
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");

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
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //TODO: was run without encoder
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intakeSensorLoops = 1;
        sumIntakeSensor = 0;

        transferMineral = false;

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
                case 7: servos.add(hardwareMap.servo.get("duckSpinSpin")); break;
                case 8: servos.add(hardwareMap.servo.get("rightOdo")); break;
                case 9: servos.add(hardwareMap.servo.get("leftOdo")); break;
            }
        }
        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin2 = hardwareMap.crservo.get("duckSpin2");
        rightIntake = hardwareMap.analogInput.get("rightIntake");
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        depositSensor = hardwareMap.analogInput.get("depositSensor");
        dist = hardwareMap.analogInput.get("dist");
        mag1 = hardwareMap.analogInput.get("magLeft");
        rightIntakeVal = 0;
        leftIntakeVal = 0;


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

        turretOffset = 0;
        slidesOffset = 0;
        v4barOffset = 0;

        turret.setVelocityPIDFCoefficients(tP,tI,tD,tF);
        turret.setPositionPIDFCoefficients(tPP);

        leftIntakeDrop = 0.0;
        leftIntakeRaise = 0.72;//0.655
        leftIntakeMid = 0.646;
        rightIntakeDrop = 0.796;
        rightIntakeRaise = 0.08;//0.202
        rightIntakeMid = 0.191;

        servos.get(0).setPosition(rightIntakeRaise);
        servos.get(1).setPosition(leftIntakeRaise);
        setV4barDeposit(depositInterfaceAngle,v4barInterfaceAngle);

        dropOdo();
        servos.get(7).setPosition(0.863);

        setSlidesLength(returnSlideLength);

        poseHistory = new ArrayList<>();
        depositHistory = new ArrayList<>();
        intakeHistory = new ArrayList<>();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public void resetAssemblies(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //TODO: was run without encoder
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetSlides(){
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        rightIntakeVal = bulkData.getAnalogInputValue(rightIntake);
        leftIntakeVal = bulkData.getAnalogInputValue(leftIntake);
        depositVal = bulkData.getAnalogInputValue(depositSensor);

        if (intakeCase >= 1 && intakeCase <= 4){
            if(currentIntake == 1){
                sumIntakeSensor += leftIntakeVal;
                intakeSensorLoops ++;
            }
            if(currentIntake == -1){
                sumIntakeSensor += rightIntakeVal;
                intakeSensorLoops ++;
            }
        }

        // you can set the bulkData to the other expansion hub to get data from the other one
        if (!(slidesCase == 0) || intakeCase == 4 || intakeCase == 5 || expansion2){ // the only encoders on the second hub are for the the turret and the slides (all of these are in slides case and none are in the intake case)
            RevBulkData bulkData = expansionHub2.getBulkInputData();
            slideExtensionLength = bulkData.getMotorCurrentPosition(slides2)/slideTickToInch;
            deleteLater = bulkData.getMotorCurrentPosition(slides)/slideTickToInch;
            turretHeading = bulkData.getMotorCurrentPosition(turret)/turretTickToRadians;
            distVal = bulkData.getAnalogInputValue(dist);
            mag1Val = bulkData.getAnalogInputValue(mag1);
        }
    }

    public Pose2d getRelError(Pose2d target){
        return new Pose2d(
                Math.cos(currentPose.getHeading()) * (target.getX()-currentPose.getX()) + Math.sin(currentPose.getHeading()) * (target.getY()-currentPose.getY()),
                Math.cos(currentPose.getHeading()) * (target.getY()-currentPose.getY()) - Math.sin(currentPose.getHeading()) * (target.getX()-currentPose.getX()),
                target.getHeading()-currentPose.getHeading()
        );
    }

    public void updateMotors(Pose2d relError, double power, double maxPowerTurn, double slowDownDist, double slowTurnAngle, double error, double sideKStatic){
        double powerAdjust = power-kStatic;
        double turnAdjust = maxPowerTurn-kStatic;
        double maxPowerSide = Math.min(0.4,power);
        double sideAdjust = maxPowerSide-kStatic;
        double forward = Math.min(Math.max(relError.getX()*power/slowDownDist,-powerAdjust),powerAdjust) + Math.signum(relError.getX()) * kStatic * Math.max(Math.signum(Math.abs(relError.getX()) - error),0);
        double left = Math.min(Math.max(relError.getY()*maxPowerSide/slowDownDist,-sideAdjust),sideAdjust) + Math.signum(relError.getY()) * kStatic * Math.max(Math.signum(Math.abs(relError.getY()) - error),0) + sideKStatic;
        double turn = Math.min(Math.max(relError.getHeading()*maxPowerTurn/slowTurnAngle,-turnAdjust),turnAdjust) + Math.signum(relError.getHeading()) * kStatic * Math.max(Math.signum(Math.abs(relError.getHeading()) - slowTurnAngle),0);
        double [] p = new double[4];
        p[0] = forward-left-turn;
        p[1] = forward+left-turn;
        p[2] = forward-left+turn;
        p[3] = forward+left+turn;
        double max = 1;
        for (double v : p) {
            max = Math.max(Math.abs(v), max);
        }
        max *= 1.0/(1.0 - kStatic);
        for (int i = 0; i < p.length; i ++){
            p[i] /= max;
            p[i] += kStatic * Math.signum(p[i]);
        }
        pinMotorPowers(p[0], p[1], p[2], p[3]);
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
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
        }
        startIntake = true;
        intakeDelay = System.currentTimeMillis();
    }

    public void startDeposit(Pose2d endPose, Pose2d targetPose, double height, double radius){
        double turretX = -0.75;
        double depositLength = 4.0;
        endPose = new Pose2d(
                endPose.getX() + Math.cos(endPose.getHeading()) * turretX,
                endPose.getY() + Math.sin(endPose.getHeading()) * turretX,
                endPose.getHeading()
        );
        double d = Math.sqrt(Math.pow(targetPose.getX() - endPose.getX(),2) + Math.pow(targetPose.getY() - endPose.getY(),2));
        double x1 = targetPose.getX() + radius * -1.0 * (targetPose.getX()-endPose.getX())/d;
        double y1 = targetPose.getY() + radius * -1.0 * (targetPose.getY()-endPose.getY())/d;
        Pose2d relTarget = new Pose2d(
                Math.cos(endPose.getHeading())*(endPose.getX()-x1) + Math.sin(endPose.getHeading())*(endPose.getY()-y1),
                Math.cos(endPose.getHeading())*(endPose.getY()-y1) - Math.sin(endPose.getHeading())*(endPose.getX()-x1)
        );
        targetTurretHeading = Math.atan2(relTarget.getY(),relTarget.getX());
        height -= (9.44882 + Math.sin(depositAngle) * depositLength);
        double effectiveSlideAngle = Math.toRadians(8.92130165444);
        double v4BarLength = 8.75;
        double slope = Math.tan(effectiveSlideAngle);
        double length = Math.sqrt(Math.pow(relTarget.getY(),2) + Math.pow(relTarget.getX(),2)) - Math.cos(depositAngle) * depositLength - 7.9503937/Math.cos(effectiveSlideAngle);
        double a = (slope*slope + 1);
        double b = -1.0*(2*length + 2*slope*height);
        double c = length*length - Math.pow(v4BarLength,2) + height * height;
        if (4.0 * a * c < b * b) {
            double slideExtension = (-1.0 * b - Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
            targetSlideExtensionLength = slideExtension / (Math.cos(effectiveSlideAngle));
            targetV4barOrientation = Math.atan2(height - slideExtension * slope, slideExtension - length);
        }
        else{
            targetSlideExtensionLength = length - v4BarLength;
            targetV4barOrientation = Math.toRadians(180);
        }
        while (targetV4barOrientation < 0){
            targetV4barOrientation += Math.PI * 2;
        }
        targetSlideExtensionLength = Math.max(0,targetSlideExtensionLength);
        startSlides = true;
        slidesDelay = System.currentTimeMillis();
    }

    public void deposit(){
        //Log.e("deposit", "depositFunction");
        deposit = true;
        depositDelay = System.currentTimeMillis();
    }

    public void dropOdo(){
        servos.get(3).setPosition(0.48);
        servos.get(9).setPosition(0.748);
        servos.get(8).setPosition(0.155);
    }

    public void raiseOdo(){
        servos.get(3).setPosition(0.668);
        servos.get(9).setPosition(0.01);
        servos.get(8).setPosition(0.884);
    }

    public void updateIntake(){
        if (intakeCase == 0 && slidesCase == 0){
            setDepositAngle(depositInterfaceAngle);
            setV4barOrientation(v4barInterfaceAngle);
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);

            if (targetV4barAngle != currentV4barAngle){
                setSlidesLength(returnSlideLength + 8, 0.4);
            }
            else{
                setSlidesLength(returnSlideLength, 0.4);
            }
            switch ((int) currentIntake) {
                case -1:
                    servos.get(1).setPosition(leftIntakeMid);
                    servos.get(0).setPosition(rightIntakeRaise);
                    break;
                case 0:
                    servos.get(0).setPosition(rightIntakeMid);
                    servos.get(1).setPosition(leftIntakeMid);
                    break;
                case 1:
                    servos.get(0).setPosition(rightIntakeMid);
                    servos.get(1).setPosition(leftIntakeRaise);
                    break;
            }
            intake.setPower(0);
        }
        if (intakeCase >= 4 && intakeCase != 9){
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
        }
        if (lastIntakeCase != intakeCase) {
            switch (intakeCase) {
                case 1: intakeSensorLoops = 1; sumIntakeSensor = 0;break; // rotate the servo down
                case 2: intake.setPower(intakePower); break; // turn on the intake (forward)
                case 3:
                    if (System.currentTimeMillis() - intakeTime >= intakeLiftDelay) {
                        if (currentIntake == 1) {
                            servos.get(1).setPosition(leftIntakeRaise);
                        }
                        if (currentIntake == -1) {
                            servos.get(0).setPosition(rightIntakeRaise);
                        }
                    }
                    break; // lift up the servo
                case 4: setTurretTarget(intakeTurretInterfaceHeading * currentIntake); setSlidesLength(returnSlideLength); break; //send turret to the correct side
                case 5:
                    setDepositAngle(depositInterfaceAngle);
                    setV4barOrientation(v4barInterfaceAngle);
                case 6:
                    intake.setPower(transfer1Power);
                break;
                case 7:
                    intake.setPower(transfer2Power);
                break;
                case 8:
                    setSlidesLength(7,0.35);
                    break;
                case 9:
                    intake.setPower(0); transferMineral = true; intakeDepositTransfer = false;
                    //Log.e("Average Intake Val",sumIntakeSensor/intakeSensorLoops + "");
                    break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis();
        }
        lastIntakeCase = intakeCase;
        int a = intakeCase;
        switch (a) {
            case 1: case 2:
                if (intakeCase == 1 && System.currentTimeMillis() - intakeTime >= dropIntakeTime){intakeCase ++;}// waiting for the servo to drop
                if (intakeCase == 2 && (currentIntake == -1 && rightIntakeVal >= intakeMinValRight) || (currentIntake == 1 && leftIntakeVal >= intakeMinValLeft)){intakeCase ++;}

                if(currentIntake == 1){servos.get(1).setPosition(leftIntakeDrop);}
                if(currentIntake == -1){servos.get(0).setPosition(rightIntakeDrop);}
            break; // wait for block in
            case 3:
                if (System.currentTimeMillis() - intakeTime >= liftIntakeTime + intakeLiftDelay && !transferMineral){
                    intakeCase ++;
                }
                if (!transferMineral){
                    setDepositAngle(depositInterfaceAngle);
                    setDepositAngle(depositInterfaceAngle);
                    setV4barOrientation(v4barInterfaceAngle);
                }
                break;  // waiting for the servo to go up && slides to be back 200 before
            case 4: if (Math.abs(turretHeading - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(5)){intakeCase ++;}break;//wait for the slides to be in the correct orientation
            case 5: if (System.currentTimeMillis() - intakeTime >= 300){intakeCase ++;}break;
            case 6: if ((intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer1Time) && System.currentTimeMillis() - intakeTime >= transfer1Time/2.0){intakeCase ++;}break;
            case 7: if ((intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer2Time)){intakeCase ++;}break;
            case 8:
                if (System.currentTimeMillis() - intakeTime >= closeDepositTime/2.0) {
                    setV4barOrientation(Math.toRadians(90));
                    setDepositAngle(depositTransferAngle);
                }
                if (System.currentTimeMillis() - intakeTime >= closeDepositTime){
                    intakeCase ++;
                }
                break; //&& targetV4barOrientation == currentV4barAngle
        }
    }

    public void updateSlides(){
        if (transferMineral) { // I have deposited into the area
            if (lastSlidesCase != slidesCase) {
                slideTime = System.currentTimeMillis();
            }
            lastSlidesCase = slidesCase;
            int a = slidesCase;
            switch (a) {
                case 1: case 2: case 3:
                    setV4barOrientation(targetV4barOrientation + v4barOffset);
                    double l = (Math.abs(slideExtensionLength - targetSlideExtensionLength - slidesOffset));
                    double slidePower = 0.5;//0.95
                    if (l < 10) { //15
                        setSlidesLength(targetSlideExtensionLength + slidesOffset,(slidePower - 0.35) + (targetSlideExtensionLength + slidesOffset - slideExtensionLength)/10.0 * 0.35);
                    } else {
                        setDepositAngle(depositTransferAngle);
                        setSlidesLength(targetSlideExtensionLength + slidesOffset - 8,slidePower); //1
                    }
                    setTurretTarget(targetTurretHeading + turretOffset);
                    if (slidesCase == 1 && Math.abs(turretHeading - (targetTurretHeading + turretOffset)) <= Math.toRadians(15)){slidesCase ++;Log.e("here","1");}
                    if (slidesCase == 2 && (Math.abs(slideExtensionLength - (targetSlideExtensionLength + slidesOffset)) <= 3 || System.currentTimeMillis() - slideTime >= 1000)){slidesCase ++;Log.e("here","2");} //3
                    if (slidesCase == 3 && Math.abs(turretHeading - (targetTurretHeading + turretOffset)) <= Math.toRadians(5) && deposit && targetV4barAngle == currentV4barAngle){slidesCase ++;Log.e("here", "3");}
                    break;
                case 4:
                    if (System.currentTimeMillis()-slideTime >= 0) {//120
                        setDepositAngle(Math.toRadians(180) - depositAngle);
                    }
                    setTurretTarget(targetTurretHeading + turretOffset);
                    setV4barOrientation(targetV4barOrientation + v4barOffset);
                    setSlidesLength(targetSlideExtensionLength + slidesOffset);
                    if (slidesCase == 4 && System.currentTimeMillis() - slideTime >= effectiveDepositTime){slidesCase ++; intakeCase = 0; lastIntakeCase = 0;Log.e("here", "4");} // + 70
                    break;
                case 5 : case 6: case 7: case 8:
                    if (targetV4barAngle != currentV4barAngle){
                        setSlidesLength(returnSlideLength + 8, 0.4);
                    }
                    else{
                        setSlidesLength(returnSlideLength, 0.4);
                        if (slidesCase == 8 && Math.abs(slideExtensionLength - returnSlideLength) <= 1){slidesCase ++;Log.e("here", "8");}
                    }
                    if (slidesCase <= 6) {
                        setTurretTarget(targetTurretHeading + turretOffset);
                    }
                    else {
                        setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
                    }
                    setV4barOrientation(v4barInterfaceAngle);
                    setDepositAngle(depositInterfaceAngle);
                    if (slidesCase == 5 && System.currentTimeMillis() - slideTime >= openDepositTime){slidesCase ++;Log.e("here", "5");}
                    if (slidesCase == 6 && Math.abs(slideExtensionLength-returnSlideLength) <= 10){slidesCase ++;Log.e("here", "6");}
                    if (slidesCase == 7 && Math.abs(turretHeading - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(10)){slidesCase ++;Log.e("here", "7");}
                    break;
                case 9: //resets the slidesCase & officially says mineral has not been transferred
                    transferMineral = false; slidesCase = 0; lastSlidesCase = 0; deposit = false; effectiveDepositTime = openDepositTime;
                    break;
            }
        }
    }

    public void setV4barDeposit(double targetDepositAngle, double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
        currentV4barAngle = targetV4barOrientation;
        this.targetDepositAngle = targetDepositAngle;
        updateDepositAngle();
        updateV4barAngle(0);
    }

    public void updateDepositAngle(){
        double angle = targetDepositAngle - currentV4barAngle;
        double targetPos = angle * 0.215820468 + 0.5;
        targetPos = Math.min(Math.max(targetPos,0.246),1.0);
        servos.get(2).setPosition(targetPos);
    }

    public void updateV4barAngle(double loopSpeed){
        currentV4barAngle += Math.signum(targetV4barAngle - currentV4barAngle) * Math.PI / 0.95 * loopSpeed; //TODO: Changed it from 0.75 seconds for 180 degrees to 0.95 sec
        if (Math.abs(targetV4barAngle - currentV4barAngle) < Math.toRadians(10)){
            currentV4barAngle = targetV4barAngle;
        }
        double servoPos = (targetV4barAngle * -0.201172) + 0.94;
        servoPos = Math.max(Math.min(servoPos,0.94),0.0);
        servos.get(4).setPosition(servoPos);
    }

    public void setDepositAngle(double targetAngle){
        targetDepositAngle = targetAngle;
    }

    public void setV4barOrientation(double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
    }

    public void setSlidesLength(double inches){
        targetSlidesPose = inches;
        slidesSpeed = 1;
    }

    public void setSlidesLength(double inches, double speed){
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }

    public void updateSlidesLength(){
        double maxSlideSpeed = 42/0.5; //42 inches per second
        double dif = (targetSlidesPose) - currentTargetSlidesPose;
        currentTargetSlidesPose += Math.signum(dif) * slidesSpeed * loopSpeed * maxSlideSpeed;
        if (Math.abs(dif) <= 1){
            currentTargetSlidesPose = targetSlidesPose;
        }
        double p = (currentTargetSlidesPose - slideExtensionLength) * kPSlides;
        if (loops >= 2) {
            slidesI += (currentTargetSlidesPose - slideExtensionLength) * loopSpeed * kISlides;
        }
        double kStatic = Math.signum(currentTargetSlidesPose - slideExtensionLength) * slidesSpeed/2.0;
        if (Math.abs(currentTargetSlidesPose - slideExtensionLength) <= 5){
            p /= 2;
            kStatic /= 2;
            slidesI = 0;
            if (currentTargetSlidesPose - slideExtensionLength >= 0){
                //p = 0;
                kStatic = 0.05;
            }
            else {
                if (currentTargetSlidesPose - slideExtensionLength <= -0.5) { // -1
                    kStatic = -0.3;
                }
                else{
                    p = 0;
                }
            }
        }
        slides.setPower(kStatic + p + slidesI);
        slides2.setPower(kStatic + p + slidesI);
    }

    public void setTurretTarget(double radians){
        targetTurretPose = radians;
        turretPower = 0.35;
    }

    public void updateTurretLength(){
        if (Math.abs(targetTurretPose - turretHeading) >= Math.toRadians(3)){
            turret.setPower(Math.signum(targetTurretPose - turretHeading) * Math.abs(turretPower));
        }
        else if (Math.abs(targetTurretPose - turretHeading) >= Math.toRadians(0.5)){
            turret.setPower(Math.signum(targetTurretPose - turretHeading) * 0.15);
        }
        else {
            turret.setPower(0);
        }
    }

    public void updateScoring(){
        if (System.currentTimeMillis() - intakeDelay >= 1000){
            startIntake = false;
        }
        if (System.currentTimeMillis() - depositDelay >= 1000){
            deposit = false;
        }
        if (System.currentTimeMillis() - slidesDelay >= 3000){
            startSlides = false;
        }
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

        long currentTime = System.nanoTime();
        if (loops == 1){
            lastLoopTime = currentTime;
        }
        loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;

        if (display3WheelOdo){
            trajectorySequenceRunner.updateThreeWheelPose(localizer.currentThreeWheelPose);
        }
        if (localizer.useT265){
            trajectorySequenceRunner.t265Confidence = localizer.confidence;
            trajectorySequenceRunner.t265Velocity = localizer.relT265Vel;
            trajectorySequenceRunner.updateT265(localizer.T265Pose);
        }

        updateDepositAngle();
        updateV4barAngle(loopSpeed);

        updateSensor();
        updateVisualizer();

        updateSlidesLength();
        updateTurretLength();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        poseHistory.add(currentPose);

        if (poseHistory.size() > 100) {
            poseHistory.remove(0);
        }

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));
        packet.put("velX", relCurrentVelocity.getX());
        packet.put("velY", relCurrentVelocity.getY());

        packet.put("velHeading (deg)", Math.toDegrees(relCurrentVelocity.getHeading()));
        packet.put("v4bar (deg)", Math.toDegrees(targetV4barAngle));
        packet.put("v4bar Actual (deg)", Math.toDegrees(currentV4barAngle));

        packet.put("intakeCase", intakeCase);
        packet.put("slidesCase", slidesCase);
        packet.put("depositVal", depositVal);
        packet.put("turret Heading", turretHeading);
        packet.put("slides length", slideExtensionLength);
        packet.put("slides length2", deleteLater);
        packet.put("slides length2", deleteLater);
        packet.put("mag1", mag1Val);
        packet.put("dist",distVal/6.4);
        packet.put("target Slide Length 1", currentTargetSlidesPose);
        packet.put("target Slide Length", targetSlidesPose);

        depositHistory.add(depositVal);
        if (depositHistory.size() > 50){
            depositHistory.remove(0);
        }
        double sumDeposit = 0;
        for (double v: depositHistory){
            sumDeposit += v;
        }
        double criticalVal = Math.pow(10,(sumDeposit/(1000*depositHistory.size()))*0.266769051121 - 0.896739150596);
        criticalVal += (1.0-criticalVal)*0.3;
        if (depositVal/(sumDeposit/depositHistory.size()) < criticalVal){
            intakeDepositTransfer = true;
            startIntakeDepositTransfer = System.currentTimeMillis();
        }
        if (intakeDepositTransfer && System.currentTimeMillis() - startIntakeDepositTransfer > 100){
            intakeDepositTransfer = false;
        }

        packet.put("criticalDepositVal", criticalVal);
        packet.put("averageDepositVal", sumDeposit/50.0);
        packet.put("depositValDelta", depositVal/(sumDeposit/50.0));

        double intakeCurrent = 0;
        double averageIntakeCurrent = 0;
        double intakeCurrentCriticalValTop = 1.2;
        double intakeCurrentCriticalValBottom = 0.875;
        double intakeDeltaCurrent = 0;
        if (2 <= intakeCase && intakeCase <= 7){
            intakeCurrent = intake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            intakeHistory.add(intakeCurrent);
            if (intakeHistory.size() > 150){
                intakeHistory.remove(0);
            }
            double sumIntake = 0;
            for (double v: intakeHistory){
                sumIntake += v;
            }
            averageIntakeCurrent = sumIntake/intakeHistory.size();
            intakeDeltaCurrent = intakeCurrent/averageIntakeCurrent;
            if (intakeDeltaCurrent > intakeCurrentCriticalValTop && System.currentTimeMillis() - intakeTime >= 100){
                intakeHit = true;
                startIntakeHit = System.currentTimeMillis();
            }
            if (intakeDeltaCurrent < intakeCurrentCriticalValBottom && System.currentTimeMillis() - intakeTime >= 100){
                intakeDepositTransfer = true;
                startIntakeDepositTransfer = System.currentTimeMillis();
            }
        }
        if (intakeHit && System.currentTimeMillis() - startIntakeHit > 50){
            intakeHit = false;
        }
        packet.put("intake Current Draw", intakeCurrent);
        packet.put("averageIntakeCurrentDraw", averageIntakeCurrent);
        packet.put("intakeCurrentCriticalValTop", intakeCurrentCriticalValTop);
        packet.put("intakeCurrentCriticalValBottom", intakeCurrentCriticalValBottom);
        packet.put("intakeDeltaCurrent", intakeDeltaCurrent);

        packet.put("leftIntake", leftIntakeVal);
        packet.put("rightIntake", rightIntakeVal);

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        if (targetPose != null){
            drawRobot(fieldOverlay, r, targetPose);
            fieldOverlay.strokeCircle(targetPose.getX(),targetPose.getY(),targetRadius);
        }

        drawRobot(fieldOverlay,r,currentPose);

        dashboard.sendTelemetryPacket(packet);

        updateScoring();
        updateIMU = false;

        if (loops % 10 == 0){
            voltage = getBatteryVoltage();
        }
        if (voltage >= 12.75){
            voltageStart = System.currentTimeMillis();
        }
    }

    public void drawRobot(Canvas fieldOverlay, robotComponents r, Pose2d poseEstimate){
        for (Component c : r.components){
            fieldOverlay.setStrokeWidth(c.lineRadius);
            fieldOverlay.setStroke(c.color);
            if (c.p.size() == 1){
                drawPoint(fieldOverlay,c.p.get(0),c.radius,poseEstimate);
            }
            else {
                for (int i = 1; i < c.p.size() + 1; i++) {
                    drawPoint(fieldOverlay, c.p.get(i % c.p.size()), c.radius, poseEstimate);
                    drawLine(fieldOverlay, c.p.get(i % c.p.size()), c.p.get((i - 1)%c.p.size()), poseEstimate);
                }
            }
        }
    }

    public void drawLine(Canvas c, Point p, Point p2, Pose2d pose){
        double x1 = Math.cos(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) - Math.sin(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + pose.getX();
        double y1 = Math.cos(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + Math.sin(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) + pose.getY();
        double x2 = Math.cos(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) - Math.sin(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + pose.getX();
        double y2 = Math.cos(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + Math.sin(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) + pose.getY();
        c.strokeLine(x1,y1,x2,y2);
    }

    public void drawPoint(Canvas c, Point p, double radius, Pose2d pose){
        if (radius != 0){
            double x = Math.cos(pose.getHeading())*p.x - Math.sin(pose.getHeading())*p.y + pose.getX();
            double y = Math.cos(pose.getHeading())*p.y + Math.sin(pose.getHeading())*p.x + pose.getY();
            c.strokeCircle(x,y,radius);
        }
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
            double denominator = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denominator);
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

    public void updateSensor(){
        updateWallDetection();
        updateLineDetection(); //not working currently (decreases accuracy)
        //updateOdoOverBarrier(); not working currently
    }

    public void updateWallDetection() {
        long currentTime = System.currentTimeMillis();
        double detectionDist = 6.75;
        double extraOffset = 5.0;
        double maxDetectionLocation = 72.0 - detectionDist - extraOffset;
        if ((currentTime - lastTouchPoll >= 100 && (Math.abs(currentPose.getX()) > maxDetectionLocation || Math.abs(currentPose.getY()) > maxDetectionLocation)) || !isKnownY || !isKnownX) {
            boolean leftSensor = leftWall.argb() >= 400000000;
            boolean rightSensor = rightWall.argb() >= 400000000;
            double heading = clipHeading(currentPose.getHeading());
            if (leftSensor ^ rightSensor) { // this is XOR it means that this or this but not both this and this
                double angleOffset = 15;
                boolean forward = Math.abs(heading) < Math.toRadians(angleOffset);
                boolean backward = Math.abs(heading) > Math.toRadians(180 - angleOffset);
                boolean left = Math.abs(heading - Math.toRadians(90)) < Math.toRadians(angleOffset);
                boolean right = Math.abs(heading + Math.toRadians(90)) < Math.toRadians(angleOffset);
                double distance = 0.5;
                double currentXDist = Math.cos(heading) * (5.0) - Math.sin(heading) * (6.25 + distance);
                double currentYDist = Math.cos(heading) * (6.25 + distance) + Math.sin(heading) * (5.0);
                double gain = 0.6;
                if (forward || backward) {
                    if (Math.abs(relCurrentVelocity.getY()) < 0.5){
                        if (forward){
                            //localizer.setPoseEstimate(new Pose2d(currentPose.getX(),currentPose.getY(),(currentPose.getHeading() * 9.0 + 0)/10.0));
                        }
                        if (backward){
                            //localizer.setPoseEstimate(new Pose2d(currentPose.getX(),currentPose.getY(),(Math.abs(currentPose.getHeading()) * 9.0 + Math.toRadians(180))/10.0 * Math.signum(currentPose.getHeading())));
                        }
                    }
                    double m = 1;
                    if (backward) {
                        m = -1;
                    }
                    double side = Math.signum(currentPose.getY());
                    if (!isKnownY) {
                        if ((leftSensor && side == m) || (rightSensor && side == -1 * m)) {
                            isKnownY = true;
                            localizer.setY((72 - Math.abs(currentYDist)) * side);
                        }
                    } else {
                        if (((leftSensor && m == side) || (rightSensor && m == -1 * side)) && Math.abs(currentPose.getY()) > maxDetectionLocation) {
                            isKnownY = true;
                            localizer.setY(currentPose.getY() * (1.0 - gain) + (72 - Math.abs(currentYDist)) * side * gain);
                        }
                    }
                }
                if (right || left) {
                    if (Math.abs(relCurrentVelocity.getY()) < 0.5){
                        if (right){
                            //localizer.setPoseEstimate(new Pose2d(currentPose.getX(),currentPose.getY(),(Math.abs(currentPose.getHeading()) * 9.0 + Math.toRadians(-90))/10.0));
                        }
                        if (left){
                            //localizer.setPoseEstimate(new Pose2d(currentPose.getX(),currentPose.getY(),(Math.abs(currentPose.getHeading()) * 9.0 + Math.toRadians(90))/10.0));
                        }
                    }
                    double m = 1;
                    if (left) {
                        m = -1;
                    }
                    double side = Math.signum(currentPose.getX());
                    if (!isKnownX) {
                        if ((leftSensor && side == m) || (rightSensor && side == -1 * m)) {
                            isKnownX = true;
                            localizer.setX((72 - Math.abs(currentXDist)) * side);
                        }
                    } else {
                        // if ((left sensor and facing wall) or (right sensor and facing opposite of wall)) and (near wall)
                        if (((leftSensor && m == side) || (rightSensor && m == -1 * side)) && Math.abs(currentPose.getX()) > maxDetectionLocation) {
                            isKnownX = true;
                            //teleports the robot to the wall closest to where it currently is
                            localizer.setX(currentPose.getX() * (1.0 - gain) + (72 - Math.abs(currentXDist)) * side * gain);
                        }
                    }
                }
            }
            lastTouchPoll = currentTime;
        }
    }

    public void updateOdoOverBarrier(){
        boolean overTrackForward = Math.abs(currentPose.getY()) < 72-48-(12.0/2.0) && Math.abs(currentPose.getX()-24) < 16 + 2;
        boolean overTrackLR = Math.abs(Math.abs(currentPose.getY())-24) < 16 + 2 && Math.abs(currentPose.getX()-(72-43.5/2.0)) < 43.5/2.0 - 12.0/2.0;
        if ((!isKnownX || !isKnownY) || ((overTrackForward || overTrackLR) && System.currentTimeMillis() - lastTiltPoll > 100)){
            updateImuAngle();
            lastTiltPoll = System.currentTimeMillis();
            Log.e("IMU Tilt", " " + Math.toDegrees(imuAngle.thirdAngle));
            if (Math.abs(Math.toDegrees(imuAngle.thirdAngle))>1){
                if (!firstOffBarrier) {
                    firstTiltTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - firstTiltTime > 50) {
                    tiltTime = System.currentTimeMillis();
                    isKnownY = false;
                    isKnownX = false;
                    finalTiltHeading = new Vec3F((float)clipHeading(currentPose.getHeading()), imuAngle.secondAngle, imuAngle.thirdAngle);//imuAngle.firstAngle
                    tiltForward = imuAngle.thirdAngle > 0;
                    tiltBackward = !tiltForward;
                    firstOffBarrier = true;
                    raiseOdo();
                }
            }
            else{
                firstTiltTime = System.currentTimeMillis();
                if (System.currentTimeMillis()-tiltTime > 50){
                    //when this is first true then we update the pose
                    if (firstOffBarrier){
                        firstOffBarrier = false;
                        double heading = clipHeading(currentPose.getHeading());//finalTiltHeading.getData()[0];
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
                    dropOdo();
                }
            }
        }
    }

    public void updateLineDetection(){
        double robotWidth = 12.5;
        boolean detectLine = false;
        double colorX = 1.25;//0.996
        double detectionDist = 1.25;
        int threshold = 200;
        int sensorThreshold = 10; //5
        boolean leftRight = Math.abs(currentPose.getX()-(72-(43.5-1))) < sensorThreshold && Math.abs(Math.abs(currentPose.getY())-(72-robotWidth/2.0)) < sensorThreshold;
        boolean topLeft = Math.abs(currentPose.getX()-(72-robotWidth/2.0)) < sensorThreshold && Math.abs(currentPose.getY()-(72-(43.5-1))) < sensorThreshold;
        boolean topRight = Math.abs(currentPose.getX()-(72-robotWidth/2.0)) < sensorThreshold && Math.abs(currentPose.getY()+(72-(43.5-1))) < sensorThreshold;
        double velocity = Math.sqrt(Math.pow(relCurrentVelocity.getX(),2) + Math.pow(relCurrentVelocity.getY(),2));
        if ((!isKnownX || !isKnownY) || (leftRight || topLeft || topRight && velocity < 3)){
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
            if (leftRight || (!isKnownX && Math.abs(var) > Math.toRadians(85))) {
                double speed = Math.signum(currentVelocity.getX()) * multiplier * detectionDist;
                double m1 = Math.cos(heading)*colorX;
                double gain = 1.0;
                if (isKnownX){
                    gain = 0.1;
                }
                localizer.setX(currentPose.getX() * (1.0 - gain) + (72 - 43.5 + 1 - speed - m1) * gain);
                isKnownX = true;
            }
            else if (topLeft || topRight || (!isKnownY && Math.abs(var) < Math.toRadians(15))) {
                double m1 = Math.signum(currentPose.getY());
                double m2 = Math.sin(heading)*colorX;
                double speed = Math.signum(currentVelocity.getY()) * multiplier * detectionDist;
                double gain = 1.0;
                if (isKnownY){
                    gain = 0.1;
                }
                localizer.setY(currentPose.getY() * (1.0 - gain) + ((72 - 43.5 + 1) * m1 - speed - m2) * gain);
                isKnownY = true;
            }
        }
        lastLightReading = detectLine;
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

    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public Pose2d getLastError() {return trajectorySequenceRunner.getLastPoseError();}
}