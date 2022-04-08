package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class MecanumTeleOp extends LinearOpMode {
    private ExpansionHubMotor leftFront;
    private ExpansionHubMotor leftRear;
    private ExpansionHubMotor rightRear;
    private ExpansionHubMotor rightFront;

    // TODO Change constants (higher = faster, lower = slower)
    private static final double DRIVE_POWER_CONST = 1.0;
    private static final double TURN_POWER_CONST = 1.0;

    @Override
    public void runOpMode() {
        // TODO Update hardware map configurations if necessary
        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftRear = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightRear = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (!isStopRequested()) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * TURN_POWER_CONST;

            if (withinRange(forward, -0.01, 0.01) && withinRange(strafe, -0.01, 0.01) &&
                    withinRange(turn, -0.01, 0.01)){
                stopMotors();
            } else {
                double speed;
                if (withinRange(forward, -0.01, 0.01) && withinRange(strafe, -0.01, 0.01)) {
                    speed = 0;
                } else if (withinRange(forward, -0.01, 0.01)) {
                    speed = Math.sqrt(2) * Math.abs(strafe);
                } else if (withinRange(strafe, -0.01, 0.01)) {
                    speed = Math.abs(forward);
                } else {
                    speed = Math.min(Math.abs(strafe) + Math.abs(forward), 1);
                }

                driveAtAngle(speed, Math.atan2(strafe, -forward), turn);
            }
        }
    }

    public void driveAtAngle(double speed, double angle, double turn) {
        double desiredAngle = (angle) + Math.PI / 4;

        if (desiredAngle < 0) {
            desiredAngle = desiredAngle + 2 * Math.PI;
        }
        if (desiredAngle >= 2 * Math.PI) {
            desiredAngle = desiredAngle % (2 * Math.PI);
        }

        double intermediateSin = Math.sin(desiredAngle);
        double intermediateCos = Math.cos(desiredAngle);

        double leftfront = speed * (intermediateSin) + (turn * 0.75 / DRIVE_POWER_CONST);
        double leftBackward = speed * (intermediateCos) + (turn * 0.75 / DRIVE_POWER_CONST);
        double rightfront = speed * (intermediateCos) - (turn * 0.75 / DRIVE_POWER_CONST);
        double rightBackward = speed * (intermediateSin) - (turn * 0.75 / DRIVE_POWER_CONST);

        if (Math.abs(rightBackward) < 0.05) {
            rightBackward = 0;
        }
        if (Math.abs(rightfront) < 0.05) {
            rightfront = 0;
        }
        if (Math.abs(leftBackward) < 0.05) {
            leftBackward = 0;
        }
        if (Math.abs(leftfront) < 0.05) {
            leftfront = 0;
        }

        setPower(leftfront, leftBackward, rightfront, rightBackward);
    }

    private void setPower(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        rightFront.setPower(rf);
    }

    private void setPowerAll(double pow) {
        leftFront.setPower(pow);
        leftRear.setPower(pow);
        rightRear.setPower(pow);
        rightFront.setPower(pow);
    }

    private void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    private boolean withinRange(double val, double min, double max){
        return val > min && val < max;
    }
}
