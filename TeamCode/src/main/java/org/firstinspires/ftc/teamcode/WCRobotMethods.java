package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.log;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class WCRobotMethods {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor Outtake;
    DcMotor Intake;
    Servo arm;
    DcMotor frontRight;
    BHI260IMU imu;
    Telemetry telemetry;
    DcMotor deadWheelLeft;
    DcMotor deadWheelRight;
    DcMotor deadWheelCenter;
    double imu_zero;

    public WCRobotMethods(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
//DeadWheels
        deadWheelCenter = hardwareMap.get(DcMotor.class, "frontRight");
        deadWheelRight = hardwareMap.get(DcMotor.class, "backRight");
        deadWheelLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontLeft.setDirection(FORWARD);
        frontRight.setDirection(REVERSE);
        backLeft.setDirection(FORWARD);
        backRight.setDirection(REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
    }

    public void updateTelemetry(Telemetry telemetry) {
//        telemetry.addData("Center", deadWheelCenter.getCurrentPosition());
//        telemetry.addData("Left", deadWheelLeft.getCurrentPosition());
//        telemetry.addData("Right", deadWheelRight.getCurrentPosition());
        telemetry.update();

    }

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


    public void driveWithStick(Gamepad gamepadStick) {
        robotOrientedDrive(gamepadStick.left_stick_x, -gamepadStick.left_stick_y, gamepadStick.right_stick_x);
    }

    public void resetIMU() {
        deadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        deadWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        imu_zero = imu.getRobotYawPitchRollAngles().getYaw(RADIANS) - offset;
    }

    public void robotOrientedDrive(double x, double y, double r) {
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }

    public void fieldOrientedDrive(double x0, double y0, double r) {
        double theta = getRotation();
        double x = x0 * sin(theta) - y0 * cos(theta);
        double y = x0 * cos(theta) + y0 * sin(theta);
        robotOrientedDrive(x, y, r);
//        telemetry.addData("theta", theta);
//        telemetry.update();
    }

    // Constants — tune these for your robot
    private static final double TICKS_PER_REV = 2000.0;   // your encoder spec
    private static final double WHEEL_RADIUS = 1.378;     // inches
    private static final double GEAR_RATIO = 1.0;         // 1 if encoder on wheel shaft
    private static final double LATERAL_DISTANCE = 12;  // distance between left/right odometry wheels (in)
    private static final double FORWARD_OFFSET = -3.5;    // offset of center (perpendicular) wheel from robot center (in)

    // State
    private int lastLeft = 0, lastRight = 0, lastCenter = 0;
    private double heading = 0.0; // radians total since reset

    public double getRotation() {
        // Read current encoder positions
        int lTicks = deadWheelLeft.getCurrentPosition();
        int rTicks = deadWheelRight.getCurrentPosition();
        int cTicks = deadWheelCenter.getCurrentPosition();

        // Convert to distance (inches)
        double dL = ticksToInches(lTicks - lastLeft);
        double dR = ticksToInches(rTicks - lastRight);
        double dC = ticksToInches(cTicks - lastCenter);

        // Store for next call
        lastLeft = lTicks;
        lastRight = rTicks;
        lastCenter = cTicks;

        // Compute heading change from left/right difference
        double dTheta = (dR - dL) / LATERAL_DISTANCE;

        // Correct center wheel for its offset (prevents false sideways movement)
        double dCenterCorrected = dC - FORWARD_OFFSET * dTheta;

        // Integrate heading (radians)
        heading += dTheta;

        return heading; // radians, counterclockwise positive
    }

    private double ticksToInches(int ticks) {
        return (2 * Math.PI * WHEEL_RADIUS) * (ticks / TICKS_PER_REV) * GEAR_RATIO;
    }


//    public double getRotationOld() {
//        return imu.getRobotYawPitchRollAngles().getYaw(RADIANS) - imu_zero;
//    }

    public void moveTo(double x, double y) {
        robotOrientedDrive(x, y, 0);
//        wait(3);
        stopAll();
    }


    public void initImu() {
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(myIMUparameters);
    }
}


