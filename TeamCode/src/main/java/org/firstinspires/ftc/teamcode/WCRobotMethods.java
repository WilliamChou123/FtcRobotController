package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
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
import com.qualcomm.robotcore.util.Range;

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
    //    DcMotor slide1;
//    DcMotor slide2;
    DcMotor Middle;
    DcMotor Gecko;
    double imu_zero;


    public WCRobotMethods(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Middle = hardwareMap.get(DcMotor.class, "mid");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");
        Gecko = hardwareMap.get(DcMotor.class, "gecko");


//        slide1 = hardwareMap.get(DcMotor.class, "slide1");
//        slide2 = hardwareMap.get(DcMotor.class, "slide2");

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

    public void fieldOrientedDrive() {

    }

    public void resetIMU() {
        deadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        deadWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }

    public double getRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw(RADIANS) - imu_zero;
    }

    public double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    public void moveForward(double time) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < time * 1000) {
            Intake.setPower(0.5);
            robotOrientedDrive(0, -0.5, 0);
        }
        stopAll();
    }

    public void rotate(double time) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < time * 1000) {
            robotOrientedDrive(0, 0, 0.5);
        }
        stopAll();
    }

    public void rotateOp(double time) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < time * 1000) {
            robotOrientedDrive(0, 0, -0.5);
        }
        stopAll();
    }

    public void moveMid(double power) {
        Middle.setPower(power);
        Intake.setPower(power);
        Gecko.setPower(power);
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


