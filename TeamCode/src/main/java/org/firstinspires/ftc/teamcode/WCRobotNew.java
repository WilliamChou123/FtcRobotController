package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WCRobotNew {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor Outtake;
    DcMotor Intake;
    Servo arm;
    DcMotor frontRight;
    BHI260IMU imu;
    Telemetry telemetry;

    double imu_zero;

    public WCRobotNew(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


    public void driveWithStick(Gamepad gamepadStick) {
        robotOrientedDrive(gamepadStick.left_stick_x, -gamepadStick.left_stick_y, gamepadStick.right_stick_x);
    }

//    public void resetIMU() {
//        imu_zero = imu.getRobotYawPitchRollAngles().getYaw(RADIANS);
//    }

    public void resetIMU(double offset) {
        imu_zero = imu.getRobotYawPitchRollAngles().getYaw(RADIANS) - offset;
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
        telemetry.addData("theta", theta);
        telemetry.update();
    }


    public double getRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw(RADIANS) - imu_zero;
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


