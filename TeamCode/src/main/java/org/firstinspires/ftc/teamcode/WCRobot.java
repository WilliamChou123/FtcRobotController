package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "WCRobot", group = "Linear Opmode")
public class WCRobot extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor Outtake;
    DcMotor Intake;
    Servo arm;
    DcMotor frontRight;
    BHI260IMU imu;

    //        public void turnCCW(int deg) {
//            motorLeft.setPower(-1);
//            motorRight.setPower(1);
//            sleep((long)460 / 90 * deg);
//            turnOff();
//        }
//        public void turnCW(int deg) {
//            motorLeft.setPower(1);
//            motorRight.setPower(-1);
//            sleep((long)460 / 90 * deg);
//            turnOff();
//        }
    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void fieldCentricDrive(Gamepad gamepadStick) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void robotOrientedDrive(Gamepad gamepadStick) {
        frontLeft.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        frontRight.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x + gamepadStick.right_stick_x);
        backLeft.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        backRight.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x + gamepadStick.right_stick_x);
    }


    public double getRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    private double error = 0.5;

    //    public void turnCW(double targetAngle) {
//        double Kp = 0.01; // Adjust this experimentally
//        double error = targetAngle - imu.getAngularOrientation().firstAngle;
//
//        while (Math.abs(error) > 1) { // 1 degree tolerance
//            error = targetAngle - imu.getAngularOrientation().firstAngle;
//
//            double power = Kp * error;
//
//            // Limit motor power
//            power = Math.max(Math.min(power, 0.5), -0.5);
//
//            // Mecanum turn: CW
//            frontLeft.setPower(power);
//            backLeft.setPower(power);
//            frontRight.setPower(-power);
//            backRight.setPower(-power);
//
//            sleep(10); // Prevent CPU hogging
//        }
//
//        // Stop all motors
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
    public void initImu() {
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(myIMUparameters);
        telemetry.addLine("Initializing IMU...");
        telemetry.update();
        sleep(1000);  // short wait
        telemetry.addLine("IMU Ready!");
        telemetry.update();

    }

    public void updateTelemetry() {
        telemetry.addData("IMU Rotation", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(Servo.class, "arm");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        Outtake.setDirection(DcMotor.Direction.REVERSE);
        initImu();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            updateTelemetry();
            if (gamepad1.right_bumper) {
                imu.resetYaw();
            }
            if (gamepad1.dpad_down) {
                gamepad1.rumble(100);
                arm.setPosition(180);
            }
            if (gamepad1.dpad_up) {
                gamepad1.rumble(100);
                arm.setPosition(0);
            }
            Outtake.setPower(-gamepad1.left_trigger * 1.5);
            Intake.setPower(gamepad1.left_trigger * 2);
            fieldCentricDrive(gamepad1);

        }
    }
}


