package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@Autonomous
public class WCRobotAuto extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor Outtake; //port 2 epansion
    DcMotor Intake; //port 2 epansion

    DcMotor frontRight;
    ColorSensor color1;
    DistanceSensor distance1;
    IMU imu;

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
    public void turnOff() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void moveForward(int steps, double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep((long) steps * 100);
        turnOff();
    }

    //        public void setWheelPower(int steps, double left, double right) {
//            frontLeft.setPower(left);
//            backLeft.setPower(right);
//            sleep((long) steps * 100);
//            turnOff();
//        }
    public void addColorTelemetry() {
        telemetry.addData("Red", (color1.red()));
        telemetry.addData("Blue", (color1.blue()));
        telemetry.update();
    }

    public void moveUntilWall() {
//            while (distance1.getDistance(DistanceUnit.CM) >= 50) {
//                moveForward(1,1);
//            }
    }

    public void outtake() {
//            frontLeft.setPower(0.1);
//            frontRight.setPower(0.1);
    }

    public void intake() {
//            frontLeft.setPower(-1);
//            frontRight.setPower(-1);
    }

    //        public boolean isWallInFront() {
//              return (distance1.getDistance(DistanceUnit.CM) <= 60)
//        }
    public void robotOrientedDrive(Gamepad gamepadStick) {

        frontLeft.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        frontRight.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x + gamepadStick.right_stick_x);
        backLeft.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        backRight.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x + gamepadStick.right_stick_x);
    }

//    private double error = 0.5;

    public void turnCW(double targetAngle) {
        double Kp = 0.01;
        double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (Math.abs(error) > 1) {
            error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            ;
            double power = Kp * error;
            power = Math.max(Math.min(power, 0.5), -0.5) * 10;

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            sleep(10); // Prevent CPU hogging
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void runOpMode() {
//        Intake = hardwareMap.get(DcMotor.class, "intake");
//        Outtake = hardwareMap.get(DcMotor.class, "outtake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Rotation", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update();
//            moveForward(5, 1.0);
//            sleep(1000);
//            turnCW(90);
//            sleep(1000);
        }

    }
}


