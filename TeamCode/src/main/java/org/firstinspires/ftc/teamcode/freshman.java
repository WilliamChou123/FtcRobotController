package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class freshman extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor frontRight;
    ColorSensor color1;
    DistanceSensor distance1;
//        BNO055IMU imu;

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

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotOrientedDrive(gamepad1);
        }
        sleep(100);
        moveForward(5, 1.0);
    }
}


