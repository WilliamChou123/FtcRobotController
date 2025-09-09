package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WCRobot extends LinearOpMode {
        DcMotor motorLeft;
        DcMotor motorRight;
        DcMotor frontLeft;
        DcMotor frontRight;
        ColorSensor color1;
        DistanceSensor distance1;
        BNO055IMU imu;

        public void turnCCW(int deg) {
            motorLeft.setPower(-1);
            motorRight.setPower(1);
            sleep((long)460 / 90 * deg);
            turnOff();
        }
        public void turnCW(int deg) {
            motorLeft.setPower(1);
            motorRight.setPower(-1);
            sleep((long)460 / 90 * deg);
            turnOff();
        }
        public void turnOff() {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
        public void moveForward(int steps, double speed) {
            motorLeft.setPower(speed );
            motorRight.setPower(speed);
            sleep((long) steps * 100);
            turnOff();
        }
        public void setWheelPower(int steps, double left, double right) {
            motorLeft.setPower(left);
            motorRight.setPower(right);
            sleep((long) steps * 100);
            turnOff();
        }
        public void addColorTelemetry() {
            telemetry.addData("Red", (color1.red()));
            telemetry.addData("Blue", (color1.blue()));
            telemetry.update();
        }

        public void moveUntilWall() {
            while (distance1.getDistance(DistanceUnit.CM) >= 50) {
                moveForward(1,1);
            }
        }
        public void outtake() {
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
        }
        public void intake() {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
        }
//        public boolean isWallInFront() {
//              return (distance1.getDistance(DistanceUnit.CM) <= 60)
//        }

        @Override
        public void runOpMode() {
            motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
            motorRight = hardwareMap.get(DcMotor.class, "motorRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            color1 = hardwareMap.get(ColorSensor.class, "color1");
            distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            motorLeft.setDirection(DcMotor.Direction.REVERSE);

            waitForStart();
            sleep(100);
            moveForward(5, 1);
            turnCW(90);

        }
    }


