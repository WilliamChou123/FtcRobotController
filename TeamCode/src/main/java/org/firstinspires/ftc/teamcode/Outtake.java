package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Charlie", group = "Linear Opmode")
public class Outtake extends LinearOpMode {
    DcMotor Outtake1;
    DcMotor Outtake2;

    @Override
    public void runOpMode() {
        Outtake1 = hardwareMap.get(DcMotor.class, "intake");
        Outtake2 = hardwareMap.get(DcMotor.class, "outtake");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            Outtake1.setPower(gamepad1.left_trigger * -2);
            Outtake2.setPower(gamepad1.left_trigger * 2);
        }

    }
}


