package org.firstinspires.ftc.teamcode;

public class CharlieField extends BaseRobot {
    @Override
    public void runOpMode() {
        waitForStart();
        PestoFTCConfig.initializePinpoint = true;
        super.runOpMode();
        while (opModeIsActive() && !isStopRequested()) {
            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }

}
