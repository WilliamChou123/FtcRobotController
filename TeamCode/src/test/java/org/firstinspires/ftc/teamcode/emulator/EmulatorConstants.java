package org.firstinspires.ftc.teamcode.emulator;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/** Desktop-safe mirror of the production constants that AutoPath actually needs. */
final class EmulatorConstants {
    static final String FRONT_LEFT_NAME = "lf";
    static final String FRONT_RIGHT_NAME = "rf";
    static final String BACK_LEFT_NAME = "lr";
    static final String BACK_RIGHT_NAME = "rr";
    static final String IMU_NAME = "imu";

    /*
     * Loading production Constants on a JVM also initializes RevHubOrientationOnRobot, whose
     * implementation requires android.opengl.Matrix. Keep this test-only mirror numerically in
     * sync with Constants.foresightConfig instead of pulling Android/REV initialization into the
     * desktop runner.
     */
    static final ForesightConfig FORESIGHT_CONFIG = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.5);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.5);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);
        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                        .put(2.5, primaryTranslationalForward)
        );
        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                        .put(2.5, primaryTranslationalLateral)
        );
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(3));
        c.headingBrakeCoefficients.set(
                Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695)
        );
        c.linearBrakeCoefficients.set(
                Matrix.diag(0.10605894992901523, 0.08719146175596092)
        );
        c.quadraticBrakeCoefficients.set(
                Matrix.diag(0.0014663966976606565, 0.0013837064502458813)
        );
        c.maxAchievableForwardVelocity.set(72.72923108818539);
        c.maxAchievableStrafeVelocity.set(52.34323936525474);
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
        c.maxPathSpeed.set(0.5);
    });

    private EmulatorConstants() {
    }
}
