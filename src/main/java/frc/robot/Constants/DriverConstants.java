package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public final class DriverConstants {
        public final static int kPort = 0;
        public final static int kTrey = 1;

        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
        public static final double DRIVE_DEADBAND = 0;
        public static final double ROTATION_DEADBAND = 0;
}