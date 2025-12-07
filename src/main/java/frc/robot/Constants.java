package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose.
 * All constants should be declared globally (i.e. public static). Do not put anything
 * functional in this class.
 */
public final class Constants {
    /**
     * Constants for the intake subsystem.
     */
    public static class IntakeConstants {
        // CAN ID for the intake motor
        // TODO: Set this to the actual CAN ID of your intake motor
        public static final int INTAKE_MOTOR_ID = 27;
        public static final int INDEXER_MOTOR_ID = 29;
    }

    public static class ShooterConstants {
        public static final int SHOOT_MOTOR_ID = 34;
        public static final int HOOD_MOTOR_ID = 32;
        public static final int CAN;
}
