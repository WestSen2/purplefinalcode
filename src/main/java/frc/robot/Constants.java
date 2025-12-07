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
        public static final int INTAKE_MOTOR_ID = 10;

        // Intake motor speeds (in volts)
        public static final double INTAKE_SPEED = 8.0;  // Adjust this value as needed
        public static final double OUTTAKE_SPEED = -6.0; // For reversing if needed

        // Current limit for intake motor (in amps)
        public static final double CURRENT_LIMIT = 40.0;
    }
}
