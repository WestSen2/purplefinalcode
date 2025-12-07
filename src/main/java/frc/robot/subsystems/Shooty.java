package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANCoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder; 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Ground intake subsystem that uses a single motor to intake game pieces.
 */
public class Shooty extends SubsystemBase {
    private final TalonFX shootMotor;
    private final TalonFX hoodMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Use the Phoenix 6 version of CANcoder
    private final CANcoder m_angleSensor; 

    // Intake speeds (in volts)
    private static final double SHOOT_SPEED = 8.0;  // Adjust this value as needed
    private static final double HOOD_CHANGE = 0.1; // For reversing if needed

    private static final double MAX_DEGREES = 10;
    private static final double MIN_DEGREES = 0;

    /**
     * Creates a new GroundIntake subsystem.
     *
     * @param shootID The CAN ID of the shooter motor
     * @param hoodID The CAN ID of the hood motor
     * @param canID The CAN ID of the CANcoder
     */
    public Shooty(int shootID, int hoodID, int canID) {
        shootMotor = new TalonFX(shootID);
        hoodMotor = new TalonFX(hoodID);
        // Instantiate the Phoenix 6 CANcoder with the provided ID
        m_angleSensor = new CANcoder(canID); 

        // Configure the motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootMotor.getConfigurator().apply(config);
        hoodMotor.getConfigurator().apply(config); // Apply config to both motors

        // Configure the CANcoder using the correct Phoenix 6 objects/methods
        CANCoderConfiguration configs = new CANCoderConfiguration();
        configs.MagnetSensor.MagnetOffset = 0.0; // Adjust this value based on your physical setup in Tuner X
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        // Apply the configurations to the sensor instance
        m_angleSensor.getConfigurator().apply(configs);
        
    }

    /**
     * Runs the intake motor to pull in game pieces.
     */
    public void shoot() {
        shootMotor.setControl(voltageRequest.withOutput(SHOOT_SPEED));
    }

    /**
     * Runs the hood motor to aim farther (positive voltage).
     */
    public void aimFarther() {
        if (getAngleDegrees() < MAX_DEGREES) {
            hoodMotor.setControl(voltageRequest.withOutput(HOOD_CHANGE));
        } else {
            stopTurning(); // Stop moving if at max limit
        }
    }

    /**
     * Runs the hood motor to aim closer (negative voltage).
     */
    public void aimCloser() {
        if (getAngleDegrees() > MIN_DEGREES) {
            hoodMotor.setControl(voltageRequest.withOutput(-HOOD_CHANGE));
        } else {
            stopTurning(); // Stop moving if at min limit
        }
    }

    public void stopTurning() {
        hoodMotor.setControl(voltageRequest.withOutput(0));
    }

    /**
     * Stops the intake motor.
     */
    public void stopShooting() {
        shootMotor.setControl(voltageRequest.withOutput(0));
    }

    /**
     * Gets the current angle of the mechanism in degrees.
     * @return The angle in degrees (0 to 10).
     */
    public double getAngleDegrees() {
        // Use the Phoenix 6 method chain: .getAbsolutePosition().getValue()
        double rotations = m_angleSensor.getAbsolutePosition().getValue();

        // Convert rotations (1 rotation = 360 degrees) to degrees
        double degrees = rotations * 360.0;
        
        return degrees;
    }

    /**
     * Command to run the shooter while the button is held.
     *
     * @return Command that runs the shooter
     */
    // <<<< ADDED shootCommand() METHOD BACK IN >>>>
    public Command shootCommand() {
        return this.runEnd(
            this::shoot,
            this::stopShooting
        );
    }

    // Renamed aimFurtherCommand to aimFartherCommand to match method name
    public Command aimFartherCommand() {
        return this.runEnd(
            this::aimFarther,
            this::stopTurning
        );
    }

    public Command aimCloserCommand() {
        return this.runEnd(
            this::aimCloser,
            this::stopTurning
        );
    }

    @Override
    public void periodic() {
        // Put telemetry here if needed
        // SmartDashboard.putNumber("Current Hood Angle", getAngleDegrees());
    }
}
