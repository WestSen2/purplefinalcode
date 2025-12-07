package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Ground intake subsystem that uses a single motor to intake game pieces.
 */
public class GroundIntake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Creates a new GroundIntake subsystem.
     *
     * @param motorID The CAN ID of the intake motor
     */
    public GroundIntake(int motorID) {

        intakeMotor = new TalonFX(motorID);

        // Configure the motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(config);
        
    }

    /**
     * Runs the intake motor to pull in game pieces.
     */
    public void intake() {
        intakeMotor.setControl(voltageRequest.withOutput(IntakeConstants.INTAKE_SPEED));
    }

    /**
     * Runs the intake motor in reverse to eject game pieces.
     */
    public void outtake() {
        intakeMotor.setControl(voltageRequest.withOutput(IntakeConstants.OUTTAKE_SPEED));
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setControl(voltageRequest.withOutput(0));
    }

    /**
     * Command to run the intake while the button is held.
     *
     * @return Command that runs the intake
     */
    public Command intakeCommand() {
        return this.runEnd(
            this::intake,
            this::stop
        );
    }

    /**
     * Command to run the outtake while the button is held.
     *
     * @return Command that runs the outtake
     */
    public Command outtakeCommand() {
        return this.runEnd(
            this::outtake,
            this::stop
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
