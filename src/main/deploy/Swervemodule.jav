package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // Hardware
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absoluteEncoder;

    // Config
    private final double absoluteEncoderOffsetRadians;
    private final boolean steerMotorInverted;
    private final PIDController steerPID = new PIDController(4.0, 0.0, 0.0); // tune PID

    // Gear & wheel
    private static final double DRIVE_GEAR_RATIO = 6.75; // SDS MK4i L2 default
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    private static final double WHEEL_DIAMETER_METERS = 0.098; // 4"

    public SwerveModule(int driveMotorId, int steerMotorId, int canCoderId,
                        double absoluteEncoderOffsetDegrees, boolean steerMotorInverted) {

        this.steerMotorInverted = steerMotorInverted;
        this.absoluteEncoderOffsetRadians = Math.toRadians(absoluteEncoderOffsetDegrees);

        // Initialize hardware
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);
        absoluteEncoder = new CANcoder(canCoderId);

        // Configure Kraken drive
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.getMotorOutput().setInverted(false);
        driveConfig.getCurrentLimits().setSupplyCurrentLimit(40);
        driveMotor.getConfigurator().apply(driveConfig);

        // Configure Kraken steer
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.getMotorOutput().setInverted(steerMotorInverted);
        steerConfig.getCurrentLimits().setSupplyCurrentLimit(30);
        steerMotor.getConfigurator().apply(steerConfig);

        // Enable continuous input for steering PID
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        // Zero steer encoder from CANcoder
        resetSteerEncoder();
    }

    /** Zero steer motor based on CANcoder absolute position */
    public void resetSteerEncoder() {
        double absoluteAngle = getAbsoluteAngleRadians();
        steerMotor.getEncoder().setPosition(absoluteAngle / (2 * Math.PI) * STEER_GEAR_RATIO);
    }

    /** Returns CANcoder absolute angle in radians, normalized [-pi, pi] */
    public double getAbsoluteAngleRadians() {
        double raw = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        double angle = raw - absoluteEncoderOffsetRadians;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /** Current state of this module */
    public SwerveModuleState getState() {
        double velocityMPS = driveMotor.getVelocity().getValueAsDouble()
                * (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;

        double angleRad = steerMotor.getEncoder().getPosition() / STEER_GEAR_RATIO * 2 * Math.PI;

        return new SwerveModuleState(velocityMPS, new Rotation2d(angleRad));
    }

    /** Set the desired module state (speed + angle) */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);

        // Steering PID
        double currentAngleRad = steerMotor.getEncoder().getPosition() / STEER_GEAR_RATIO * 2 * Math.PI;
        double steerOutput = steerPID.calculate(currentAngleRad, state.angle.getRadians());
        steerMotor.set(steerOutput);

        // Drive control (velocity)
        double velocityNativeUnits = state.speedMetersPerSecond / (WHEEL_DIAMETER_METERS * Math.PI) * DRIVE_GEAR_RATIO;
        driveMotor.setControl(new VelocityVoltage(velocityNativeUnits));

        // Telemetry
        SmartDashboard.putNumber("Swerve/" + driveMotor.getDeviceID() + " Angle", Math.toDegrees(currentAngleRad));
        SmartDashboard.putNumber("Swerve/" + driveMotor.getDeviceID() + " Speed MPS", state.speedMetersPerSecond);
    }

    /** Stop both drive and steer motors */
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
