// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooty; 

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final GroundIntake intake = new GroundIntake(Constants.IntakeConstants.INTAKE_MOTOR_ID, Constants.IntakeConstants.INDEXER_MOTOR_ID);
    
    public final Shooty shooterHood = new Shooty(
        Constants.ShooterConstants.SHOOT_MOTOR_ID, 
        Constants.ShooterConstants.HOOD_MOTOR_ID, 
        Constants.ShooterConstants.CANCODER_ID
    );

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // ... (existing drivetrain and SysId code remains unchanged) ...
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding create/options and triangle/square.
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Intake control - L1 (left bumper) runs intake
        joystick.L1().whileTrue(intake.intakeCommand());

        // reset the field-centric heading on touchpad press
        joystick.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // <<<< UPDATED INTAKE/OUTTAKE BINDINGS FOR L2/R2 >>>>
        
        // L2 Trigger (left trigger, check if pressed past 50%) runs intake
        new Trigger(() -> joystick.getL2Axis() > 0.5).whileTrue(intake.intakeCommand());

        // R2 Trigger (right trigger, check if pressed past 50%) runs outtake
        new Trigger(() -> joystick.getR2Axis() > 0.5).whileTrue(intake.outtakeCommand());

        // <<<< SHOOTER/HOOD BINDINGS >>>>
        // Moved the shoot command from L2 to the Square button since L2/R2 are now used for intake/outtake
        joystick.square().whileTrue(shooterHood.shootCommand());

        // D-Pad Up (POV 0) aims the hood farther (moves up) while held
        joystick.povUp().whileTrue(shooterHood.aimFartherCommand());

        // D-Pad Down (POV 180) aims the hood closer (moves down) while held
        joystick.povDown().whileTrue(shooterHood.aimCloserCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
