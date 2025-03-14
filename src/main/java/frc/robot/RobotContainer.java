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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.*;

import frc.robot.commands.climb.*;
import frc.robot.commands.legoHand.coral.*;
//import frc.robot.commands.legoHand.algae.*;

import frc.robot.subsystems.climber.climbPosition;
import frc.robot.subsystems.elevator.elevatorPosition;
import frc.robot.subsystems.coral.coralSpeed;
import frc.robot.subsystems.wheelOfDeath.pivotPosition;


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

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final climber climb = new climber();
    public final coral coral = new coral();
    public final elevator elevator = new elevator();
    public final wheelOfDeath pivot = new wheelOfDeath();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        climb.setDefaultCommand(new RunCommand(() -> {climb.setStow();}, climb));
        pivot.setDefaultCommand(pivot.setPostition(pivotPosition.STOW));
        elevator.setDefaultCommand(elevator.setPostition(elevatorPosition.STOW));



        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        final Trigger cStow = joystick.leftBumper();
        cStow.onTrue(new climbStow(climb));
        final Trigger cReach = joystick.rightBumper();
        cReach.onTrue(new climbReach(climb));
        final Trigger cStop = joystick.leftTrigger(.7);
        cStop.onTrue(new climbStop(climb)); 

        final Trigger cIntake = operator.leftTrigger(.5);
        cIntake.onTrue(
            new coralIntake(coral)
            .andThen(coral.waitUntilHasCoral())
            .andThen(new coralStop(coral))
        );
        final Trigger cOuttake = operator.rightTrigger(.5);
        cOuttake.whileTrue(
            new coralOuttake(coral))
            .onFalse(
            coral.waitUntilHasNoCoral()
            .andThen(new coralStop(coral))
        );

        // position triggers
        final Trigger cLayerOne = operator.a();
        cLayerOne.onTrue(
            pivot.setPostition(pivotPosition.SAFE)
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPostition(elevatorPosition.L_ONE))
            .alongWith(pivot.setPostition(pivotPosition.L_ONE))
        );
        final Trigger cLayerTwo = operator.x();
        cLayerTwo.onTrue(
            pivot.setPostition(pivotPosition.SAFE)
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPostition(elevatorPosition.L_TWO))
            .alongWith(pivot.setPostition(pivotPosition.L_TWO))
        );
        final Trigger cLayerThree = operator.b();
        cLayerThree.onTrue(
            pivot.setPostition(pivotPosition.SAFE)
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPostition(elevatorPosition.L_THREE))
            .alongWith(pivot.setPostition(pivotPosition.L_THREE))
        );
        final Trigger cLayerFour = operator.y();
        cLayerFour.onTrue(
            pivot.setPostition(pivotPosition.SAFE)
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPostition(elevatorPosition.L_FOUR))
            .alongWith(pivot.setPostition(pivotPosition.L_FOUR))
        );
        final Trigger cBay = operator.rightBumper();
        cBay.onTrue(
            pivot.setPostition(pivotPosition.SAFE)
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPostition(elevatorPosition.C_BAY))
            .alongWith(pivot.setPostition(pivotPosition.C_BAY))
        );
        final Trigger superStructureStop = operator.start();
        superStructureStop.onTrue(
            pivot.stopMotors()
            .alongWith(elevator.stopMotors())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
