// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.legoHand.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral;
import frc.robot.subsystems.coral.coralSpeed;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralIntake extends Command {
  private final coral coral;
  /** Creates a new coralIntake. */
  public coralIntake(coral subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    coral = subsystem;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.setSpeed(coralSpeed.INTAKE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
