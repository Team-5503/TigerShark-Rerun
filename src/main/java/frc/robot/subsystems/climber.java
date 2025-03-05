// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * INITIALIZATION
 */

public class climber extends SubsystemBase {
  SparkMax climbMotor;
  private SparkClosedLoopController closedLoopControllerCl;
  private RelativeEncoder climbEncoder;
  private SparkMaxConfig climbConfig;

  public climbPosition currentTargetPosition;
  /** Creates a new climber. */
  public climber() {
    climbMotor = new SparkMax(41, MotorType.kBrushless); //TODO: change value to a constant and fix id if needed
    closedLoopControllerCl = climbMotor.getClosedLoopController();
    climbEncoder = climbMotor.getEncoder();
    currentTargetPosition = climbPosition.STOW;
    //position = climb.getPosition();

    configure();
  }

  /*
   * CONFIGURATION
   */

  private void configure(){
    climbConfig = new SparkMaxConfig();
    //TODO: set up inversion and implement
    climbConfig
      .smartCurrentLimit(80, 40) //TODO: change value(s) to constant(s)
      .idleMode(IdleMode.kBrake); //TODO: change value(s) to a constant
    climbConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(.15, 0, .05, 0) //TODO: change value(s) to constant(s)
      .outputRange(-.8,.8 );
    climbConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(1) //TODO: change value(s) to a constant
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(-63); //TODO: change value(s) to a constant

   climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /*
   * FUCNTIONS FOR SUBSYSTEM
   */
  
  /*
   * FUCNTIONS TO SET MOTORS AND ENCODERS
   */


  public void setReach(){
    closedLoopControllerCl.setReference(-56.25, ControlType.kPosition);
  }

  public void setStow(){
    closedLoopControllerCl.setReference(-4.16, ControlType.kPosition);
  }

  public void setPosition(climbPosition position){
    currentTargetPosition = position;
    closedLoopControllerCl.setReference(position.rotations, ControlType.kPosition);
  }

  public void stop(){
   climbMotor.stopMotor();
  }

  public void resetEncoder(){
    climbEncoder.setPosition(0);
  }

  /*
   * FUNCTIONS TO GET VALUES
   */

  public double getPosition(){
    return climbEncoder.getPosition();
  }

  private double getClimbError() {
    return Math.abs(Math.abs(climbEncoder.getPosition()) - Math.abs(currentTargetPosition.rotations));
  }

  private boolean isAtSetpoint(){
    return (getClimbError() < 5); //TODO: change value to a constant
  }

  /*
   * COMMANDS THAT DO NOT SET ANY POSITIONS
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSetpoint() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtSetpoint();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", getPosition());
    SmartDashboard.putNumber("Target Climb Position", currentTargetPosition.getRotations());
    SmartDashboard.putBoolean("Climb at Setpoint", isAtSetpoint());
  }

  public enum climbPosition {
    // ENUMS FOR POSITIONS 
    STOW(-4.16),
    REACH(-56.25);

    private double rotations;
    /**Constrcutor for rotations for climbPositions (Enum for climb poses)
    * @param rotations
    * verticle movement in rotations
    */
    climbPosition(double rotations) {
        this.rotations = rotations;
    }

    public double getRotations() {
        return this.rotations;
    }
  }
}
