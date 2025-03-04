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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  SparkMax m_pivotMotor;
  private SparkClosedLoopController closedLoopControllerCl;
  private RelativeEncoder climbEncoder;
  /** Creates a new climber. */
  public climber() {
    m_pivotMotor = new SparkMax(41, MotorType.kBrushless); //Change the device id to correct one
    closedLoopControllerCl = m_pivotMotor.getClosedLoopController();
    climbEncoder = m_pivotMotor.getEncoder();
    //position = climb.getPosition();

    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);
    climbConfig.closedLoop
      //Change values later
      .p(.15)
      .i(0)
      .d(.05)
      .outputRange(-.7,.7 );
    climbConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(1)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(-41);

    m_pivotMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  /*
   * FUCNTIONS FOR CLIMB
   */
  

   public double getPosition(){
    return climbEncoder.getPosition();
  }

  public void setReach(){
    closedLoopControllerCl.setReference(-27, ControlType.kPosition);
  }

  public void setStow(){
    closedLoopControllerCl.setReference(-2, ControlType.kPosition);
  }

  public void stop(){
    m_pivotMotor.stopMotor();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
