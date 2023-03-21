// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonFX intakeDom;
  private WPI_TalonFX intakeSub;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeDom = new WPI_TalonFX(Constants.IntakeVals.INTAKE_MASTER);
    intakeSub = new WPI_TalonFX(Constants.IntakeVals.INTAKE_SUB);
    
    intakeSub.follow(intakeDom);
    intakeSub.setInverted(true);

    intakeDom.setNeutralMode(NeutralMode.Brake);
    intakeSub.setNeutralMode(NeutralMode.Brake);
  }

  public void setIntakeSpeed(double speed) {
    intakeDom.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
