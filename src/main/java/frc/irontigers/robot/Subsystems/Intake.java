// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonFX intakeDom;
  private WPI_TalonFX intakeSub;

  private final LinearFilter currentFilter;
  private final LinearFilter velocityFilter;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeDom = new WPI_TalonFX(Constants.IntakeVals.INTAKE_MASTER);
    intakeSub = new WPI_TalonFX(Constants.IntakeVals.INTAKE_SUB);
    
    intakeSub.follow(intakeDom);
    intakeSub.setInverted(true);

    intakeDom.setNeutralMode(NeutralMode.Brake);
    intakeSub.setNeutralMode(NeutralMode.Brake);

    currentFilter = LinearFilter.movingAverage(5);
    velocityFilter = LinearFilter.movingAverage(3);
  }

  public void setIntakeSpeed(double speed) {
    intakeDom.set(speed);
  }

  public double getCombinedSupplyCurrent() {
    return currentFilter.calculate((intakeDom.getSupplyCurrent() + intakeSub.getSupplyCurrent()) / 2.0);
  }

  public double getCombinedRPM() {
    return velocityFilter.calculate(((intakeDom.getSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0) + (intakeSub.getSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0)) / 2.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake/Current/Leader", intakeDom.getSupplyCurrent());
    // SmartDashboard.putNumber("Intake/Current/Follower", intakeSub.getSupplyCurrent());
    SmartDashboard.putNumber("Intake/Current/CombinedAvg",getCombinedSupplyCurrent());

    // SmartDashboard.putNumber("Intake/Velocity/Leader", intakeDom.getSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0);
    // SmartDashboard.putNumber("Intake/Velocity/Follower", intakeSub.getSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0);
    SmartDashboard.putNumber("Intake/Velocity/CombinedAvg", getCombinedRPM());
  }
}
