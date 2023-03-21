// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;

public class ArmExtender extends SubsystemBase {
  /** Creates a new ArmExtender. */
  private WPI_TalonFX armExtender;
  private DoubleLogEntry armExtensionLog;

  public ArmExtender() {
    armExtender = new WPI_TalonFX(Constants.ArmVals.ARM_EXTENDER);

    armExtender.setInverted(true);

    DataLog log = DataLogManager.getLog();{
      armExtensionLog = new DoubleLogEntry(log, "arm/extension");
    }

    armExtender.setNeutralMode(NeutralMode.Brake);
  }

  public void setExtensionSpeed(double speed) {
    // if (getArmExtensionPosition() <= 20.9375){
    //   armExtender.set(speed);
    // }else{
    //   armExtender.set(0);
    // }
     armExtender.set(speed);
  }

  public double getArmExtensionPosition() {
    double extensionPosition = armExtender.getSelectedSensorPosition() * Constants.ArmVals.EXTENDER_CONVERSION_FACTOR;
    armExtensionLog.append(extensionPosition);
    
    return extensionPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Length", getArmExtensionPosition());
  }
}
