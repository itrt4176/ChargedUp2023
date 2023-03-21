// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import org.yaml.snakeyaml.scanner.Constant;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Commands.ManualArmRotation;
import frc.irontigers.robot.Constants.ArmVals;

public class ArmRotator extends SubsystemBase {
 

  /** Creates a new Arm. */
  private WPI_TalonFX armRotatorMain;
  private WPI_TalonFX armRotatorSub;

  private double armRotationPosition;
  private Pose2d armExtensionPosition;

  private DoubleLogEntry armPositionLog;

  public ArmRotator() {
    armRotatorMain = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR_MASTER);
    armRotatorSub = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR_SUB);

    armRotatorSub.follow(armRotatorMain);
    armRotatorSub.setInverted(true);

    DataLog log = DataLogManager.getLog();{
      armPositionLog = new DoubleLogEntry(log, "arm/position");
    }

    armRotatorMain.setNeutralMode(NeutralMode.Brake);
  }

  // public WPI_TalonFX getArmExtender() {
  //   return armExtender;
  // }


  public double getArmDegrees(){
    return armRotatorMain.getSelectedSensorPosition() * Constants.ArmVals.PULSES_TO_DEGREES;
  }

  
     
  public void setRotationSpeed(double speed) {
     armRotatorMain.set(speed);
  }

  public boolean isLimitSwitchPressed(){
    return armRotatorMain.getSensorCollection().isRevLimitSwitchClosed() == 1; 
  }

  public double getRotatorPosition() {
    double position = armRotatorMain.getSelectedSensorPosition();
    armPositionLog.append(position);

 
    return position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    
    SmartDashboard.putNumber("Arm Position", getArmDegrees());
    SmartDashboard.putNumber("Raw Rotator Pulses", armRotatorMain.getSelectedSensorPosition());

  }

  

}
 