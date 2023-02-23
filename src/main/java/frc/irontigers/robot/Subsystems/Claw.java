// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private WPI_TalonFX clawOne;
  private WPI_TalonFX clawTwo;
  private MotorControllerGroup claw;


  private DoubleLogEntry clawOnePositionLog;
  private DoubleLogEntry clawTwoPositionLog;

  public Claw() {
    clawOne = new WPI_TalonFX(Constants.ClawVals.CLAW_ONE);
    clawTwo = new WPI_TalonFX(Constants.ClawVals.CLAW_TWO);
    claw = new MotorControllerGroup(clawOne, clawTwo);

    DataLog log = DataLogManager.getLog();{
      clawOnePositionLog = new DoubleLogEntry(log, "clawOne/position");
      clawTwoPositionLog = new DoubleLogEntry(log, "clawTwo/position");
    }

    clawOne.setNeutralMode(NeutralMode.Brake);
    clawTwo.setNeutralMode(NeutralMode.Brake);
  }

  public void setClawOneSpeed(double speed) {
    clawOne.set(speed);
  }

  public void setClawTwoSpeed(double speed) {
    clawTwo.set(speed);
  }

  public double[] getClawPositions() {
    double[] clawPositions = {clawOne.getSelectedSensorPosition(), clawTwo.getSelectedSensorPosition()};
    clawOnePositionLog.append(clawPositions[0]);
    clawTwoPositionLog.append(clawPositions[1]);
    
    return clawPositions;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
