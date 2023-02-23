// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Subsystems.Arm.*;
import frc.irontigers.robot.Subsystems.Arm;

import frc.tigerlib.XboxControllerIT;

public class ArmManualLengthAdjustment extends CommandBase {
  /** Creates a new ArmManualLengthAdjustment. */
  Arm arm;
  XboxControllerIT manualController;
  public ArmManualLengthAdjustment(Arm arm, XboxControllerIT manualController) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.manualController = manualController;
    

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((manualController.getLeftTriggerAxis() - manualController.getRightTriggerAxis()) < 0) { //if extending
      if (arm.getArmExtender().getSelectedSensorPosition() > Constants.ArmVals.ARM_EXTENDER_UPPER_LIMIT) {
        arm.setExtensionSpeed(0.35*(manualController.getLeftTriggerAxis() - manualController.getRightTriggerAxis()));
      } else {
        arm.setExtensionSpeed(0);
      }
    } else if ((manualController.getLeftTriggerAxis() - manualController.getRightTriggerAxis()) > 0) { //if retracting
      if (arm.getArmExtender().getSelectedSensorPosition() < Constants.ArmVals.ARM_EXTENDER_LOWER_LIMIT) {
        arm.setExtensionSpeed(0.35*(manualController.getLeftTriggerAxis() - manualController.getRightTriggerAxis()));
      } else {
        arm.setExtensionSpeed(0);
      }
    } else {
      arm.setExtensionSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
