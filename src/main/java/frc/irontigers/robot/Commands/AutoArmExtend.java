// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Subsystems.Arm;
import frc.tigerlib.XboxControllerIT;

public class AutoArmExtend extends CommandBase {
  /** Creates a new AutoArmExtend. */
  Arm arm;
  XboxControllerIT manualController;
  int destination;
  boolean extending;

  public AutoArmExtend(Arm arm, XboxControllerIT manualController, int destination) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.manualController = manualController;
    this.destination = destination;
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (arm.getArmExtensionPosition() < destination) {
      extending = false;
    } else if (arm.getArmExtensionPosition() > destination) {
      extending = true;
    } else {
      extending = true; //make an edge case to end it instantly?
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extending == true) {
      arm.setExtensionSpeed(0.35);
    } else if (extending == false) {
      arm.setExtensionSpeed(-0.35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arm.getArmExtensionPosition() > Constants.ArmVals.ARM_EXTENDER_LOWER_LIMIT) {
      return true;
    } else if (arm.getArmExtensionPosition() < Constants.ArmVals.ARM_EXTENDER_UPPER_LIMIT) {
    return true;
    } else if (extending == true) {
      if (arm.getArmExtensionPosition() < destination) {
        return true;
      }
    } else if (extending == false) {
      if (arm.getArmExtensionPosition() > destination) {
        return true;
      }
    } else {
     return false;
    }

    return false;
  }
}
