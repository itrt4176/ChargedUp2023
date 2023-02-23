// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    arm.setExtensionSpeed(0.75*(manualController.getLeftTriggerAxis() - manualController.getRightTriggerAxis()));
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
