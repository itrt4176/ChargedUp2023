// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.Arm;

public class HomeArm extends CommandBase {
  private Arm arm;

  /** Creates a new HomeArm. */
  public HomeArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setRotationSpeed(-.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setRotationSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isLimitSwitchPressed();
  }
}
