// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.Arm;
import frc.tigerlib.XboxControllerIT;

public class ManualArmRotation extends CommandBase {
  /** Creates a new ManualArmRotation. */
  private Arm arm;
  private XboxControllerIT controller;
  public ManualArmRotation(Arm arm, XboxControllerIT controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm =arm;
    this.controller = controller;
    // addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getRightY();
    if (arm.getArmDegrees() <= 0 && speed < 0){
      arm.setRotationSpeed(0);
    }else{
      arm.setRotationSpeed( .55 * speed);
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
