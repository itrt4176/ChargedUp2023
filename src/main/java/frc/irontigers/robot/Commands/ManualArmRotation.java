// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.irontigers.robot.Subsystems.ArmRotator;
import frc.tigerlib.XboxControllerIT;

public class ManualArmRotation extends CommandBase {
  /** Creates a new ManualArmRotation. */
  private ArmRotator armRotator;
  private XboxControllerIT controller;
  private CommandJoystick joystick;
  public ManualArmRotation(ArmRotator armRotator, XboxControllerIT controller /*CommandJoystick joystick*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armRotator = armRotator;
    this.controller = controller;
    //this.joystick = joystick;
    addRequirements(armRotator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    double speed = controller.getRightY();
    if (armRotator.getArmDegrees() <= 0 && speed < 0){
      armRotator.setRotationSpeed(0);
    }else{
      armRotator.setRotationSpeed( .55 * speed);
    }
  }
  
  /*
  public void execute() {
    double speed = joystick.getY();
    if (speed > 0) {
      speed = MathUtil.clamp(speed - .5, 0, .5);
    }
    else if (speed < 0) {
      speed = MathUtil.clamp(speed + .5, -.5, 0);
    }

    if (arm.getArmDegrees() <= 0 && speed < 0){
      arm.setRotationSpeed(0);
    }else{
      arm.setRotationSpeed(speed);
    }
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
