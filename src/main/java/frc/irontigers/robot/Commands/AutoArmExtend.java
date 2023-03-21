// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Subsystems.ArmExtender;
import frc.irontigers.robot.Subsystems.ArmRotator;
import frc.tigerlib.XboxControllerIT;

public class AutoArmExtend extends CommandBase {
  /** Creates a new AutoArmExtend. */
  ArmExtender armExtender;
  XboxControllerIT manualController;
  double destination;
  boolean extending;
  double difference;

  public AutoArmExtend(ArmExtender armExtender, double destination) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armExtender = armExtender;
    this.manualController = manualController;
    this.destination = destination;
    
    
    addRequirements(armExtender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    difference = destination - armExtender.getArmExtensionPosition();
    double speed = MathUtil.clamp(difference * (0.9 / 3.0), -0.9, 0.9);

    if (Math.abs(speed) < 0.14) { 
      return;
    }

    armExtender.setExtensionSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtender.setExtensionSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return Math.abs(difference) <= 0.1;
  }
}

