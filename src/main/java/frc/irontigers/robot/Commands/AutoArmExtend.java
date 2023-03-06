// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Subsystems.Arm;
import frc.tigerlib.XboxControllerIT;

public class AutoArmExtend extends CommandBase {
  /** Creates a new AutoArmExtend. */
  Arm arm;
  XboxControllerIT manualController;
  double destination;
  boolean extending;
  double difference;

  public AutoArmExtend(Arm arm, double destination) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.manualController = manualController;
    this.destination = destination;
    
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    difference = destination - arm.getArmExtensionPosition();
    double speed = MathUtil.clamp(difference * (0.85 / 3.0), -0.85, 0.85);

    if (Math.abs(speed) < 0.07) { 
      return;
    }

    arm.setExtensionSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setExtensionSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return Math.abs(difference) <= 0.1;
  }
}

