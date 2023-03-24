// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.irontigers.robot.Commands.AutoArmExtend;
import frc.irontigers.robot.Commands.MoveArmToAngle;
import frc.irontigers.robot.Subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHigh extends ParallelCommandGroup {
  /** Creates a new PlaceHigh. */
  public PlaceHigh(Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveArmToAngle(arm, 195),
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> arm.getArmDegrees() >= 110.0),
            new AutoArmExtend(arm, 23.6))
    );
  }
}
