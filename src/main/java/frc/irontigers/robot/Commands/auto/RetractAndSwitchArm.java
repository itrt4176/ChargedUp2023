// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.irontigers.robot.Commands.AutoArmExtend;
import frc.irontigers.robot.Commands.MoveArmToAngle;
import frc.irontigers.robot.Subsystems.ArmExtender;
import frc.irontigers.robot.Subsystems.ArmRotator;
import frc.irontigers.robot.Subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractAndSwitchArm extends ParallelCommandGroup {
  /** Creates a new RetractAndSwitchArm. */
  public RetractAndSwitchArm(ArmExtender armExtender, ArmRotator armRotator, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> armExtender.getArmExtensionPosition() <= 23.6 - 12.0),
            new InstantCommand(claw::close)),
        new AutoArmExtend(armExtender, 0),
        new MoveArmToAngle(armRotator, 2.5)
    );
  }
}
