// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Commands.AutoArmExtend;
import frc.irontigers.robot.Commands.MoveArmToAngle;
import frc.irontigers.robot.Commands.AutoSimpleDrive;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.tigerlib.XboxControllerIT;
import frc.tigerlib.command.DifferentialJoystickDrive;
import frc.tigerlib.command.ToggleInversionCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..

  private final XboxControllerIT mainController = new XboxControllerIT(0);
  private final XboxControllerIT clawController = new XboxControllerIT(1);
  
  private final DriveSystem driveSystem = new DriveSystem();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  private final DifferentialJoystickDrive joystickDrive = new DifferentialJoystickDrive(driveSystem, mainController);
  private final ToggleInversionCommand toggleInversion = new ToggleInversionCommand(driveSystem);

  private final Trigger gearShiftUp = mainController.rightBumper();
  private final Trigger gearShiftDown = mainController.leftBumper();

  private final ArmManualLengthAdjustment armLengthAdjustment = new ArmManualLengthAdjustment(arm, mainController);
  private final MoveArmToAngle armSetAngle90 = new MoveArmToAngle(arm, 90);
  private final MoveArmToAngle armSetAngle180 = new MoveArmToAngle(arm, 180);

  private final AutoArmExtend autoFullRetract = new AutoArmExtend(arm, 0);
  private final AutoArmExtend autoHalfExtend = new AutoArmExtend(arm, 20.75/2.0);
  private final AutoArmExtend autoFullExtend = new AutoArmExtend(arm, 20.75);

  private final Trigger toggleInvertButton = mainController.b();

  private final Trigger armRotationForward = mainController.y();
  private final Trigger armRotationBackward = mainController.a();
 
  // private final Trigger  armSet90 = mainController.povLeft();
  // private final Trigger armSet180 = mainController.povRight();

  private final Trigger fullRetract = mainController.povLeft();
  private final Trigger halfExtend = mainController.povUp();
  private final Trigger fullExtend = mainController.povRight();

  // private final Trigger clawIn = mainController.povUp();
  // private final Trigger clawOut = mainController.povDown();

 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button binding.
    configureButtonBindings();
    mainController.setDeadzone(.2);
    driveSystem.setDefaultCommand(joystickDrive);
    arm.setDefaultCommand(armLengthAdjustment);
    //Maybe adjust once arm rotation is coded.
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxControllerIT}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    gearShiftUp.onTrue(new InstantCommand(() -> driveSystem.shiftUp()));
    gearShiftDown.onTrue(new InstantCommand(() -> driveSystem.shiftDown()));

    toggleInvertButton.onTrue(toggleInversion);

    armRotationForward.onTrue(new InstantCommand(() -> arm.setRotationSpeed(.15)));
    armRotationForward.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0)));

    armRotationBackward.onTrue(new InstantCommand(() -> arm.setRotationSpeed(-.15)));
    armRotationBackward.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0)));
    // armStopRotation.onTrue(new InstantCommand(() -> arm.setRotationSpeed(0.0)));

    // armSet90.onTrue(armSetAngle90);
    // armSet180.onTrue(armSetAngle180);

    // clawIn.whileTrue(new StartEndCommand(
    //   () -> claw.setClawOneSpeed(0.75), 
    //   () -> claw.setClawOneSpeed(0)));
    // clawIn.onFalse(new InstantCommand(() -> claw.setClawOneSpeed(0)));

    // clawIn.onTrue( new InstantCommand(() -> claw.setClawStateTrue()));
    // clawOut.onTrue(new InstantCommand(() -> claw.setClawStateFalse()));

    // clawOut.whileTrue(new StartEndCommand(
    //   () -> claw.setClawOneSpeed(-0.75), 
    //   () -> claw.setClawOneSpeed(0)));
    // // clawOut.onFalse(new InstantCommand(() -> claw.setClawOneSpeed(0)));

    fullRetract.onTrue(autoFullRetract);
    halfExtend.onTrue(autoHalfExtend);
    fullExtend.onTrue(autoFullExtend);

  }
 
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup drive = new SequentialCommandGroup(
      new AutoSimpleDrive(driveSystem),
      new WaitUntilCommand(14.5));
      
    return drive;
  }
}
