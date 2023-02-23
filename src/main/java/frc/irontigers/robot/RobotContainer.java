// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.tigerlib.XboxControllerIT;
import frc.tigerlib.command.DifferentialJoystickDrive;
import frc.tigerlib.command.ToggleInversionCommand;
import frc.irontigers.robot.Subsystems.Claw;



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
  // private final Claw claw = new Claw();

  private final DifferentialJoystickDrive joystickDrive = new DifferentialJoystickDrive(driveSystem, mainController);
  private final ToggleInversionCommand toggleInversion = new ToggleInversionCommand(driveSystem);

  private final Trigger gearShiftUp = mainController.rightBumper();
  private final Trigger gearShiftDown = mainController.leftBumper();

  private final ArmManualLengthAdjustment armLengthAdjustment = new ArmManualLengthAdjustment(arm, mainController);

  private final Trigger toggleInvertButton = mainController.b();

  private final Trigger armRotationForward = mainController.y();
  private final Trigger armRotationBackward = mainController.a();
  private final Trigger armStopRotation = mainController.x();

  private final Trigger clawOneForward = clawController.b();
  private final Trigger clawOneBackward = clawController.x();
  private final Trigger clawTwoForward = clawController.y();
  private final Trigger clawTwoBackward = clawController.a(); 
  private final Trigger clawsStop = clawController.rightBumper(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button binding.
    configureButtonBindings();
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

    /*clawOneForward.whileTrue(new InstantCommand(() -> claw.setClawOneSpeed(.05)).finallyDo((end) -> claw.setClawOneSpeed(0.0)));
    clawOneBackward.whileTrue(new InstantCommand(() -> claw.setClawOneSpeed(-.05)).finallyDo((end) -> claw.setClawOneSpeed(0.0)));
    clawTwoForward.whileTrue(new InstantCommand(() -> claw.setClawTwoSpeed(.05)).finallyDo((end) -> claw.setClawTwoSpeed(0.0)));
    clawTwoBackward.whileTrue(new InstantCommand(() -> claw.setClawTwoSpeed(-.05)).finallyDo((end) -> claw.setClawTwoSpeed(0.0)));
    clawsStop.onTrue(
      new InstantCommand(() ->{
        claw.setClawOneSpeed(0);
        claw.setClawTwoSpeed(0);
      })
    );*/
  }
 
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
