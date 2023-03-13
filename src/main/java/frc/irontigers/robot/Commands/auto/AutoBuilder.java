package frc.irontigers.robot.Commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;

public class AutoBuilder {
    private ShuffleboardTab autoTab;
    private DriveSystem drive;
    private Arm arm;
    private Claw claw;

    private SendableChooser<String> preload;
    private SendableChooser<Integer> startPosition;
    private SendableChooser<String> endPosition;

    private ComplexWidget preloadEntry;
    private NetworkTableEntry startPosEntry;
    private NetworkTableEntry endPosEntry;

    private Map<String, List<Integer>> startingBayMap;
    private Map<Integer, List<String>> endingPosMap;

    public AutoBuilder(DriveSystem drive, Arm arm, Claw claw) {
        this.drive = drive;
        this.arm = arm;
        this.claw = claw;

        autoTab = Shuffleboard.getTab("Auto");

        preload = new SendableChooser<>();
        preload.addOption("Cone", "Cone");
        preload.addOption("Cube", "Cube");

        // preloadEntry = autoTab.add(preload).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0);

        // preloadEntry.

        startingBayMap = new HashMap<>();
        startingBayMap.put(
                "Cone",
                List.of(
                        1,
                        3,
                        4,
                        6,
                        7,
                        9));
        startingBayMap.put(
                "Cube",
                List.of(
                        2,
                        5,
                        8));

        endingPosMap = new HashMap<>();
        endingPosMap.put(
                1,
                List.of(
                        "B2"));
        endingPosMap.put(
                4,
                List.of(
                        "CS"));
        endingPosMap.put(
                5,
                List.of(
                        "CS"));
        endingPosMap.put(
                6,
                List.of(
                        "CS"));
    }
}
