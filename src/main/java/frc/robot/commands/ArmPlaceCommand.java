package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMeasurements;
import frc.robot.Constants.Units;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPlaceCommand extends CommandBase{
    private final ArmSubsystem m_armsubsystem;
    private double finalPos;

    public ArmPlaceCommand(ArmSubsystem armsubsystem) {
        m_armsubsystem = armsubsystem;
        addRequirements(armsubsystem);

        finalPos = 90.0;
    }

    @Override
    public void initialize() {
        m_armsubsystem.moveToPosition(finalPos*Units.DEGREE.to(Units.REVOLUTION)*RobotMeasurements.ARM_MOTION_REDUCTION);
    }

}
