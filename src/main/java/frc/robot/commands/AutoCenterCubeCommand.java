package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCenterCubeCommand extends SequentialCommandGroup{
    private final DrivetrainSubsystem m_drivetrain;
    private final ArmSubsystem m_arm;
    private final IntakeSubsystem m_intake;

    public AutoCenterCubeCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, IntakeSubsystem intake, SendableChooser<Command> chooser){
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_intake = intake;

        addCommands(
            new ParallelRaceGroup(
                chooser.getSelected(),
                new WaitCommand(2)
            ),
            
            //Get meters from field measurements or pathplanner
            new ParallelRaceGroup(
                new DrivetrainTranslationCommand(m_drivetrain, 0.5),
                new WaitCommand(2)
            ),

            new ParallelRaceGroup(
                new OuttakeCubeCommand(m_intake),
                // Wait extra time so we make sure that the cube exits
                new WaitCommand(2)
            )
        );
    }
}
