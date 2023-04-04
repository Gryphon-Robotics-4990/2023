package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCenterCubeCommand extends SequentialCommandGroup{
    private final DrivetrainSubsystem m_drivetrain;
    private final ArmSubsystem m_arm;
    private final IntakeSubsystem m_intake;

    public AutoCenterCubeCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, IntakeSubsystem intake){
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_intake = intake;

        addCommands(
            new ParallelRaceGroup(
                new TopCubeCommand(arm),
                new WaitCommand(1.5)
            ),
            //Get meters from field measurements or pathplanner
            new DrivetrainTranslationCommand(m_drivetrain, 1),
            new OuttakeCubeCommand(m_intake)
        );
    }

    //public ParallelRaceGroup driveForwardCommand(double meters){

    //}
}
