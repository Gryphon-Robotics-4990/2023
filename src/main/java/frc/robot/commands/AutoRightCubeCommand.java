package frc.robot.commands;
import java.util.Map;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRightCubeCommand extends SequentialCommandGroup{
    private final DrivetrainSubsystem m_drivetrain;
    private final ArmSubsystem m_arm;
    private final IntakeSubsystem m_intake;

    public AutoRightCubeCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, IntakeSubsystem intake, SendableChooser<Command> chooser){
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_intake = intake;

        addCommands(
            new ParallelRaceGroup(
                chooser.getSelected(),
                new WaitCommand(2)
            ),
            //new InstantCommand(() -> chooser.getSelected().raceWith(new WaitCommand(2)).schedule(), m_arm),
            //Get meters from field measurements or pathplanner
            new ParallelRaceGroup(
                new DrivetrainTranslationCommand(m_drivetrain, 0.5),
                new WaitCommand(2)
            ),

            new ParallelRaceGroup(
                new OuttakeCubeCommand(m_intake),
                new WaitCommand(2)
            ),

            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new DrivetrainTranslationCommand(m_drivetrain, -4.2),
                    new WaitCommand(4)
                ),
                new ArmNeutralCommand(m_arm)
            )
        );
    }

    //public ParallelRaceGroup driveForwardCommand(double meters){

    //}
}
