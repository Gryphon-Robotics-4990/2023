package frc.robot.vision;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPositioningCommand extends SequentialCommandGroup {
    private final AutoPositioningController m_controller;

    public AutoPositioningCommand(AutoPositioningController controller){
        m_controller = controller;

        addCommands(
        m_controller.getAutonomousCommand()
        // Isaac's iterative positioning command
        // Days Since Isaac started working on his command: 37
        );
    }
}
