package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberCommand extends CommandBase {
    //creates the m_grabber variable
    private GrabberSubsystem m_grabber;

    public GrabberCommand(GrabberSubsystem grabber) {
        //grabs in the grabber subsystem *ba dum chi*
        m_grabber = grabber;
        addRequirements(m_grabber);
    }

    @Override    
    public void initialize() {
        //grabs your spaghetti
        m_grabber.grab();
    }

    @Override
    public void end(boolean interupted) {
        //The grab has been halted in its tracks
        m_grabber.ungrab();
    }
}
