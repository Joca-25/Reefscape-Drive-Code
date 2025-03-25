package frc.robot.Commands;
import java.lang.annotation.Target;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUp extends Command {
    private int setPoint;


    private final ElevatorSubsystem m_ElevatorSubsystem;

        public ElevatorUp(ElevatorSubsystem elevator, int target){
            
             setPoint = target;
             m_ElevatorSubsystem = elevator;

             addRequirements(m_ElevatorSubsystem);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void execute(){
       
        double speed = m_ElevatorSubsystem.elevator_cntlr.calculate(m_ElevatorSubsystem.getDistance(), setPoint);
        m_ElevatorSubsystem.forward();
    }

    @Override
    public void end(boolean interrupted){

        m_ElevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return m_ElevatorSubsystem.elevator_cntlr.atSetpoint();
    }

    

}