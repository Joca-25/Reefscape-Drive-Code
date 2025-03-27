package frc.robot.Commands;
import java.lang.annotation.Target;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotShooterDown extends Command{

    private int setPoint;


    private final ShooterSubsystem m_PivotShooterSubsystem;

    public PivotShooterDown(ShooterSubsystem pivotDown, int target){
            
             setPoint = target;
             m_PivotShooterSubsystem = pivotDown;

             addRequirements(m_PivotShooterSubsystem);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void execute(){
       
        double speed = m_PivotShooterSubsystem.pivot_cntrlr.calculate(m_PivotShooterSubsystem.getShooterPosition(), setPoint);
        m_PivotShooterSubsystem.pivot(-.05);
    }

    @Override
    public void end(boolean interrupted){

        m_PivotShooterSubsystem.pivotStop();
    }

    @Override
    public boolean isFinished(){
        return m_PivotShooterSubsystem.pivot_cntrlr.atGoal();
    }
}
