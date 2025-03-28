package frc.robot.Commands;
import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FindTag extends Command {
    private int setPoint;
    private int x;
    private int img_width;
    private final DriveSubsystem m_drive;
    private final int TOLERANCE =10;
        public FindTag(DriveSubsystem drive, int target){
            
            //  setPoint = target;
             m_drive = drive;
            img_width = (int) SmartDashboard.getNumber("Image Width", 640)/2;
             addRequirements(m_drive);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void execute(){
        x=(int) SmartDashboard.getNumber("Camera Center", -1);
        boolean rev = x>img_width;
        x= (x<img_width)? x: x-img_width;
        double speed = 0.1 * ((rev)?-1:1);
        if(x!=-1 && img_width-x<TOLERANCE){
                m_drive.drive(0, 0, speed,false);        
        }
    }

    @Override
    public void end(boolean interrupted){

        // m_drive.stop();
    }

    @Override
    public boolean isFinished(){
        // return m_drive.atSetpoint();
        return x-img_width>TOLERANCE;
    }

    

}