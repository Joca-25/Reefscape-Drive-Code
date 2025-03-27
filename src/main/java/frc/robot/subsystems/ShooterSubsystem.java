package frc.robot.subsystems;



//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder; 
    private final AbsoluteEncoder shooterEncoder; 


    private final double kP_shooter = 1,
                        kI_shooter = 0.1,
                        kD_shooter = 0;
    private final double kP_pivot = 1,
                        kI_pivot = 0.1,
                        kD_pivot = 0;
                        
    public ProfiledPIDController shooter_cntlr = new ProfiledPIDController(
        kP_shooter, kI_shooter, kD_shooter,
        new TrapezoidProfile.Constraints(5, 10));
    public ProfiledPIDController pivot_cntrlr = new ProfiledPIDController(
        kP_pivot, kI_pivot, kD_pivot,
        new TrapezoidProfile.Constraints(5, 10));


    double kShooterForward= 0.1;
    double kShooterReverse= 0.025;
    double kPivotForward= 0.025;
    double kPivotReverse = 0.1;

    public ShooterSubsystem(int pivotCANId, int shooterCANId){
        //Motors
        pivotMotor = new SparkMax(pivotCANId, MotorType.kBrushless);
        shooterMotor = new SparkMax(shooterCANId, MotorType.kBrushless);
        //Encoders
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        shooterEncoder = shooterMotor.getAbsoluteEncoder();
        //shooter_cntlr.setTolerance(0.1);
    
    }

    /**  */
    public void shooterOutward() {
        shooterMotor.set(kShooterForward);
    }

    
    public void shooterInward() {
    
        // implicitly require `this`
        shooterMotor.set(kShooterReverse);
    }
    
    public void shooterStop(){   
        shooterMotor.set(0);
    }
    public void shoot(double speed){
        shooterMotor.set(speed);
    }

    public double getShooterPosition(){
        return shooterEncoder.getPosition();
    }
    public void pivotStop(){   
        pivotMotor.set(0);
    }

    public void pivotUp(){   
       
        pivotMotor.set(kPivotForward);
    }

    public void pivotDown(){
        
        pivotMotor.set(kPivotReverse);
    }

    public void pivot(double speed){
        pivotMotor.set(speed);
    }
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }

    
   
}