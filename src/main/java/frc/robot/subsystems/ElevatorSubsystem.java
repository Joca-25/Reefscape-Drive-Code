package frc.robot.subsystems;

//package edu.wpi.first.wpilibj.examples.hatchbotinlined.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

/**
 * A hatch mechanism actuated by a single
 * {@link edu.wpi.first.wpilibj.DoubleSolenoid}.
 */
public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax ElevatorMotor;
  private final RelativeEncoder ElevatorEncoder;
  private final double kP_elevator = 1,
      kI_elevator = 0.1,
      kD_elevator = 0;
  public ProfiledPIDController elevator_cntlr = new ProfiledPIDController(
      kP_elevator, kI_elevator, kD_elevator,
      new TrapezoidProfile.Constraints(5, 10));

  // private final ProfiledPIDController m_controller =
  // new ProfiledPIDController(
  // DriveConstants.kTurnP,
  // 0,
  // 0,
  // new TrapezoidProfile.Constraints(
  // DriveConstants.kMaxTurnRateDegPerS,
  // DriveConstants.kMaxTurnAccelerationDegPerSSquared));
  // private final SimpleMotorFeedforward m_feedforward =
  // new SimpleMotorFeedforward(
  // DriveConstants.ksVolts,
  // DriveConstants.kvVoltSecondsPerDegree,
  // DriveConstants.kaVoltSecondsSquaredPerDegree);

  public ElevatorSubsystem(int leftCanId) {

    ElevatorMotor = new SparkMax(leftCanId, MotorType.kBrushless);
    ElevatorEncoder = ElevatorMotor.getEncoder();
    ElevatorEncoder.setPosition(0);
    elevator_cntlr.reset(0);
    // ElevatorEncoder.setPositionConversionFactor()
  }

  public void forward() {
    ElevatorMotor.set(.3);

  }

  public void Backward() {
    ElevatorMotor.set(-.3);
  }

  public void stop() {
    ElevatorMotor.set(0);
  }

  public void elevate(double speed) {
    ElevatorMotor.set(speed);
  }

  public double getElevatorEncoder() {

    return ElevatorEncoder.getVelocity();
  }

  public double getDistance() {
    return ElevatorEncoder.getPosition();

  }
  public void setPID(double kP,double kI, double kD){
    elevator_cntlr.setPID(kP, kI, kD);

  }

}
