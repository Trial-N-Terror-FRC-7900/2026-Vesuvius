package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.ShooterConstants;

public class Flywheels extends SubsystemBase
{
  private SparkMaxConfig flywheelLeadMotorConfig;
  private SparkMaxConfig flywheelFollowerMotorConfig;
  private SparkMax m_flywheellead;
  private SparkMax m_flywheelfollower;

  public Flywheels()
  {
    flywheelLeadMotorConfig = new SparkMaxConfig();
    flywheelFollowerMotorConfig = new SparkMaxConfig();

    flywheelLeadMotorConfig.closedLoop
                          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0)
                          .i(0)
                          .d(0)
                          .outputRange(0, 1); // Set Min range to 0 because it shouldnt need to spin that way.
    /*
     * Set by https://robotsbythec.org/resources/signal-delay-calculator
     * Idk if 0.5% is good enough or not
     * -Ideal Flywheel RPM
     */
    flywheelLeadMotorConfig.encoder.quadratureAverageDepth(64).quadratureMeasurementPeriod(89);
    flywheelLeadMotorConfig.idleMode(IdleMode.kCoast); // Dont care
    flywheelLeadMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);
                          
    flywheelFollowerMotorConfig.follow(ShooterConstants.ShooterFollowerMotorCANID, true);
    flywheelFollowerMotorConfig.idleMode(IdleMode.kCoast); // Dont care 
    flywheelFollowerMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);

    m_flywheellead = new SparkMax(ShooterConstants.ShooterLeadMotorCANID, MotorType.kBrushless);
    m_flywheelfollower = new SparkMax(ShooterConstants.ShooterFollowerMotorCANID, MotorType.kBrushless);

    m_flywheellead.configure(flywheelLeadMotorConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_flywheelfollower.configure(flywheelFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setSpeed(double Speed){
    return this.run(() -> {
      m_flywheellead.getClosedLoopController().setSetpoint(Speed, ControlType.kVelocity);
    });
  }

  public Command stop(){
    return this.run(() -> m_flywheellead.set(0));
  }

  public boolean atSpeed(){
    if(Math.abs(m_flywheellead.getClosedLoopController().getSetpoint() - m_flywheellead.getEncoder().getVelocity()) < ShooterConstants.VelocityTolerance){
      return true;
    }
    else{
      return false;
    }
  }
}