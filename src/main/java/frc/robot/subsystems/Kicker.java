package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.KickerConstants;

public class Kicker extends SubsystemBase
{
  private SparkFlexConfig kickerMotorConfig;
  private SparkFlex m_kicker;

  private AbsoluteEncoder kickerConnectedEncoder;

  public Kicker()
  {
    kickerMotorConfig = new SparkFlexConfig();

    m_kicker = new SparkFlex(KickerConstants.KickerMotorCANID, MotorType.kBrushless);
    kickerConnectedEncoder = m_kicker.getAbsoluteEncoder();

    kickerMotorConfig.closedLoop
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
    kickerMotorConfig.encoder.quadratureAverageDepth(64).quadratureMeasurementPeriod(89);
    kickerMotorConfig.idleMode(IdleMode.kCoast); //Dont care
    kickerMotorConfig.smartCurrentLimit(KickerConstants.currentLimit);

  }

  /**
   * Gets the Supplier Position of the Encoder connected to the Kicker Motor
   * 
   * @return The Position of the Connected Encoder
   */
  public Supplier<Angle> getAbsoluteEncoderAngleSupplier(){

    Angle encoderPosition = Rotations.of(kickerConnectedEncoder.getPosition());

    return () -> encoderPosition;

  }

  public Command setSpeed(double Speed){

    return this.run(() -> {

      m_kicker.getClosedLoopController().setSetpoint(Speed, ControlType.kVelocity);

    });
  }

    public Command stop(){

    return this.run(() -> {

      m_kicker.set(0);

    });
  }

  public boolean atSpeed(){
    if(Math.abs(m_kicker.getClosedLoopController().getSetpoint() - m_kicker.getEncoder().getVelocity()) < KickerConstants.VelocityTolerance){
      return true;
    }
    else{
      return false;
    }
  }
}