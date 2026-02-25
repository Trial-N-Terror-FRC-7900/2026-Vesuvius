package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants.TurretConstants;

public class Kicker extends SubsystemBase
{
  private SparkFlexConfig kickerMotorConfig;
  private SparkFlex m_kicker;

  private AbsoluteEncoder kickerConnectedEncoder;

  public Kicker()
  {
    kickerMotorConfig = new SparkFlexConfig();

    m_kicker = new SparkFlex(TurretConstants.turretMotorCANID, MotorType.kBrushless);
    kickerConnectedEncoder = m_kicker.getAbsoluteEncoder();


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

      m_kicker.set(Speed);

    });
  }

    public Command stop(){

    return this.run(() -> {

      m_kicker.set(0);

    });
  }

  public boolean atSpeed(){

    return true;
  }
}