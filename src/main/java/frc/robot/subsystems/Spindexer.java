package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants.SpindexerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Spindexer extends SubsystemBase
{
  private SparkFlexConfig spindexerMotorConfig;
  private SparkFlex m_spindexer;

  public Spindexer()
  {
    spindexerMotorConfig = new SparkFlexConfig();

    spindexerMotorConfig.closedLoop
                          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0)
                          .i(0)
                          .d(0)
                          .outputRange(-1, 1);

    spindexerMotorConfig.smartCurrentLimit(SpindexerConstants.currentLimit);
    spindexerMotorConfig.idleMode(IdleMode.kBrake); //Dont want to accidentally feed a ball

    m_spindexer = new SparkFlex(SpindexerConstants.SpindexerMotorCANID, MotorType.kBrushless);

    m_spindexer.configure(spindexerMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public Command feedFuel(){

    return this.run(() -> {
      m_spindexer.set(SpindexerConstants.SpindexerForwardSpeed);
    });
  }

  public Command StopFeed(){

    return this.run(() -> {
      m_spindexer.set(0);
    });
  }

  public Command reverseFeedFuel(){

    return this.run(() -> {
      m_spindexer.set(SpindexerConstants.SpindexerReverseSpeed);
    });
  }
}