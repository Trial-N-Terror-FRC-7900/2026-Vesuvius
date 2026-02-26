package frc.robot.subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.HoodConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;


public class Hood extends SubsystemBase
{
  private SparkMaxConfig hoodMotorConfig;
  private SparkMax m_hood;

  public Hood()
  {
    hoodMotorConfig = new SparkMaxConfig();

    hoodMotorConfig.closedLoop
                          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0)
                          .i(0)
                          .d(0)
                          .outputRange(-1, 1);

    hoodMotorConfig.smartCurrentLimit(20);
    hoodMotorConfig.idleMode(IdleMode.kBrake);


    //12 : 100 = 0.12
    //40 : 100 = 0.4
    //12 : 16  = 0.75
    //10 : 170 = 0.05882352941176470588235294117647
    //
    //Total: 0.00211764705882352941176470588235
    //
    hoodMotorConfig.encoder.positionConversionFactor(0.00211764);
    hoodMotorConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition).reverseLimitSwitchType(Type.kNormallyOpen);

    m_hood = new SparkMax(HoodConstants.HoodMotorCANID, MotorType.kBrushless);
  }

  /**
   * Home the Hood to start point
   * 
   * @return If successful
   */
  public boolean setupHood(){

    //Add checking for success.
    startHomingMovement().withTimeout(1).until(homingCondition()).andThen(endHomingMovement());

    return true;

  }

  public Command startHomingMovement(){

    return this.run(() -> {
      m_hood.set(HoodConstants.HomingSpeed);
    });

  }

  public BooleanSupplier homingCondition(){

    //Add Current Limit, Turning Limit
    return () -> m_hood.getReverseLimitSwitch().isPressed();
  }

  public Command endHomingMovement(){
    
    return this.run(() -> {
      m_hood.set(0);
      m_hood.getEncoder().setPosition(0);
    });
  }

  public Command setExitAngle(Angle angleSetpoint){

    return this.run(() ->{

      m_hood.getClosedLoopController().setSetpoint(angleSetpoint.in(Degrees), ControlType.kVelocity);

    });

  }
}