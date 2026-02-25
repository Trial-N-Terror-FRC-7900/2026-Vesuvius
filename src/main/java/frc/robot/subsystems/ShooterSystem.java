package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;

public class ShooterSystem
{
  Kicker kicker = new Kicker();
  Turret turret = new Turret();
  Hood hood = new Hood();

  private SparkMaxConfig flywheelLeadMotorConfig;
  private SparkMaxConfig flywheelFollowerMotorConfig;
  private SparkMax m_flywheellead;
  private SparkMax m_flywheelfollower;

  public ShooterSystem()
  {
    flywheelLeadMotorConfig = new SparkMaxConfig();
    flywheelFollowerMotorConfig = new SparkMaxConfig();

    flywheelLeadMotorConfig.closedLoop
                          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0)
                          .i(0)
                          .d(0)
                          .outputRange(-1, 1);
    /*
     * Set by https://robotsbythec.org/resources/signal-delay-calculator
     * Idk if 0.5% is good enough or not
     * -Ideal Flywheel RPM
     */
    flywheelLeadMotorConfig.encoder.quadratureAverageDepth(64).quadratureMeasurementPeriod(89);
    flywheelLeadMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);
                          
    flywheelFollowerMotorConfig.follow(ShooterConstants.ShooterFollowerMotorCANID, true);
    flywheelFollowerMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);

    m_flywheellead = new SparkMax(ShooterConstants.ShooterLeadMotorCANID, MotorType.kBrushless);
    m_flywheelfollower = new SparkMax(ShooterConstants.ShooterFollowerMotorCANID, MotorType.kBrushless);

    m_flywheellead.configure(flywheelLeadMotorConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_flywheelfollower.configure(flywheelFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean setupShooterSystem(){
    // Setup Turret and Hood
    // If either fails return false
    if(turret.setupTurret(turret.getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()) && hood.setupHood()){
      return true;
    }
    else{
      return false;
    }
  }

  public void setWheelSpeed(){

  }

  public void shoot(){
    /*
     * If Shooter at Speed and Kicker at Speed  and angled to target
     *    turn on Spindexer
     * Else
     *    Wait
     */

  }
}