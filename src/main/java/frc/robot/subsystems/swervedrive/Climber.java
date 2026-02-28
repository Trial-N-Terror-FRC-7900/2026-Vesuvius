package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private SparkFlexConfig ClimberMotorConfig;
    private SparkFlex m_Climber;
    private SparkClosedLoopController m_ClimberPID;
    private RelativeEncoder m_ClimberEncoder;

    public Climber(){
        ClimberMotorConfig = new SparkFlexConfig();
        m_Climber = new SparkFlex(ClimberConstants.ClimberMotorCANID, MotorType.kBrushless);
        m_ClimberPID = m_Climber.getClosedLoopController();
        m_ClimberEncoder = m_Climber.getEncoder();
        
        //Intake Angle Config
        ClimberMotorConfig.smartCurrentLimit(60);

        ClimberMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        ClimberMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(3)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        ClimberMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        m_Climber.configure(ClimberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command climberUp(){
        return this.run(() -> {
            m_ClimberPID.setSetpoint(
                ClimberConstants.ClimberUpPos, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command climberDown(){
        return this.run(() -> {
            m_ClimberPID.setSetpoint(
                ClimberConstants.ClimberDownPos, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command climbedDown(){
        return this.run(() -> {
            m_ClimberPID.setSetpoint(
                (ClimberConstants.ClimberUpPos/2)-25, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command manualDown(){
        return this.run(() -> {
            m_Climber.set(-.6);
        });
    }
}