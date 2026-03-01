package frc.robot.subsystems;

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
import frc.robot.Constants.SpindexerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    private SparkFlexConfig SpindexerMotorConfig;
    private SparkFlex m_Spindexer;
    private SparkClosedLoopController m_SpindexerPID;
    private RelativeEncoder m_SpindexerEncoder;

    public Spindexer(){
        SpindexerMotorConfig = new SparkFlexConfig();
        m_Spindexer = new SparkFlex(SpindexerConstants.SpindexerMotorCANID, MotorType.kBrushless);
        m_SpindexerPID = m_Spindexer.getClosedLoopController();
        m_SpindexerEncoder = m_Spindexer.getEncoder();
        
        //Intake Angle Config
        SpindexerMotorConfig.smartCurrentLimit(70);

        SpindexerMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        SpindexerMotorConfig.closedLoop
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

        SpindexerMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        m_Spindexer.configure(SpindexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command spindexerFeed(){
        return this.run(() -> {
            m_Spindexer.set(SpindexerConstants.SpindexSpeed);
        });
    }

    public Command spindexerStop(){
        return this.run(() -> {
            m_Spindexer.set(0);
        });
    }
}
