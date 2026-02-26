package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private SparkMaxConfig IntakeAngleMotorConfig;
    private SparkMaxConfig IntakeWheelsMotorConfig;
    private SparkMax m_IntakeAngle;
    private SparkClosedLoopController m_IntakeAnglePID;
    private RelativeEncoder m_IntakeAngleEncoder;
    private SparkMax m_IntakeWheels;
    private SparkClosedLoopController m_IntakeWheelsPID;
    private RelativeEncoder m_IntakeWheelsEncoder;

    public Intake(){
        IntakeAngleMotorConfig = new SparkMaxConfig();
        IntakeWheelsMotorConfig = new SparkMaxConfig();
        m_IntakeAngle = new SparkMax(IntakeConstants.IntakeAngleMotorCANID, MotorType.kBrushless);
        m_IntakeAnglePID = m_IntakeAngle.getClosedLoopController();
        m_IntakeAngleEncoder = m_IntakeAngle.getEncoder();
        m_IntakeWheels = new SparkMax(IntakeConstants.IntakeWheelsMotorCANID, MotorType.kBrushless);
        m_IntakeWheelsPID = m_IntakeWheels.getClosedLoopController();
        m_IntakeWheelsEncoder = m_IntakeWheels.getEncoder();
        
        //Intake Angle Config
        IntakeAngleMotorConfig.smartCurrentLimit(60);

        IntakeAngleMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        IntakeAngleMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(3)
            .i(0)
            .d(0)
            .outputRange(-0.5, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        IntakeAngleMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);
        
        //Intake Wheels Config
        IntakeWheelsMotorConfig.smartCurrentLimit(60);

        IntakeWheelsMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        IntakeWheelsMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(3)
            .i(0)
            .d(0)
            .outputRange(-0.5, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        IntakeWheelsMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        m_IntakeAngle.configure(IntakeAngleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_IntakeWheels.configure(IntakeWheelsMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command angleUp(){
        return this.run(() -> {
            m_IntakeAnglePID.setSetpoint(
                IntakeConstants.IntakeAngleUp, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command angleDown(){
        return this.run(() -> {
            m_IntakeAnglePID.setSetpoint(
                IntakeConstants.IntakeAngleDown, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }
    
    public Command runWheels(){
        return this.run(() -> {
            m_IntakeWheels.set(IntakeConstants.IntakeWheelSpeed);
        });
    }

    public Command reverseWheels(){
        return this.run(() -> {
            m_IntakeWheels.set(-IntakeConstants.IntakeWheelSpeed);
        });
    }

    public Command stopWheels(){
        return this.run(() -> {
            m_IntakeWheels.stopMotor();
        });
    }
}
