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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase{
    private SparkMaxConfig turretMotorConfig;
    private SparkMaxConfig followerMotorConfig;
    private SparkFlexConfig rotateMotorConfig;
    private SparkMax m_leftTurret;
    private SparkMax m_rightTurret;
    private SparkFlex m_rotationTurret;

    public Turret()
    {
        turretMotorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();
        rotateMotorConfig = new SparkFlexConfig();
        m_leftTurret = new SparkMax(TurretConstants.LeftFlywheelMotorCANID, MotorType.kBrushless);
        m_rightTurret = new SparkMax(TurretConstants.RightFlywheelMotorCANID, MotorType.kBrushless);
        m_rotationTurret = new SparkFlex(TurretConstants.RotationMotorCANID, MotorType.kBrushless);

        followerMotorConfig.smartCurrentLimit(60);

        turretMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        turretMotorConfig.closedLoop
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

        turretMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        followerMotorConfig.follow(TurretConstants.LeftFlywheelMotorCANID, true);
        followerMotorConfig.idleMode(IdleMode.kCoast); // Dont care 
        followerMotorConfig.smartCurrentLimit(60);

        rotateMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        rotateMotorConfig.closedLoop
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

        rotateMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        rotateMotorConfig.idleMode(IdleMode.kBrake);

        m_leftTurret.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightTurret.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rotationTurret.configure(rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public Command flywheelFeed(){
        return this.run(() -> {
            m_leftTurret.set(TurretConstants.FlywheelSpeed);
        });
    }

    public Command flywheelStop(){
        return this.run(() -> {
            m_leftTurret.stopMotor();
        });
    }
    
    public Command rotationLeft(){
        return this.run(() -> {
            m_rotationTurret.set(-TurretConstants.RotationSpeed);
        });
    }

    public Command rotationRight(){
        return this.run(() -> {
            m_rotationTurret.set(TurretConstants.RotationSpeed);
        });
    }

    public Command rotationStop(){
        return this.run(() -> {
            m_rotationTurret.stopMotor();
        });
    }
}
