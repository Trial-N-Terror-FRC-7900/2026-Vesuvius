package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConstants;

public class Hood extends SubsystemBase{
    private SparkMaxConfig HoodMotorConfig;
    private SparkMax m_Hood;
    private SparkClosedLoopController m_HoodPID;
    private RelativeEncoder m_HoodEncoder;
    private InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    public Hood(){
        HoodMotorConfig = new SparkMaxConfig();
        m_Hood = new SparkMax(TurretConstants.HoodMotorCanID, MotorType.kBrushless);
        m_HoodPID = m_Hood.getClosedLoopController();
        m_HoodEncoder = m_Hood.getEncoder();
        
        //Intake Angle Config
        HoodMotorConfig.smartCurrentLimit(20);

        HoodMotorConfig.encoder
            //12 : 100 = 0.12
            //40 : 100 = 0.4
            //12 : 16  = 0.75
            //10 : 170 = 0.05882352941176470588235294117647
            //
            //Total: 0.00211764705882352941176470588235
            //
            // This conversion factor should convert the rotation of the Motor to the angle the ball is launching from
            // It is not required
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        HoodMotorConfig.softLimit.reverseSoftLimit(TurretConstants.hoodLimitReverse).reverseSoftLimitEnabled(true);
        HoodMotorConfig.softLimit.forwardSoftLimit(TurretConstants.hoodLimitFoward).forwardSoftLimitEnabled(true);

        HoodMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        HoodMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        m_Hood.configure(HoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Calibration of Distance to Hood Angle
        distanceToHoodAngle.put(/*DISTANCE*/.72, /*ANGLE*/0.0);   //Front Against Hub
        //distanceToHoodAngle.put(/*DISTANCE*/2.14, /*ANGLE*/14.5); //At Tower
        //distanceToHoodAngle.put(/*DISTANCE*/3.12, /*ANGLE*/13.3); //At Trench
        distanceToHoodAngle.put(/*DISTANCE*/3.45, /*ANGLE*/7.15); //Climb Shooting
        distanceToHoodAngle.put(/*DISTANCE*/3.68, /*ANGLE*/12.0); //Backwards at Depot
        distanceToHoodAngle.put(/*DISTANCE*/5.26, /*ANGLE*/28.5); //Corner
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Hood Position: ", m_HoodEncoder.getPosition());
    }

    public Command manualUp(){
        return this.run(() -> {
            m_Hood.set(TurretConstants.HoodSpeed);
        });
    }

    public Command hoodUp(){
        return this.run(() -> {
            m_HoodPID.setSetpoint(
                TurretConstants.HoodUpPos, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command manualDown(){
        return this.run(() -> {
            m_Hood.set(-TurretConstants.HoodSpeed);
        });
    }

    public Command hoodDown(){
        return this.run(() -> {
            m_HoodPID.setSetpoint(
                0, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command hoodDownCheck() {
        return hoodDown().until(isHoodPos(0));
    }

    public BooleanSupplier isHoodPos(double setpoint){
        return () -> Math.abs(m_HoodEncoder.getPosition() - setpoint) <= TurretConstants.hoodTolerance;
    }

    public Command stop(){
        return this.run(() -> {
            m_Hood.stopMotor();
        });
    }

    public void setHoodfromDistance(double distance){
            m_HoodPID.setSetpoint(
            distanceToHoodAngle.get(distance),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0);
            SmartDashboard.putNumber("Distance at hood", distance);
            SmartDashboard.putNumber("Angle Calc from Distance", distanceToHoodAngle.get(distance));
    }
}