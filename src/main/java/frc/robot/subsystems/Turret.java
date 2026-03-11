package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase{
    private SparkMaxConfig turretMotorConfig;
    private SparkMaxConfig followerMotorConfig;
    private SparkFlexConfig rotateMotorConfig;
    private SparkMax m_leftTurret;
    private SparkMax m_rightTurret;
    private SparkFlex m_rotationTurret;

    double FlywheelAdjust = 0.0;

    Kicker kicker = new Kicker();

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

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Flywheel Speed: ", TurretConstants.FlywheelSpeed + FlywheelAdjust);
        if(turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().isPresent()){
            turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().ifPresent(turretAngle -> { SmartDashboard.putNumber("Turret Angle Pos", turretAngle.in(Rotations));});
        }
        SmartDashboard.putNumber("Kicker Encoder", kicker.getAbsoluteEncoderAngleSupplier().get().in(Rotations));
        SmartDashboard.putNumber("Turret Motor Connected Encoder", m_rotationTurret.getAbsoluteEncoder().getPosition());
    }

    public Supplier<Angle> getAbsoluteEncoderAngleSupplier(){

        Angle encoderPosition = Rotations.of(m_rotationTurret.getAbsoluteEncoder().getPosition());

    return () -> encoderPosition;

  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param encoder1Supplier    The First Encoder
   * @param encoder2Supplier    The Second Encoder
   * 
   * @return The Easy CRT solver.
   */
  public EasyCRT turretSolver(Supplier<Angle> encoder1Supplier, Supplier<Angle> encoder2Supplier)
  {
    // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.
    var easyCrt = new EasyCRTConfig(encoder1Supplier, encoder2Supplier)
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 1,
                /* driveGearTeeth */ 200,
                /* encoder1Pinion */ 21,
                /* encoder2Pinion */ 19)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Rotations.of(-1.0), Rotations.of(1.0)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(true, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    // you can inspect:
    easyCrt.getUniqueCoverage();          // Optional<Angle> coverage from prime counts and common scale
    easyCrt.coverageSatisfiesRange();     // Does coverage exceed maxMechanismAngle?
    easyCrt.getRecommendedCrtGearPair();  // Suggested pair within constraints

    // Create the solver:
    var easyCrtSolver = new EasyCRT(easyCrt);

    return easyCrtSolver;
  }
    
    public Command adjustFlywheelSpeed(double isInverted){
    return this.runOnce(() -> FlywheelAdjust += (isInverted * TurretConstants.FlywheelAdjust));
    }

    public Command flywheelFeed(){
        return this.run(() -> {
            m_leftTurret.set(TurretConstants.FlywheelSpeed + FlywheelAdjust);
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

    //Limit -0.6754 and 0.2273
}
