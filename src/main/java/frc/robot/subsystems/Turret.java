package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.TurretConstants;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;


public class Turret
{
  private SparkFlexConfig turretMotorConfig;
  private SparkFlex m_turret;

  private AbsoluteEncoder directEncoder;

  public Turret()
  {
    turretMotorConfig = new SparkFlexConfig();

    m_turret = new SparkFlex(TurretConstants.turretMotorCANID, MotorType.kBrushless);
    directEncoder = m_turret.getAbsoluteEncoder();
  }

  public Supplier<Angle> getAbsoluteEncoderAngleSupplier(){

    Angle encoderPosition = Rotations.of(directEncoder.getPosition());

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
                /* commonRatio (mech:drive) */ 9.52380952,
                /* driveGearTeeth */ 21,
                /* encoder1Pinion */ 19,
                /* encoder2Pinion */ 21)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Rotations.of(-1.0), Rotations.of(1.0)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(false, false)
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

  public boolean setupTurret(Supplier<Angle> encoder1Supplier, Supplier<Angle> encoder2Supplier){ // Setup Turret and Return True if Successful

    if(turretSolver(encoder1Supplier, encoder2Supplier).getAngleOptional().isPresent()){
      turretSolver(encoder1Supplier, encoder2Supplier).getAngleOptional().ifPresent(turretAngle -> { m_turret.getEncoder().setPosition(turretAngle.in(Rotations));});
      return true;
    }
    else{
      return false;
    }
  }
}