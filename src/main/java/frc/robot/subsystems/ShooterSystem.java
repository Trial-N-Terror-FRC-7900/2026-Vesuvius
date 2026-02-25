package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Flywheels;

public class ShooterSystem extends SubsystemBase
{
  Kicker kicker = new Kicker();
  Turret turret = new Turret();
  Hood hood = new Hood();
  Spindexer spindexer = new Spindexer();
  Flywheels flywheels = new Flywheels();

  public ShooterSystem()
  {
   
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

  /**
   * Spin Up Wheels before Firing
   */
  public Command primeShooter(){
    return this.run(() -> {
        flywheels.setSpeed(ShooterConstants.LaunchSpeed);
        kicker.setSpeed(100);
    });
  }


  /**
   * Spin Down Wheels when not Firing
   */
  public Command idleShooter(){
    return this.run(() -> {
        //Calling this after the wheels are at full speed will cause it to forcibly spin down
        //The control loop can no longer provide negative speed which shouldn't allow it to forcibly spin down
        flywheels.setSpeed(ShooterConstants.IdleSpeed); 
        kicker.setSpeed(0);
        spindexer.StopFeed();
    });
  }


  /**
   * Stop everything
   */
  public Command stopShooter(){
    return this.run(() -> {
        flywheels.stop();
        kicker.stop();
        spindexer.StopFeed();
    });
  }

  public Command shoot(){

    return this.run(() -> {

      if(flywheels.atSpeed() && kicker.atSpeed()){
        spindexer.feedFuel();
      }
      else{
        spindexer.StopFeed();
      }

    });
  }
}