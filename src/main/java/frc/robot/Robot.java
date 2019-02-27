
package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry ts1 = table.getEntry("ts1");
  NetworkTableEntry ts0 = table.getEntry("ts0");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry camTran = table.getEntry("camtran");
  double x = 0;
  double height = 0;
  double width = 0;
  double dist = 0;
  static double heightPixelConstant = 262.9565217391; //conversion factor of pixels to inches
  static double widthPixelConstant = 264; //conversion factor of pixels to inches
  static double heightInchConstant = 5.75; //actual height of object in inches
  static double widthInchConstant = 14; //actual width of object in inches

  


  Joystick _joystick = new Joystick(0);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  DriveTrain drive = new DriveTrain();
  @Override
  public void robotInit() {
    gyro.calibrate();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic(){
    x = tx.getDouble(0.0);
    double Kp = 0.1;
    double min_command = 0.05;
    height = tvert.getDouble(0.0);
    width = thor.getDouble(0.0);
    if(height > 0){
      dist = heightPixelConstant*heightInchConstant/height;
    }else{
      dist = 0.0;
    }
    SmartDashboard.putNumber("limelightHeight", height);
    SmartDashboard.putNumber("limelightWidth", width);
    SmartDashboard.putNumber("Distance", dist);

    if (_joystick.getRawButton(1)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
      double steering_adjust = 0.0;
      double heading_error = -x;
      double max_driveSpeed = 1.0;
      double desired_distance = 20;
      double driving_adjust = 0;
      if (dist < 20){
        driving_adjust = 0.0;
      }else if (dist > 20 && dist < 40){
        driving_adjust = .025*(dist - desired_distance);
        if (driving_adjust > max_driveSpeed){
          driving_adjust = max_driveSpeed;
        }
      }else if(dist > 40 && dist < 60){
        driving_adjust = .05*(dist-desired_distance);
      }else if (dist > 60){
          driving_adjust = max_driveSpeed;
        
      }
      if (x > 1.0){
        steering_adjust = Kp*heading_error - min_command;
      }else if (x < 1.0){
        steering_adjust = Kp*heading_error + min_command;
      }
      drive.turnToTarget((_joystick.getY()*(-1)*driving_adjust), -steering_adjust);
    }else if(_joystick.getRawButton(2)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      double steering_adjust = 0.0;
      double heading_error = -x;
      double max_driveSpeed = 1.0;
      double desired_distance = 20;
      double driving_adjust = 0;
      if (dist < 20){
        driving_adjust = 0.0;
      }else if (dist > 20 && dist < 40){
        driving_adjust = .025*(dist - desired_distance);
        if (driving_adjust > max_driveSpeed){
          driving_adjust = max_driveSpeed;
        }
      }else if(dist > 40 && dist < 60){
        driving_adjust = .05*(dist-desired_distance);
      }else if (dist > 60){
          driving_adjust = max_driveSpeed; 
      }
      if (x > 1.0){
        steering_adjust = Kp*heading_error - min_command;
      }else if (x < 1.0){
        steering_adjust = Kp*heading_error + min_command;
      }
      drive.turnToTarget((_joystick.getY()*(-1)*driving_adjust), -steering_adjust);
    }else{
      drive.run(_joystick.getY(), _joystick.getZ());
    }
  }

  @Override
  public void autonomousInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  @Override
  public void teleopPeriodic() {
    


  }

  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }


  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
