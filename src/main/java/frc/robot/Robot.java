
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
  


  Joystick _joystick = new Joystick(0);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  DriveTrain drive = new DriveTrain();
  @Override
  public void robotInit() {
    gyro.calibrate();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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
    x = tx.getDouble(0.0);
    double Kp = 0.1;
    double min_command = 0.05;

    if (_joystick.getRawButton(1)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
      double steering_adjust = 0.0;
      double heading_error = -x;
      if (x > 1.0){
        steering_adjust = Kp*heading_error - min_command;
      }else if (x < 1.0){
        steering_adjust = Kp*heading_error + min_command;
      }
      drive.turnToTarget((_joystick.getY()/1.5)*-1, -steering_adjust);
    }else if(_joystick.getRawButton(2)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      double steering_adjust = 0.0;
      double heading_error = -x;
      if (x > 1.0){
        steering_adjust = Kp*heading_error - min_command;
      }else if (x < 1.0){
        steering_adjust = Kp*heading_error + min_command;
      }
      drive.turnToTarget((_joystick.getY()/1.5)*-1, -steering_adjust);
    }else{
      drive.run(_joystick.getY(), _joystick.getZ());
    }


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
