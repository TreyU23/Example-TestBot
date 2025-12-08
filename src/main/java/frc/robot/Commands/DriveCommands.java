package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriverConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class DriveCommands {
    public static final SwerveRequest.FieldCentric DRIVE_REQUEST = new SwerveRequest.FieldCentric()
        .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DRIVE_DEADBAND)
        .withRotationalDeadband(DriverConstants.MAX_ANGULAR_RATE * DriverConstants.ROTATION_DEADBAND)
        .withDriveRequestType(DriveRequestType.Velocity);

    // These slew rates are updated via updateSlew in SSM.java based on the commanded state
    public static SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
    public static SlewRateLimiter m_slewY = new SlewRateLimiter(12.0);
    public static SlewRateLimiter m_slewRot = new SlewRateLimiter(36.0);
    
    public static void updateSlew(double x, double y, double r) {
        m_slewX = new SlewRateLimiter(x, -x, m_slewX.lastValue());
        m_slewY = new SlewRateLimiter(y, -y, m_slewY.lastValue());
        m_slewRot = new SlewRateLimiter(r, -r, m_slewRot.lastValue());
    }
        
    public static Command fieldOrientedDrive(
                    CommandSwerveDrivetrain drivetrain,
                    DoubleSupplier forwardBack,
                    DoubleSupplier leftRight,
                    DoubleSupplier rotation) {

                return drivetrain.applyRequest(() ->{
                  
                    double transCurveAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(),0.9,0.1);
                    SmartDashboard.putNumber("TransCurveAdj", transCurveAdjustment);
                    double rotCurveAdjustment = adjustRotCurve(rotation.getAsDouble(), 0.9, 0.1);
                    SmartDashboard.putNumber("RotCurveAdj", rotCurveAdjustment);
                    
                    return DRIVE_REQUEST
                        .withVelocityX(m_slewX.calculate(-forwardBack.getAsDouble() * DriverConstants.MAX_SPEED*transCurveAdjustment))
                        .withVelocityY(m_slewY.calculate(-leftRight.getAsDouble() * DriverConstants.MAX_SPEED*transCurveAdjustment))
                        .withRotationalRate(m_slewRot.calculate(-rotation.getAsDouble() * DriverConstants.MAX_ANGULAR_RATE*rotCurveAdjustment)); 
                });
    }

    public static Command resetFieldOrientation(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.runOnce(drivetrain::seedFieldCentric);
    }

    private static double adjustInputCurve(double x, double y, double a, double b){
        double mag = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        if(mag > 1.0) mag = 1.0;
        if(mag < 0.0) mag = 0.0;
        mag = a*Math.pow(mag, 3)+b*mag;
        return mag;
    }

    public static double adjustRotCurve(double r, double a, double b){
        return Math.abs(a*Math.pow(r, 3)+b*r);
    }
}