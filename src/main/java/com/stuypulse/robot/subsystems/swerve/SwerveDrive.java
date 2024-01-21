package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.SwerveDrive.BackLeft;
import com.stuypulse.robot.constants.Settings.SwerveDrive.BackRight;
import com.stuypulse.robot.constants.Settings.SwerveDrive.FrontLeft;
import com.stuypulse.robot.constants.Settings.SwerveDrive.FrontRight;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.stuylib.math.Vector2D;

import static com.stuypulse.robot.constants.Ports.SwerveDrive.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {
    
    private static SwerveDrive instance;

    static {
        if (RobotBase.isReal()) {
            instance = new SwerveDrive(
                new AbstractModule[] {
                    new SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FRONT_LEFT_DRIVE, FrontLeft.ABSOLUTE_OFFSET, FRONT_LEFT_TURN),
                    new SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, FRONT_RIGHT_DRIVE, FrontRight.ABSOLUTE_OFFSET, FRONT_RIGHT_TURN),
                    new SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, BACK_LEFT_DRIVE, BackLeft.ABSOLUTE_OFFSET, BACK_LEFT_TURN),
                    new SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, BACK_RIGHT_DRIVE, BackRight.ABSOLUTE_OFFSET, BACK_RIGHT_TURN)
                }
            );
        }
        
    }
    public static SwerveDrive getInstance(){
        return instance;
    }

    private final AbstractModule[] modules;  
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] module2ds;

    public SwerveDrive(AbstractModule[] modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModulesOffset());
        gyro = new AHRS(SPI.Port.kMXP);
        module2ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getID() + "-2d");
        }
    }

    /***Getters***/

    public Translation2d[] getModulesOffset() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getOffset();
        }
        return offsets;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());

    }

    /***Setters***/

    public SwerveModuleState filterModuleStates(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Settings.SwerveDrive.MODULE_VELOCITY_DEADBAND.get()) {
            return state;
        }
        else {
            return new SwerveModuleState(0, state.angle);
        }
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("States must be the same length as modules");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Settings.SwerveDrive.MAX_MODULE_SPEED.get());

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    /***Drive Functions***/ 

    public void drive(Vector2D translation, double rotation) {

    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /***Gyro Functions***/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroYaw() {
        return getGyroAngle();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }


    /***Kinematics***/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /***Periodic***/
    @Override
    public void periodic() {
        /* get the pose odometry, get the angle from odometry
        set the field object to the new poses {
            the translationn + the individual module's offset, rotated by the odometry's angle
            the state of the module's angle + the angle
        }
        */

        AbstractOdometry odometry = AbstractOdometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for (int i = 0; i < module2ds.length; i++) {
            module2ds[i].setPose(
                new Pose2d(
                    pose.getTranslation().plus(modules[i].getOffset().rotateBy(angle)),
                    modules[i].getAngle().rotateBy(angle)
                )
            );
        }

        SmartDashboard.putNumber("Swerve/Gyro Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Pitch (deg)", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Roll (deg)", getGyroRoll().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Yaw (deg)", getGyroYaw().getDegrees());

        SmartDashboard.putNumber("Swerve/Chassis Speeds/Angular", getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * 0.02));
    }

}
