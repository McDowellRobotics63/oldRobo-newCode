package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.FieldOrientedDriveHelper;
import org.usfirst.frc.team63.robot.Kinematics;
import org.usfirst.frc.team63.robot.RobotMap;
import org.usfirst.frc.team63.robot.commands.FieldOrientedDriveCommand;
import org.usfirst.frc.team63.robot.util.DriveSignal;
import org.usfirst.frc.team63.robot.util.DriveVelocity;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	
	//The front of the robot is the climber
    public enum RobotDriveDirection {
        ROBOT_FORWARD, ROBOT_BACK, ROBOT_LEFT, ROBOT_RIGHT, GEAR_FORWARD, GEAR_BACK, GEAR_LEFT, GEAR_RIGHT;
    }
    
    public enum RobotRotateDirection {
        ROBOT_CW, ROBOT_CCW;
    }
    
	public enum DriveControlState {
        OPEN_LOOP, VELOCITY_CONTROL
    }
	
	private static final int kVelocityControlSlot = 0;
    public final TalonSRX  TalonFrontLeft, TalonFrontRight, TalonBackLeft, TalonBackRight;
    
    private final AHRS imu_; 
    
    private boolean isBrakeMode_ = true;   
    private DriveControlState driveControlState_;
	public FieldOrientedDriveHelper mDriveHelper = new FieldOrientedDriveHelper();
	
    public DriveSubsystem() {
    	TalonFrontLeft = new TalonSRX(RobotMap.FRONT_LEFT_MOTOR);
    	TalonBackLeft = new TalonSRX(RobotMap.REAR_LEFT_MOTOR);
    	TalonFrontRight = new TalonSRX(RobotMap.FRONT_RIGHT_MOTOR);
    	TalonBackRight = new TalonSRX(RobotMap.REAR_RIGHT_MOTOR);
    	
        imu_ = new AHRS(SPI.Port.kMXP);        

        // Start in open loop mode
        configureTalonsForOpenLoop();
        setOpenLoop(DriveSignal.NEUTRAL);    	
    }
    
    public void setTalonBaseConfigurationAuto()
    {
    	TalonFrontLeft.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonFrontLeft.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonFrontLeft.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonFrontLeft.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);  
    	
    	TalonBackLeft.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonBackLeft.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonBackLeft.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonBackLeft.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);   	    
    	
    	TalonFrontRight.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonFrontRight.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonFrontRight.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonFrontRight.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);   	    
    	
    	TalonBackRight.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonBackRight.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonBackRight.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonBackRight.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);    	    
    	
    	// Setup feedback params
    	TalonFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonFrontLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonFrontLeft.setInverted(true);
    	
    	TalonBackLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonBackLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonBackLeft.setInverted(true);
    	
    	TalonBackRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonBackRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonBackRight.setInverted(true);
    	
    	TalonFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonFrontRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonFrontRight.setInverted(true);
    	
        // Load velocity control gains
    	TalonFrontLeft.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	TalonFrontRight.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	TalonBackLeft.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	
    	TalonBackRight.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    }

    public void setTalonBaseConfigurationTelop()
    {
    	TalonFrontLeft.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonFrontLeft.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonFrontLeft.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonFrontLeft.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);  
    	
    	TalonBackLeft.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonBackLeft.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonBackLeft.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonBackLeft.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);   	    
    	
    	TalonFrontRight.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonFrontRight.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonFrontRight.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonFrontRight.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);   	    
    	
    	TalonBackRight.configNominalOutputForward(0.0, RobotMap.kTimeoutMs);
    	TalonBackRight.configNominalOutputReverse(-0.0, RobotMap.kTimeoutMs);
    	TalonBackRight.configPeakOutputForward(+12.0, RobotMap.kTimeoutMs);    	    
    	TalonBackRight.configPeakOutputReverse(+12.0, RobotMap.kTimeoutMs);     	    
    	
    	// Setup feedback params
    	TalonFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonFrontLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonFrontLeft.setInverted(true);
    	
    	TalonBackLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonBackLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonBackLeft.setInverted(true);
    	
    	TalonBackRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonBackRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonBackRight.setInverted(true);
    	
    	TalonFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	TalonFrontRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, RobotMap.kTimeoutMs);
    	TalonFrontRight.setInverted(true);
    	
        // Load velocity control gains
    	TalonFrontLeft.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonFrontLeft.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	TalonFrontRight.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonFrontRight.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	TalonBackLeft.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonBackLeft.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    	
    	TalonBackRight.config_kP(0,RobotMap.kDriveVelocityKpAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kI(0, RobotMap.kDriveVelocityKiAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kD(0,RobotMap.kDriveVelocityKdAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_kF(0,RobotMap.kDriveVelocityKfAuto, RobotMap.kTimeoutMs);
    	TalonBackRight.config_IntegralZone(0,RobotMap.kDriveVelocityIZone, RobotMap.kTimeoutMs);
    	
    }
    
    private void configureTalonsForOpenLoop() {
    	configureTalonForOpenLoop(TalonFrontLeft);
    	configureTalonForOpenLoop(TalonBackLeft);
    	configureTalonForOpenLoop(TalonFrontRight);
    	configureTalonForOpenLoop(TalonBackRight);
    	setBrakeMode(false);
    }
    
    private void configureTalonForOpenLoop(TalonSRX talon)
    {
    	talon.configOpenloopRamp(0, RobotMap.kTimeoutMs); //param 1 =seconds from neutral throttle to full throttle
    	talon.set(TalonSRX.TalonControlMode.PercentVbus, 0);
    	talon.setInverted(true);
    }
    
    private void configureTalonsForSpeedControl() {    	
    	configureTalonForSpeedControl(TalonFrontLeft, false);
    	configureTalonForSpeedControl(TalonBackLeft, false);
    	configureTalonForSpeedControl(TalonFrontRight, true);
    	configureTalonForSpeedControl(TalonBackRight, true);
        setBrakeMode(true);
    }
    
    private void configureTalonForSpeedControl(TalonSRX talon, boolean reverseOutput)
    {
    	talon.configOpenloopRamp(0.0, RobotMap.kTimeoutMs);
    	talon.changeControlMode(TalonSRX.TalonControlMode.Speed);
    	talon.selectProfileSlot(kVelocityControlSlot, 0); // param 2 = pid index
    	talon.configAllowableClosedloopError(kVelocityControlSlot, RobotMap.kDriveVelocityAllowableError, RobotMap.kTimeoutMs);       
    	talon.setInverted(reverseOutput);
    }
    
    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
        	TalonFrontLeft.setNeutralMode(null);
        	TalonBackLeft.setNeutralMode(null);
        	TalonFrontRight.setNeutralMode(null);
        	TalonBackRight.setNeutralMode(null);
            isBrakeMode_ = on;
        }
    }
    
    public synchronized void operatorControl(double fwd_back, double left_right, double rotate)
    {
    	setOpenLoop(mDriveHelper.fieldOrientedDrive(fwd_back, left_right, rotate, getGyroAngleDegrees()));
    }
    
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            configureTalonsForOpenLoop();
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        TalonFrontLeft.set(signal.leftFrontMotor);
        TalonBackLeft.set(signal.leftRearMotor);
        TalonFrontRight.set(signal.rightFrontMotor);
        TalonBackRight.set(signal.rightRearMotor);        
    }
    
    public synchronized void setVelocityZero()
    {
        if (driveControlState_ != DriveControlState.VELOCITY_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_CONTROL;
        }
    	
    	updateVelocitySetpoint(new DriveVelocity(0,0,0,0));
    }
    
    public synchronized void moveStraightForwardBack(double inches_per_sec)
    {
        if (driveControlState_ != DriveControlState.VELOCITY_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_CONTROL;
        }       
        
        updateVelocitySetpoint(Kinematics.inverseKinematics(new RigidTransform2d.Delta(inches_per_sec, 0, 0)));
    }
    
    public synchronized void moveStraightLeftRight(double inches_per_sec)
    {
        if (driveControlState_ != DriveControlState.VELOCITY_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_CONTROL;
        }
        
        updateVelocitySetpoint(Kinematics.inverseKinematics(new RigidTransform2d.Delta(0, inches_per_sec, 0)));
    }
    
    public synchronized void rotateDrive(double degrees_per_sec)
    {
        if (driveControlState_ != DriveControlState.VELOCITY_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_CONTROL;
        }
        
        updateVelocitySetpoint(Kinematics.inverseKinematics(new RigidTransform2d.Delta(0, 0, degrees_per_sec)));
    }
    
    private synchronized void updateVelocitySetpoint(DriveVelocity setpoint) {    	
		TalonFrontLeft.set(inchesPerSecondToRpm(setpoint.left_front));
		TalonBackLeft.set(inchesPerSecondToRpm(setpoint.left_rear));
		TalonFrontRight.set(inchesPerSecondToRpm(setpoint.right_front));
		TalonBackRight.set(inchesPerSecondToRpm(setpoint.right_rear));  
    }

    public synchronized void zeroSensors() {
        resetEncoders();
        resetGyro();
    }
    
    public synchronized void resetEncoders() {
    	TalonFrontLeft.setPosition(0);
    	TalonBackLeft.setPosition(0);
    	TalonFrontRight.setPosition(0);
    	TalonBackRight.setPosition(0);
		
    	TalonFrontLeft.setEncPosition(0);
    	TalonBackLeft.setEncPosition(0);
    	TalonFrontRight.setEncPosition(0);
    	TalonBackRight.setEncPosition(0);
    }    
    
	public synchronized void outputToSmartDashboard()
	{		
		SmartDashboard.putNumber("driveControlState_", driveControlState_.ordinal());		
		SmartDashboard.putNumber("drive_mode", mDriveHelper.GetDriveMode().ordinal());
		SmartDashboard.putNumber("left_front_velocity_check", getFrontLeftVelocityInchesPerSec());
		SmartDashboard.putNumber("right_front_velocitycheck", getFrontRightVelocityInchesPerSec());
		SmartDashboard.putNumber("left_rear_velocitycheck", getRearLeftVelocityInchesPerSec());
		SmartDashboard.putNumber("right_rear_velocitycheck", getRearRightVelocityInchesPerSec());
		SmartDashboard.putNumber("left_front_distance", getFrontLeftDistanceInches());
		SmartDashboard.putNumber("right_front_distance", getFrontRightDistanceInches());
		SmartDashboard.putNumber("left_rear_distance", getRearLeftDistanceInches());
		SmartDashboard.putNumber("gyro_deg", getGyroAngleDegrees());
		SmartDashboard.putNumber("mOrientationSetpoint", mDriveHelper.GetOrientationSetpoint());
		SmartDashboard.putBoolean("isNavXCalibrating", isNavXCalibrating());
		SmartDashboard.putBoolean("isNavXConnected", isNavXConnected());
	}
    
    public synchronized boolean isNavXCalibrating()
    {
    	return imu_.isCalibrating();
    }
    
    public synchronized boolean isNavXConnected()
    {
    	return imu_.isConnected();
    }
    
    public synchronized void resetGyro()
    {
    	imu_.reset();
    }
    
    public synchronized double getGyroAngleDegrees() {
        return -imu_.getAngle() % 360.0;
    }
    
    private static double inchesToRotations(double inches) {
        return inches / (RobotMap.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
   
    public double getAverageDistance()
    {
    	return (getFrontLeftDistanceInches() + getRearLeftDistanceInches() + getFrontRightDistanceInches() + getRearRightDistanceInches()) / 4.0;
    }
    
    public double getFrontLeftDistanceInches() {
        return rotationsToInches(TalonFrontLeft.getPosition());
    }

    public double getRearLeftDistanceInches() {
        return rotationsToInches(TalonBackLeft.getPosition());
    }
    
    public double getFrontRightDistanceInches() {
        return rotationsToInches(TalonFrontRight.getPosition());
    }

    public double getRearRightDistanceInches() {
        return rotationsToInches(TalonBackRight.getPosition());
    }

    public double getAverageVelocity()
    {
    	return (getFrontLeftVelocityInchesPerSec() + getRearLeftVelocityInchesPerSec() + getFrontRightVelocityInchesPerSec() + getRearRightVelocityInchesPerSec()) / 4.0;
    }
    
    public double getFrontLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(TalonFrontLeft.getSpeed());
    }
   
    public double getRearLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(TalonBackLeft.getSpeed());
    }
    
    public double getFrontRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(TalonFrontRight.getSpeed());
    }
   
    public double getRearRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(TalonBackRight.getSpeed());
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (RobotMap.kDriveWheelDiameterInches * Math.PI);
    }
    
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
   
	public void initDefaultCommand() {
		setDefaultCommand(new FieldOrientedDriveCommand());
	}
}
