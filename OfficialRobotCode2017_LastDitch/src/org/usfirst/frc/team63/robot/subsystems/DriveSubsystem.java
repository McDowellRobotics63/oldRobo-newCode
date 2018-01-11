package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.FieldOrientedDriveHelper;
import org.usfirst.frc.team63.robot.Kinematics;
import org.usfirst.frc.team63.robot.RobotMap;
import org.usfirst.frc.team63.robot.commands.FieldOrientedDriveCommand;
import org.usfirst.frc.team63.robot.util.DriveSignal;
import org.usfirst.frc.team63.robot.util.DriveVelocity;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;

import com.ctre.CANTalon;
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
    public final CANTalon TalonFrontLeft, TalonFrontRight, TalonBackLeft, TalonBackRight;
    
    private final AHRS imu_; 
    
    private boolean isBrakeMode_ = true;   
    private DriveControlState driveControlState_;
	public FieldOrientedDriveHelper mDriveHelper = new FieldOrientedDriveHelper();
	
    public DriveSubsystem() {
    	TalonFrontLeft = new CANTalon(RobotMap.FRONT_LEFT_MOTOR);
    	TalonBackLeft = new CANTalon(RobotMap.REAR_LEFT_MOTOR);
    	TalonFrontRight = new CANTalon(RobotMap.FRONT_RIGHT_MOTOR);
    	TalonBackRight = new CANTalon(RobotMap.REAR_RIGHT_MOTOR);
    	
        imu_ = new AHRS(SPI.Port.kMXP);        

        // Start in open loop mode
        configureTalonsForOpenLoop();
        setOpenLoop(DriveSignal.NEUTRAL);    	
    }
    
    public void setTalonBaseConfigurationAuto()
    {
    	TalonFrontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonFrontLeft.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonBackLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonBackLeft.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonFrontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonFrontRight.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonBackRight.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonBackRight.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	// Setup feedback params
    	TalonFrontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonFrontLeft.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonFrontLeft.configEncoderCodesPerRev(250);
    	TalonFrontLeft.reverseSensor(true);
    	
    	TalonBackLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonBackLeft.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonBackLeft.configEncoderCodesPerRev(250);
    	TalonBackLeft.reverseSensor(true);
    	
    	TalonFrontRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonFrontRight.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonFrontRight.configEncoderCodesPerRev(250);
    	TalonFrontRight.reverseSensor(false);
    	
    	TalonBackRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonBackRight.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonBackRight.configEncoderCodesPerRev(250);
    	TalonBackRight.reverseSensor(false);
    	
        // Load velocity control gains
    	TalonFrontLeft.setPID(RobotMap.kDriveVelocityKpAuto, RobotMap.kDriveVelocityKiAuto, RobotMap.kDriveVelocityKdAuto,
    			RobotMap.kDriveVelocityKfAuto, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonBackLeft.setPID(RobotMap.kDriveVelocityKpAuto, RobotMap.kDriveVelocityKiAuto, RobotMap.kDriveVelocityKdAuto,
    			RobotMap.kDriveVelocityKfAuto, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonFrontRight.setPID(RobotMap.kDriveVelocityKpAuto, RobotMap.kDriveVelocityKiAuto, RobotMap.kDriveVelocityKdAuto,
    			RobotMap.kDriveVelocityKfAuto, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonBackRight.setPID(RobotMap.kDriveVelocityKpAuto, RobotMap.kDriveVelocityKiAuto, RobotMap.kDriveVelocityKdAuto,
    			RobotMap.kDriveVelocityKfAuto, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    }

    public void setTalonBaseConfigurationTelop()
    {
    	TalonFrontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonFrontLeft.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonBackLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonBackLeft.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonFrontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonFrontRight.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	TalonBackRight.configNominalOutputVoltage(+0.0f, -0.0f);
    	TalonBackRight.configPeakOutputVoltage(+12.0f, -12.0f);    	    
    	
    	// Setup feedback params
    	TalonFrontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonFrontLeft.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonFrontLeft.configEncoderCodesPerRev(250);
    	TalonFrontLeft.reverseSensor(true);
    	
    	TalonBackLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonBackLeft.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonBackLeft.configEncoderCodesPerRev(250);
    	TalonBackLeft.reverseSensor(true);
    	
    	TalonFrontRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonFrontRight.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonFrontRight.configEncoderCodesPerRev(250);
    	TalonFrontRight.reverseSensor(false);
    	
    	TalonBackRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	TalonBackRight.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	TalonBackRight.configEncoderCodesPerRev(250);
    	TalonBackRight.reverseSensor(false);
    	
        // Load velocity control gains
    	TalonFrontLeft.setPID(RobotMap.kDriveVelocityKpTelop, RobotMap.kDriveVelocityKiTelop, RobotMap.kDriveVelocityKdTelop,
    			RobotMap.kDriveVelocityKfTelop, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonBackLeft.setPID(RobotMap.kDriveVelocityKpTelop, RobotMap.kDriveVelocityKiTelop, RobotMap.kDriveVelocityKdTelop,
    			RobotMap.kDriveVelocityKfTelop, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonFrontRight.setPID(RobotMap.kDriveVelocityKpTelop, RobotMap.kDriveVelocityKiTelop, RobotMap.kDriveVelocityKdTelop,
    			RobotMap.kDriveVelocityKfTelop, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    	
    	TalonBackRight.setPID(RobotMap.kDriveVelocityKpTelop, RobotMap.kDriveVelocityKiTelop, RobotMap.kDriveVelocityKdTelop,
    			RobotMap.kDriveVelocityKfTelop, RobotMap.kDriveVelocityIZone, RobotMap.kDriveVelocityRampRate,
                RobotMap.kVelocityControlSlot);
    }
    
    private void configureTalonsForOpenLoop() {
    	configureTalonForOpenLoop(TalonFrontLeft);
    	configureTalonForOpenLoop(TalonBackLeft);
    	configureTalonForOpenLoop(TalonFrontRight);
    	configureTalonForOpenLoop(TalonBackRight);
    	setBrakeMode(false);
    }
    
    private void configureTalonForOpenLoop(CANTalon talon)
    {
    	talon.setVoltageRampRate(120.0f);
    	talon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	talon.set(0);
    	talon.reverseOutput(false);
    }
    
    private void configureTalonsForSpeedControl() {    	
    	configureTalonForSpeedControl(TalonFrontLeft, false);
    	configureTalonForSpeedControl(TalonBackLeft, false);
    	configureTalonForSpeedControl(TalonFrontRight, true);
    	configureTalonForSpeedControl(TalonBackRight, true);
        setBrakeMode(true);
    }
    
    private void configureTalonForSpeedControl(CANTalon talon, boolean reverseOutput)
    {
    	talon.setVoltageRampRate(0.0f);
    	talon.changeControlMode(CANTalon.TalonControlMode.Speed);
    	talon.setProfile(kVelocityControlSlot);
    	talon.setAllowableClosedLoopErr(RobotMap.kDriveVelocityAllowableError);       
    	talon.reverseOutput(reverseOutput);
    }
    
    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
        	TalonFrontLeft.enableBrakeMode(on);
        	TalonBackLeft.enableBrakeMode(on);
        	TalonFrontRight.enableBrakeMode(on);
        	TalonBackRight.enableBrakeMode(on);
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
