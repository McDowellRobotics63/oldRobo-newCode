package org.usfirst.frc.team63.robot.simple_commands;

import org.usfirst.frc.team63.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BallDejamCommand extends Command {

    public BallDejamCommand() {
    	requires(Robot.balls);    	
    	setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.drive.ArcadeDrive(-0.5, 0.0);
    	Robot.balls.BallDejam();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
