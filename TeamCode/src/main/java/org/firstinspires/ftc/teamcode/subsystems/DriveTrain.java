package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {


    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    private DifferentialDrive driveTrain;

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public PIDController leftPID;
    public PIDController rightPID;

    public enum DriveMode {
        AUTONOMOUS,
        MANUAL,
    }

    public DriveTrain(final HardwareMap hwMap, DriveMode driveMode) {
        Robot robot = Robot.getInstance();


        robot.rightBack = new HerbergerMotor(hwMap, "rightBack", 134.4);
        robot.rightFront = new HerbergerMotor(hwMap, "rightFront", 134.4);
        robot.leftBack = new HerbergerMotor(hwMap, "leftBack", 134.4);
        robot.leftFront = new HerbergerMotor(hwMap, "leftFront", 134.4);

        leftMotors = new MotorGroup(robot.leftFront, robot.leftBack);
        rightMotors = new MotorGroup(robot.rightFront, robot.rightBack);

        if(driveMode == DriveMode.AUTONOMOUS) rightMotors.setInverted(true);

        driveTrain = new DifferentialDrive(leftMotors, rightMotors);

        leftPID = new PIDController(0.001, 0, 0.0001);
        rightPID = new PIDController(0.001, 0, 0.0001);

        resetEncoders();
    }


    public void drive(double forward, double turn) {
        driveTrain.arcadeDrive(forward, turn);
    }

    public void driveTrainController() {
        Robot robot = Robot.getInstance();
        double rightCalculation = rightPID.calculate(robot.rightFront.getEncoderCount());
        double leftCalculation = leftPID.calculate(robot.leftFront.getEncoderCount());

        driveTrain.tankDrive(leftCalculation, rightCalculation);
    }

    public void autoDrive(double distance) {

        rightPID.setSetPoint(-distance);
        leftPID.setSetPoint(distance);

        resetEncoders();
    }

    public void autoTurn(double turnAngle) {

        double distance = 6 * turnAngle;

        rightPID.setSetPoint(distance);
        leftPID.setSetPoint(distance);

        resetEncoders();
    }

    public void resetEncoders() {
        Robot robot = Robot.getInstance();

        robot.leftBack.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightFront.resetEncoder();

    }

    public boolean isBusy() {
        Robot robot = Robot.getInstance();
        boolean isBusy;
        if (robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }

    public boolean atSetPoint() {
        if(rightPID.atSetPoint() && leftPID.atSetPoint()) return true;
        else return false;
    }


    public void stop()
    {
        Robot robot = Robot.getInstance();

        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);

    }



}

