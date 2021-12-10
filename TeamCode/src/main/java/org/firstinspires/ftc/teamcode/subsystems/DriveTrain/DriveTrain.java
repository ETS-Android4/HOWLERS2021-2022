package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.HerbergerMotor;

public class DriveTrain extends SubsystemBase {


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

    public DriveMode driveMode;

    public PIDController leftPID;
    public PIDController rightPID;


    public enum DriveMode {
        AUTONOMOUS,
        MANUAL,
    }

    public DriveTrain(final HardwareMap hwMap, DriveMode driveMode) {
        Robot robot = Robot.getInstance();

        this.driveMode = driveMode;


        robot.rightBack = new Motor(hwMap, "rightBack", Motor.GoBILDA.RPM_312);
        robot.rightFront = new Motor(hwMap, "rightFront", Motor.GoBILDA.RPM_312);
        robot.leftBack = new Motor(hwMap, "leftBack", Motor.GoBILDA.RPM_312);
        robot.leftFront = new Motor(hwMap, "leftFront", Motor.GoBILDA.RPM_312);

        robot.rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotors = new MotorGroup(robot.leftFront, robot.leftBack);
        rightMotors = new MotorGroup(robot.rightFront, robot.rightBack);

        robot.leftFront.setInverted(true);
        robot.leftBack.setInverted(true);
        robot.rightBack.setInverted(true);

        driveTrain = new DifferentialDrive(leftMotors, rightMotors);

        leftPID = new PIDController(0, 0, 0);
        rightPID = new PIDController(0, 0, 0);

        resetEncoders();
    }


    public void drive(double forward, double turn) {
        driveTrain.arcadeDrive(forward, turn);
    }

    public void driveTrainController() {
        if(driveMode == DriveMode.AUTONOMOUS) {
            Robot robot = Robot.getInstance();
            double rightCalculation = rightPID.calculate(robot.rightFront.getCurrentPosition());
            double leftCalculation = leftPID.calculate(robot.leftFront.getCurrentPosition());

            driveTrain.tankDrive(leftCalculation, rightCalculation);
        }
    }

    @Override
    public void periodic() {
        driveTrainController();
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

    public void slow() {
        setSpeed(0.3);
    }
    public void fast() {
        setSpeed(0.6);
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

