package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;

public class LiftArm {

    public PIDController liftPID;

    public enum LiftHeight
    {
        TOP,
        MIDDLE,
        BOTTOM,
        ZERO,

    }

    public LiftHeight liftHeight = LiftHeight.ZERO;

    public LiftArm(final HardwareMap hwMap) {
        Robot robot = Robot.getInstance();


        robot.lift = new HerbergerMotor(hwMap, "lift", 134.4);
        robot.lift.runUsingEncoder();
        robot.lift.setInverted(true);
        robot.lift.resetEncoder();

        robot.box = new SimpleServo(hwMap,"box",0,90);
        robot.box.setInverted(true);

        liftPID = new PIDController(10, 0 ,0.01);
    }

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }


    public void setHeight(double setHeight) {

        liftPID.setSetPoint(setHeight);

    }

    public void liftController() {
       Robot robot = Robot.getInstance();

        switch(liftHeight) {
            case ZERO:
                setHeight(0);
                break;
            case BOTTOM:
                setHeight(350);
                break;
            case MIDDLE:
                setHeight(750);
                break;
            case TOP:
                setHeight(1250);
                break;
        }
        robot.lift.setVelocity(liftPID.calculate(robot.lift.getEncoderCount()));
    }

    public boolean isBusy() {
        Robot robot = Robot.getInstance();
        boolean isBusy;
        if (robot.lift.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }


    public void stop()
    {
        Robot robot = Robot.getInstance();

        robot.lift.set(0);


    }




}

