package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class LiftArm extends SubsystemBase {

    public PIDController liftPID;

    private LiftHeight liftHeight = LiftHeight.ZERO;
    public LiftHeight getHeight() { return liftHeight; }
    public void setHeight(LiftHeight liftHeight) { this.liftHeight = liftHeight; }


    public LiftArm(final HardwareMap hwMap) {
        Robot robot = Robot.getInstance();


        robot.lift = new HerbergerMotor(hwMap, "lift", 134.4);
        robot.lift.runUsingEncoder();
        robot.lift.setInverted(true);
        robot.lift.resetEncoder();

        robot.box = new SimpleServo(hwMap,"box",0,180);

        liftPID = new PIDController(10, 0 ,0.01);
    }

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void liftController() {
       Robot robot = Robot.getInstance();

        switch(liftHeight) {
            case PICKUP:
                liftPID.setSetPoint(0);
                break;
            case ZERO:
                liftPID.setSetPoint(750);
                break;
            case BOTTOM:
                liftPID.setSetPoint(2000);
                break;
            case MIDDLE:
                liftPID.setSetPoint(3000);
                break;
            case TOP:
                liftPID.setSetPoint(4000);
                break;
        }
        robot.lift.setVelocity(liftPID.calculate(robot.lift.getEncoderCount()));
    }

    @Override
    public void periodic() {
        liftController();
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

