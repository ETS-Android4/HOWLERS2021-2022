package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.HerbergerMotor;

public class LiftArm extends SubsystemBase {

    public PIDController liftPID;
    public double getPIDTarget() {
        return liftPID.getSetPoint();
    }

    private LiftHeight liftHeight = LiftHeight.ZERO;
    public LiftHeight getHeight() { return liftHeight; }
    public void setHeight(LiftHeight liftHeight) { this.liftHeight = liftHeight; }
    public double currentError;
    public boolean PIDCONTROL = true;

    public LiftArm(final HardwareMap hwMap) {
        Robot robot = Robot.getInstance();


        robot.lift = new HerbergerMotor(hwMap, "lift", 134.4);
        robot.lift.setInverted(true);
        robot.lift.resetEncoder();

        robot.claw = new SimpleServo(hwMap,"box",200,360);
        robot.claw.setInverted(true);

        liftPID = new PIDController(0.004, 0, 0);
        liftPID.setTolerance(10);
        currentError = 0;
    }

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void liftController() {
        if(!PIDCONTROL) return;
       Robot robot = Robot.getInstance();

        switch(liftHeight) {
            case PICKUP:
                liftPID.setSetPoint(30);
                break;
            case ZERO:
                liftPID.setSetPoint(1300);
                break;
            case BOTTOM:
                liftPID.setSetPoint(5000);
                break;
            case MIDDLE:
                liftPID.setSetPoint(6000);
                break;
            case TOP:
                liftPID.setSetPoint(8500);
                break;
        }
        currentError = liftPID.calculate(robot.lift.getEncoderCount());
        robot.lift.set(liftPID.calculate(robot.lift.getEncoderCount()));
    }

    public void DISABLE() {
        Robot robot = Robot.getInstance();
        PIDCONTROL = false;
        robot.lift.set(0);
    }

    @Override
    public void periodic() {
        liftController();
    }

    public void openClaw() {
        Robot.getInstance().claw.setPosition(0.6);
    }

    public void closeClaw() {
        Robot.getInstance().claw.setPosition(0.98);
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

