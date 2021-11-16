package org.firstinspires.ftc.teamcode.Scheduler;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;

public class Command {

    public enum CommandState {
        IDLE,
        WORKING,
        FINISHED,
    }

    public enum Type {
        DRIVE,
        TURN,
        LIFT,
        SETSERVO,
        STARTWHEEL,
        STOPWHEEL,
    }

    public Type type;
    public double value;
    public CommandState state;

    Command(Type type, double value) {
        this.type = type;
        this.value = value;
        this.state = CommandState.IDLE;
    }

    public void run() {
        Robot robot = Robot.getInstance();

        if(type == Type.STARTWHEEL || type == Type.STOPWHEEL) state = CommandState.FINISHED;
        else state = CommandState.WORKING;

        switch(type) {
            case DRIVE: robot.driveTrain.autoDrive(value); break;
            case TURN: robot.driveTrain.autoTurn(value); break;
            case LIFT: setLift(); break;
            case SETSERVO: robot.box.setPosition(value); break;
            case STARTWHEEL: robot.duckWheel.set(1); break;
            case STOPWHEEL: robot.duckWheel.set(0); break;
        }
    }

    public void setLift() {
        Robot robot = Robot.getInstance();

        switch((int) value) {
            case 0: robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO; break;
            case 1: robot.liftArm.liftHeight = LiftArm.LiftHeight.BOTTOM; break;
            case 2: robot.liftArm.liftHeight = LiftArm.LiftHeight.MIDDLE; break;
            case 3: robot.liftArm.liftHeight = LiftArm.LiftHeight.TOP; break;
        }
    }

    public void update() {
        Robot robot = Robot.getInstance();

        switch(type) {
            case DRIVE:
            case TURN:
                if(robot.driveTrain.atSetPoint()) state = CommandState.FINISHED;
                else state = CommandState.WORKING;
            break;
            case LIFT:
                if(robot.liftArm.liftPID.atSetPoint()) state = CommandState.FINISHED;
                else state = CommandState.WORKING;
            break;
            case SETSERVO:
                if(robot.box.getPosition() == value) state = CommandState.FINISHED;
                else state = CommandState.WORKING;
            break;
            default: state = CommandState.FINISHED; break;
        }
    }

}
