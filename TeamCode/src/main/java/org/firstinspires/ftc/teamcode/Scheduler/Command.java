package org.firstinspires.ftc.teamcode.Scheduler;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double value;
    private LiftArm.LiftHeight liftValue;

    private CommandState state = CommandState.IDLE;
    public CommandState getState() { return state; }

    private ElapsedTime timer = new ElapsedTime();

    Command(Type type, double value) {
        this.type = type;
        this.value = value;
    }

    Command(Type type, LiftArm.LiftHeight value) {
        this.type = type;
        this.liftValue = value;
    }

    Command(Type type) {
        this.type = type;
    }

    public void run() {
        Robot robot = Robot.getInstance();

        if(type == Type.STARTWHEEL || type == Type.STOPWHEEL) state = CommandState.FINISHED;
        else state = CommandState.WORKING;

        switch(type) {
            case DRIVE: robot.driveTrain.autoDrive(value); break;
            case TURN: robot.driveTrain.autoTurn(value); break;
            case LIFT: robot.liftArm.liftHeight = liftValue; break;
            case SETSERVO: robot.box.setPosition(value); break;
            case STARTWHEEL:
                robot.duckWheel.set(1);
                state = CommandState.FINISHED;
                break;
            case STOPWHEEL:
                robot.duckWheel.set(0);
                state = CommandState.FINISHED;
                break;
        }

        timer.reset();
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
                if(timer.time() == 500) state = CommandState.FINISHED;
                else state = CommandState.WORKING;
            break;
            default: state = CommandState.FINISHED; break;
        }
    }

}
