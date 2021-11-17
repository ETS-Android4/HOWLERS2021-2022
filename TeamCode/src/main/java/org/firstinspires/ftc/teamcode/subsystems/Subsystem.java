package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Scheduler.Enums.State;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;

import java.util.HashMap;
import java.util.Map;

public abstract class Subsystem {

    private State state = State.IDLE;
    public State getState() { return state; }
    public void setState(State state) {
        this.state = state;
    }
    Map<String, HerbergerMotor> motorMap = new HashMap<>();

    public boolean isWorking() {
        if(state == State.WORKING) return true;
        else return false;
    }
}
