package org.firstinspires.ftc.teamcode.Scheduler;


import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class Scheduler {

    public enum SchedulerState {
        IDLE,
        WORKING,
    }

    public SchedulerState schedulerState;

    public List<Command> scheduledCommands = new ArrayList<Command>();

    public Scheduler() {

    }


    public void schedule(Command.Type command, double value) {
        scheduledCommands.add(new Command(command, value));
    }

    public void schedulerController() {

        if(scheduledCommands.isEmpty()) schedulerState = SchedulerState.IDLE;
        else {
            schedulerState = SchedulerState.WORKING;

            Command currentCommand = scheduledCommands.get(0);

            switch(currentCommand.state) {
                case IDLE: currentCommand.run(); break;
                case WORKING: currentCommand.update(); break;
                case FINISHED: scheduledCommands.remove(currentCommand); break;
            }
        }

    }

}
