package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Scheduler.Command;
import org.firstinspires.ftc.teamcode.Scheduler.Scheduler;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;


@Autonomous(name="testAuto")
public class testAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    Scheduler scheduler = new Scheduler();



    @Override
    public void init()
    {
        robot = robot.resetInstance();

        robot.init(hardwareMap, DriveTrain.DriveMode.AUTONOMOUS);
        telemetry.addLine("Initialized");

        robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;

        scheduleCommands();
    }

    private void scheduleCommands() {
        scheduler.schedule(Command.Type.LIFT, LiftArm.LiftHeight.TOP);
        scheduler.schedule(Command.Type.SETSERVO, 0.71);
        scheduler.schedule(Command.Type.SETSERVO, 0.3);
        scheduler.schedule(Command.Type.LIFT, LiftArm.LiftHeight.BOTTOM);
        scheduler.schedule(Command.Type.STARTWHEEL);
        scheduler.schedule(Command.Type.STOPWHEEL);
    }

    @Override
    public void init_loop(){}

    @Override
    public void start()
    {

        runtime.reset();
        robot.box.setPosition(0.3);

    }

    @Override
    public void loop()
    {
        robot.driveTrain.driveTrainController();
        robot.liftArm.liftController();

        scheduler.schedulerController();

    }

    @Override
    public void stop()
    {
        robot.driveTrain.stop();
        robot.lift.set(0);
        robot.intake.set(0);



    }

}




