package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.HerbergerMotor;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftArm;

public class Robot {

        // Static variable reference of single_instance
        // of type Singleton
        private static Robot single_instance = null;

        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();
        //drivetrain
        public Motor rightFront = null;
        public Motor leftFront = null;
        public Motor rightBack = null;
        public Motor leftBack = null;

        public Motor duckMotor = null;

        public HerbergerMotor lift = null;
        public ServoEx claw = null;

        public Camera camera = null;

        public LiftArm liftArm = null;
        public DriveTrain driveTrain = null;
        public DuckWheel duckWheel = null;



    // Constructor
        // Here we will be creating private constructor
        // restricted to this class itself
        private Robot()
        {

        }

        // Static method
        // Static method to create instance of Singleton class
        public static Robot getInstance()
        {
            if (single_instance == null)
                single_instance = new Robot();

            return single_instance;
        }
        public static Robot resetInstance()
        {
                single_instance = new Robot();
                return single_instance;
        }

        public void init(HardwareMap ahwMap, DriveTrain.DriveMode driveMode)
        {
            hwMap = ahwMap;
            camera = new Camera(hwMap);
            driveTrain = new DriveTrain(hwMap, driveMode);
            liftArm = new LiftArm(hwMap);
            duckWheel = new DuckWheel(hwMap);

        }

}
