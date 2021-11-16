package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;

public class Robot {

        // Static variable reference of single_instance
        // of type Singleton
        private static Robot single_instance = null;

        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();
        //drivetrain
        public HerbergerMotor rightFront = null;
        public HerbergerMotor leftFront = null;
        public HerbergerMotor rightBack = null;
        public HerbergerMotor leftBack = null;

        public DriveTrain driveTrain = null;

        public HerbergerMotor intake = null;


        //duckwheel
        public CRServo duckWheel = null;

        //lift
        public HerbergerMotor lift = null;
        public ServoEx box = null;

        public LiftArm liftArm = null;




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
            driveTrain = new DriveTrain(hwMap, driveMode);


            liftArm = new LiftArm(hwMap);


            duckWheel = new CRServo(hwMap,"duckWheel");



            intake = new HerbergerMotor(hwMap,"intake",134.4);



        }

}
