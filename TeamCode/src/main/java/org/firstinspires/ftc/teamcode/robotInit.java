package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


//@Disabled
public class robotInit {

    /* Public OpMode members. */
    //creating objects
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor armLift;
    public DcMotor spinner;
//    public DcMotor armLift; //arm lifting mechanism


//    public CRServo spinner; // spins the motor to bring in the cone
    public Servo closer; // keeps the cone in place

    //from Encoder Sample
    double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;  // For figuring circumference
    double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    double     DRIVE_SPEED             = 0.35;
    double     teleOP_FORWARD_SPEED    = 1;

    /* local OpMode members. */
    HardwareMap hardwareMap ;

    // instantiate = to create an instance of = constructor
    /* Constructor */
    public robotInit(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;


        // Define and Initialize Motors
        motorFL = hardwareMap.get(DcMotor.class, "motor_fl");
        motorFR = hardwareMap.get(DcMotor.class, "motor_fr");
        motorBL = hardwareMap.get(DcMotor.class, "motor_bl");
        motorBR = hardwareMap.get(DcMotor.class, "motor_br");
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        // Set the direction of the DC motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        armLift.setDirection(DcMotor.Direction.FORWARD); //Not sure which direction
        spinner.setDirection(DcMotor.Direction.FORWARD); //Not sure which direction


        // Set all DC motors to zero power
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        armLift.setPower(0);
        spinner.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Not sure
        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Not sure


        // Define and initialize ALL installed servos.
        closer = hardwareMap.get(Servo.class, "closer");

        //init servos
        //freightSnatcher1.setPosition(0.72);
        //freightSnatcher2.setPosition(0.4);

    }
}