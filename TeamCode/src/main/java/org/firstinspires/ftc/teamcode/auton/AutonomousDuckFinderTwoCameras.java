package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "AutonomousDuckFinderTwoCameras", group = "Linear Opmode")
public class AutonomousDuckFinderTwoCameras extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private double globalMovementTimer = 0;
    private double servoMovementDuration = 2;
    // a wheel will be moved accordingly depending on where the camera detected the ducky
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ad4VSlr/////AAABmU/W80sZZ0EvjCn23AM0xj0W9aNJdet9DwXAes9qzFoQdgfu6kvlH0qwmtDA5JyKzMjZvrbuUcCE6KeHTnNO+faWQZzmfirEncAqLM5yz1ebhsJEOcKoGTlZ8rVgubo/SxjeryDLzG9oX432r5bfCX60JlThxBUwAMRllGaoXrKfw3rXoJ5keGSuNLxC0chSmMy1XIU1Ldzp97LbAOEldDf98jFAjI6U04FqsqpJ2z2kbQp0Z/jxvrVxMmHRKOgz+P5VXrDB2AYU+L3ouKRbsxaakpqwY2CeHo0hBpoPRp+IcqQ4L+DNMyPhvFmis0qRGwsdJqBrpRIUItPYqKBkSSZUXBavr2/xmiJ3qiYvIVSz";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    public boolean findDuck(List<Recognition> recognitions, int tries) {
        int foundDucky = foundDuck(recognitions);
        if (!recognitions.get(foundDucky).equals(0)) {
            return true;
        }
        else if (tries <= 0) {
            return false;
        }
        else {
            sleep(250);
            return findDuck(recognitions, tries-1);
        }
    }

    public int foundDuck(List<Recognition> recognitions) {
        int index = 0;
        for (Recognition recognition : recognitions) {
            String currentLabel = recognition.getLabel();
            if (currentLabel.equals("Duck")) {
                return index;
            }
            index += 1;
        }
        return 0;
    }


    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        /*
        where first using camera one which is going to  be looking at 2 possible ducky spots
         */
        // set zoom accordingly to only look at 2 spots
        tfod.setZoom(2, 16.0/9.0);
        // screen being split in two
        float line1 = 15; // put pixel value
        // image our screen has 30 pixel left to right, this is how it would be split
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        boolean foundADucky = findDuck(updatedRecognitions, 10);
        int duckIndex = foundDuck(updatedRecognitions);

        /* a variable storing the duck right position
        with this we can just check fro left to right on the camera to know
        on which section of the camera the ducky is in
         */
        float duckRight = updatedRecognitions.get(duckIndex).getRight();


        /* example of what code does next
        || = line1
         marker1  ||  marker2
         the code first checks if it even found a ducky in what the camera is looking at
         if it did then it first checks wether the ducks right spot is behind line1,
         else that mean it's on the other side

         */
        if (foundADucky) {
            if (duckRight < line1) {
                leftBackDrive.setVelocity(3500);
                // sleep does in milliseconds - 1 second = 1000 seconds
                sleep(1000);
            } else {
                rightBackDrive.setVelocity(3500);
                sleep(1000);
            }
        }

        /*
        where next using camera two which is going to  be looking at 1 possible ducky spot
         */
        this.doCameraSwitching();
        // set zoom accordingly to only look at 1 spots
        tfod.setZoom(2.7, 16.0/9.0);
        // the screen won't be split this time

        foundADucky = findDuck(updatedRecognitions, 10);
        if (foundADucky) {
            leftFrontDrive.setVelocity(3500);
            sleep(1000);
        }



    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void doCameraSwitching() {
        if (switchableCamera.getActiveCamera() == webcam1) {
            switchableCamera.setActiveCamera(webcam2);
        }
        else {
            switchableCamera.setActiveCamera(webcam1);
        }
    }
}