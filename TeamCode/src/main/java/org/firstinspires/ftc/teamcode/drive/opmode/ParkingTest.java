package org.firstinspires.ftc.teamcode.drive.opmode;

//Import robot general stuff
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;


// Import teamcode

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Import roadrunner thingies
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

// Import robot general stuff 2.0
import com.ceylonlabs.imageviewpopup.ImagePopup;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.datamatrix.DataMatrixReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import whatever-the-frick this is

// Import Google ZXing (barcode reading)
import com.google.zxing.*;

// Import RealSense Library
import com.intel.realsense.librealsense.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Hashtable;

import ma.phoenix.ftc.cameradebugger.ImageType;

@Autonomous(group="test")


public class ParkingTest extends LinearOpMode {

    //1280x720

    /*
    public static int scanStartHoriz = 650;
    public static int scanStartVert  = 423;
    public static int scanWidth = (1280-scanStartHoriz)/2;
    public static int scanHeight = 720-scanStartVert-223;
    */

    /*
    public static int scanStartHorizIdeal = 600;
    public static int scanStartHoriz = 600+40;
    public static int scanStartVert  = 423;
    public static int scanWidth = (1280-scanStartHorizIdeal*2);
    public static int scanHeight = 720-scanStartVert-223;
    */

    /*
    public static int horizontalBorderSize = 480;
    public static int scanStartHoriz = horizontalBorderSize+40;
    public static int topBorder = 300;
    public static int bottomBorder = 100;
    public static int scanWidth = (1280- horizontalBorderSize *2);
    public static int scanHeight = 720- topBorder - bottomBorder;
    */

    public static int horizontalBorderSize = 520;
    //public static int horizontalBorderSize = 560;
            //public static int scanStartHoriz = horizontalBorderSize+40;
            //public static int topBorder = 420;
            //public static int bottomBorder = 105;
            //public static int scanWidth = (1280- horizontalBorderSize *2);
            //public static int scanHeight = 720- topBorder - bottomBorder;

    public static int scanStartVert = 0;
    public static int scanStartHoriz = 0;
    public static int scanWidth = 1280;
    public static int scanHeight = 720;


    public int exposure = 40000;
    public int gain     = 120;

    private static boolean prevExposure = false;
    private static boolean prevGain     = false;
    private Sensor mSensor = null;
    private Device mDevice = null;
    private String lastbarcode = "";

    private int readno=0;



    @Override
    public void runOpMode() throws InterruptedException {
        int parkingSlot;
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");

        ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 12.411);
        ((DcMotorEx) lift).setPositionPIDFCoefficients(15);
        // Put initialization blocks here.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Hashtable<DecodeHintType, Object> hints=new Hashtable<DecodeHintType, Object>();
        hints.put(DecodeHintType.TRY_HARDER, true);

        Result result = null;
        Reader reader = new DataMatrixReader();

        // Camera res
        int height = 0;
        int width = 0;



        // Frame buffer stuff
        boolean frameBufferAllocated = false;
        byte[] frameBuffer = null;
        FrameQueue frameQueue = new FrameQueue();

        // Telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // PIPELINE
        Pipeline pipeline = new Pipeline();
        PipelineProfile pp=null;

        // Config the config that configs the pipeline that configs the camera
        try{
            RsContext.init(hardwareMap.appContext);
            isOpModeActive();
            Config config = new Config();
            isOpModeActive();

            config.enableStream(StreamType.DEPTH, StreamFormat.Z16);
            isOpModeActive();
            config.enableStream(StreamType.INFRARED, 1, 1280, 720, StreamFormat.Y8, 5);
            isOpModeActive();

            pp=pipeline.start(config, frameQueue::Enqueue);
            isOpModeActive();
            pp.getDevice().isInAdvancedMode();
            isOpModeActive();

            mDevice = pp.getDevice();
            isOpModeActive();
            mSensor = mDevice.querySensors().get(0);
            isOpModeActive();
            if(!mDevice.isInAdvancedMode()) {
                mDevice.toggleAdvancedMode(true);
            }

            isOpModeActive();
            mSensor.setValue(Option.ENABLE_AUTO_EXPOSURE, 0);
            isOpModeActive();
            mSensor.setValue(Option.GAIN, gain); // Will throw an exception if Auto Exposure mode is on
            isOpModeActive();
            mSensor.setValue(Option.EXPOSURE, exposure);  // Will throw an exception if Auto Exposure mode is on
        }
        catch (Exception e) {
            e.printStackTrace();
        }


        waitForStart();

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-0.3);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        lift.setTargetPosition((int) ((5 / 4.409) * 384.5));

        int attempt = 0;

        TrajectoryBuilder fwd = new TrajectoryBuilder(new Pose2d(), SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));
        Trajectory fwdtraj = fwd.forward(13.75).build();
        drive.followTrajectory(fwdtraj);

        while(opModeIsActive()) {

            {
                telemetry.addData("Gain", gain);
                telemetry.addData("Exposure", exposure);
                telemetry.addData("Last barcode", lastbarcode);
                telemetry.addData("frame", attempt);
                telemetry.update();
            }

            try{
                if(gamepad1.dpad_up){
                    if(!prevGain) {
                        gain += 10;
                        System.out.println("Gain" + gain);
                        mSensor.setValue(Option.GAIN, gain);
                        prevGain = true;
                    }
                } else if(gamepad1.dpad_down){
                    if(!prevGain) {
                        gain -= 10;
                        System.out.println("Gain" + gain);
                        mSensor.setValue(Option.GAIN, gain);
                        prevGain = true;
                    }
                } else {
                    prevGain = false;
                }

                if(gamepad1.dpad_right){
                    if(!prevExposure) {
                        exposure += 1000;
                        System.out.println("Exposure" + exposure);
                        mSensor.setValue(Option.EXPOSURE, exposure);
                        prevExposure = true;
                    }
                } else if(gamepad1.dpad_left){
                    if(!prevExposure) {
                        exposure -= 1000;
                        System.out.println("Exposure" + exposure);
                        mSensor.setValue(Option.EXPOSURE, exposure);
                        prevExposure = true;
                    }
                } else {
                    prevExposure = false;
                }
            } catch (Exception e){
            }

            try {
                try (FrameSet frames = frameQueue.pollForFrames()) {
                    if (frames != null) {
                        try (Frame frame = frames.first(StreamType.INFRARED, StreamFormat.Y8)) {
                            if (frame != null) {
                                VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                                if (!frameBufferAllocated) {
                                    frameBuffer = new byte[videoFrame.getDataSize()];
                                    height = videoFrame.getHeight();
                                    width = videoFrame.getWidth();
                                }
                                videoFrame.getData(frameBuffer);
                                BinaryBitmap bitmap = new BinaryBitmap(
                                        new HybridBinarizer(
                                                new PlanarYUVLuminanceSource(
                                                        frameBuffer,
                                                        width,
                                                        height,
                                                        scanStartHoriz,
                                                        scanStartVert,
                                                        scanWidth,
                                                        scanHeight,
                                                        false
                                                )
                                        )
                                );
                                //if (gamepad1.left_bumper)
                                {
                                    //System.out.println("Image requested");
                                    BitMatrix blackMatrix = bitmap.getBlackMatrix();
                                    int mWidth=blackMatrix.getWidth();
                                    int mHeight=blackMatrix.getHeight();

                                    int bytesPerRow = ((mWidth + 7) / 8);
                                    byte[] frameBufferMonochrome = new byte[bytesPerRow * mHeight];

                                    for (int y=0; y < mHeight; y++) {
                                        for (int x=0; x < bytesPerRow; x++ ) {
                                            frameBufferMonochrome[bytesPerRow*y + x] = (byte)(
                                                ((x*8 + 0 < mWidth) ? (blackMatrix.get(x*8+0, y) ? 1<<7: 0) : 0)+
                                                ((x*8 + 1 < mWidth) ? (blackMatrix.get(x*8+1, y) ? 1<<6: 0) : 0)+
                                                ((x*8 + 2 < mWidth) ? (blackMatrix.get(x*8+2, y) ? 1<<5: 0) : 0)+
                                                ((x*8 + 3 < mWidth) ? (blackMatrix.get(x*8+3, y) ? 1<<4: 0) : 0)+
                                                ((x*8 + 4 < mWidth) ? (blackMatrix.get(x*8+4, y) ? 1<<3: 0) : 0)+
                                                ((x*8 + 5 < mWidth) ? (blackMatrix.get(x*8+5, y) ? 1<<2: 0) : 0)+
                                                ((x*8 + 6 < mWidth) ? (blackMatrix.get(x*8+6, y) ? 1<<1: 0) : 0)+
                                                ((x*8 + 7 < mWidth) ? (blackMatrix.get(x*8+7, y) ? 1<<0: 0) : 0)
                                            );
                                        }
                                    }

                                    ma.phoenix.ftc.cameradebugger.ImageTransmitter.transmitImage(ImageType.MONOCHROME_Y1, frameBufferMonochrome, mWidth, mHeight);
                                }
                                attempt+=1;

                                result = reader.decode(bitmap, hints);
                                System.out.println("Barcode text: " + result.getText());
                                lastbarcode=result.getText()+" read no "+readno++;

                                if(!result.getText().isEmpty()) {
                                    break;
                                }

                            }
                        }
                    }
                }
            }
            catch (NotFoundException e){

            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }

        TrajectoryBuilder trajectoryBuilder =  new TrajectoryBuilder(drive.getPoseEstimate(), SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));

        Trajectory fwd2 = trajectoryBuilder.forward(11).build();
        drive.followTrajectory(fwd2);

        TrajectoryBuilder builder = new TrajectoryBuilder(
                fwd2.end(),
                false,
                SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL)
        );

        Trajectory trajectory = null;
        while(opModeIsActive()){
            if(Integer.parseInt(result.getText()) == 3){
                trajectory = builder.strafeRight(15).build();
                break;
            }
            if(Integer.parseInt(result.getText()) == 2){
                trajectory = builder.forward(0).build();
                break;
            }
            if(Integer.parseInt(result.getText()) == 1){
                trajectory = builder.strafeLeft(15).build();
                break;
            }
        }


        if(trajectory != null) {
            drive.followTrajectory(trajectory);
        }
        pipeline.stop();
    }

    void isOpModeActive() throws InterruptedException {
        if(isStopRequested()){
            throw new InterruptedException();
        }
    }
}