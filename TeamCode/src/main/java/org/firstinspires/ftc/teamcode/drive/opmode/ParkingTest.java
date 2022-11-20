package org.firstinspires.ftc.teamcode.drive.opmode;

//Import robot general stuff
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;


// Import teamcode

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Import roadrunner thingies
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import java.util.Hashtable;

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
    public static int scanStartHoriz = horizontalBorderSize+40;
    public static int topBorder = 420;
    public static int bottomBorder = 105;
    public static int scanWidth = (1280- horizontalBorderSize *2);
    public static int scanHeight = 720- topBorder - bottomBorder;

    public static int exposure = 40000;
    public static int gain     = 250;

    private static boolean prevExposure = false;
    private static boolean prevGain     = false;
    private Sensor mSensor = null;
    private Device mDevice = null;
    private String lastbarcode = "";
    private int readno=0;
    boolean reprint=true;
    @Override
    public void runOpMode() throws InterruptedException {
        int parkingSlot;
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
        final ImagePopup imagePopUp = new ImagePopup(hardwareMap.appContext);
        try{
            RsContext.init(hardwareMap.appContext);
            Config config = new Config();

            config.enableStream(StreamType.DEPTH, StreamFormat.Z16);
            config.enableStream(StreamType.INFRARED, 1, 1280, 720, StreamFormat.Y8, 5);

            pp=pipeline.start(config, frameQueue::Enqueue);
            pp.getDevice().isInAdvancedMode();

            mDevice = pp.getDevice();
            mSensor = mDevice.querySensors().get(0);
            if(!mDevice.isInAdvancedMode()) {
                mDevice.toggleAdvancedMode(true);
            }

            mSensor.setValue(Option.ENABLE_AUTO_EXPOSURE, 1);
            mSensor.setValue(Option.GAIN, gain);
            mSensor.setValue(Option.EXPOSURE, exposure);

        } catch (Exception e) {
            e.printStackTrace();
        }


        waitForStart();
        int attempt = 0;

        TrajectoryBuilder fwd = new TrajectoryBuilder(new Pose2d(), SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));
        Trajectory fwdtraj = fwd.forward(14).build();
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
                        reprint = true;
                    }
                } else if(gamepad1.dpad_down){
                    if(!prevGain) {
                        gain -= 10;
                        System.out.println("Gain" + gain);
                        mSensor.setValue(Option.GAIN, gain);
                        prevGain = true;
                        reprint = true;
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
                        reprint = true;
                    }
                } else if(gamepad1.dpad_left){
                    if(!prevExposure) {
                        exposure -= 1000;
                        System.out.println("Exposure" + exposure);
                        mSensor.setValue(Option.EXPOSURE, exposure);
                        prevExposure = true;
                        reprint = true;
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
                                                        topBorder,
                                                        scanWidth,
                                                        scanHeight,
                                                        false
                                                )
                                        )
                                );
                                if (reprint)
                                {
                                    BitMatrix blackMatrix = bitmap.getBlackMatrix();
                                    int mWidth=blackMatrix.getWidth();
                                    int mHeight=blackMatrix.getHeight();
                                    for (int y=0;y<mHeight;y++) {
                                        StringBuilder line = new StringBuilder();
                                        for (int x=0;x<mWidth;x++) {
                                            if (blackMatrix.get(x, y)) line.append("██"); else line.append("  ");
                                        }
                                        line.append("|");
                                        System.out.println(line);
                                    }
                                    reprint=false;
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
                trajectory = builder.strafeRight(24).build();
                break;
            }
            if(Integer.parseInt(result.getText()) == 2){
                trajectory = builder.forward(0).build();
                break;
            }
            if(Integer.parseInt(result.getText()) == 1){
                trajectory = builder.strafeLeft(24).build();
                break;
            }
        }


        if(trajectory != null) {
            drive.followTrajectory(trajectory);
        }
        pipeline.stop();
    }
}
