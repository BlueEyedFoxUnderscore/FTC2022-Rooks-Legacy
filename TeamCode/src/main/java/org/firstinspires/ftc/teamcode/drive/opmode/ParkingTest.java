package org.firstinspires.ftc.teamcode.drive.opmode;

//Import robot general stuff
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;


// Import teamcode
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Import roadrunner thingies
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

// Import robot general stuff 2.0
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.datamatrix.DataMatrixReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Import whatever-the-frick this is
import org.checkerframework.checker.units.qual.C;

// Import Google ZXing (barcode reading)
import com.google.zxing.*;
import com.google.zxing.qrcode.QRCodeReader;

// Import RealSense Library
import com.intel.realsense.librealsense.*;

@com.acmerobotics.dashboard.config.Config
@Autonomous(group="test")


public class ParkingTest extends LinearOpMode {

    static int scanStartVert  = 0;
    static int scanStartHoriz = 0;
    static int scanWidthVert  = 0;
    static int scanWidthHoriz = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingSlot;

        Result result;
        Reader reader = new DataMatrixReader();

        // Camera res
        int height = 0;
        int width = 0;


        // Frame buffer stuff
        boolean frameBufferAllocated = false;
        byte[] frameBuffer = null;
        FrameQueue frameQueue = new FrameQueue();

        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // PIPELINE
        Pipeline pipeline = new Pipeline();


        // Config the config that configs the pipeline that configs the camera
        try{
            RsContext.init(hardwareMap.appContext);
            Config config = new Config();

            config.enableStream(StreamType.DEPTH, StreamFormat.Z16);
            config.enableStream(StreamType.INFRARED, 1, 1280, 720, StreamFormat.Y8, 5);

            pipeline.start(config, frameQueue::Enqueue);
        } catch (Exception e) {
            e.printStackTrace();
        }


        TrajectoryBuilder builder = new TrajectoryBuilder(
            new Pose2d(0, 0, 0),
            false,
            SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL)
        );
        waitForStart();
        while(opModeIsActive()) {
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
                                    scanStartHoriz = (int)(width  * 1.5/4.0);
                                    scanStartVert =  (int)(height * 1.0/2.0);
                                    scanWidthHoriz = (int)(width  * 1.0/4.0);
                                    scanWidthVert =  (int)(height * 1.0/2.0);
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
                                                        scanWidthHoriz,
                                                        scanWidthVert,
                                                        false
                                                )
                                        )
                                );
                                result = reader.decode(bitmap);
                                System.out.println("Barcode text: " + result.getText());
                                telemetry.addData("Barcode Text", result.getText());
                                telemetry.update();
                            }
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        Trajectory trajectory = null;
        while(opModeIsActive()){
            if(gamepad1.a){
                trajectory = builder.forward(10).splineTo(new Vector2d(-10.0,10.0),90).build();
                break;
            }
            if(gamepad1.b){
                trajectory = builder.forward(48).splineTo(new Vector2d( 10.0,10.0),-90).build();
                break;
            }
            if(gamepad1.y){
                trajectory = builder.forward(48)                                                     .build();
                break;
            }
            if(gamepad1.x){
                trajectory = builder.forward(0).build();
                break;
            }
        }

        pipeline.stop();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(trajectory != null) {
            drive.followTrajectory(trajectory);
        }
    }
}
