package ma.phoenix.ftc.realsensecamera;

import android.graphics.Color;

import com.intel.realsense.librealsense.Align;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.DepthFrame;
import com.intel.realsense.librealsense.Device;
import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameQueue;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.Option;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.PipelineProfile;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.Sensor;
import com.intel.realsense.librealsense.StreamFormat;
import com.intel.realsense.librealsense.StreamType;
import com.intel.realsense.librealsense.VideoFrame;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

import ma.phoenix.ftc.realsensecamera.exceptions.NoFrameSetYetAcquiredException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStartException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStopException;
import ma.phoenix.ftc.realsensecamera.exceptions.DisconnectedCameraException;
import ma.phoenix.ftc.realsensecamera.exceptions.FrameQueueCloseException;
import ma.phoenix.ftc.realsensecamera.exceptions.StreamTypeNotEnabledException;
import ma.phoenix.ftc.realsensecamera.exceptions.UnsupportedStreamTypeException;

public class ConfigurableRealSenseCamera implements AutoCloseable{
    private final Pipeline mPipeline = new Pipeline();
    private boolean mPipelineStopped = true;

    private Device mDevice;
    private Sensor mSensor;

    private Config mConfig;
    private boolean mHaveConfig = false;

    private final FrameQueue mFrameQueue = new FrameQueue(1);
    private FrameSet mCachedFrameSet;
    private boolean mHaveCachedFrameSet = false;

    private Frame mUnCastedDepthFrame;
    private DepthFrame mCachedDepthFrame;
    private boolean mHaveCachedDepthFrame = false;


    private StreamFormat mColourStreamFormat;

    private final Align align = new Align(StreamType.DEPTH);

    public byte[] infraredFrameBuffer = new byte[1];
    private int mInfraredWidth, mInfraredHeight, mInfraredStride;
    private boolean mHaveCachedColourFrameBuffer = false;
    //
    public byte[] colourFrameBuffer = new byte[1];
    private int mColourWidth, mColourHeight, mColourStride;
    private boolean mHaveCachedInfraredFrameBuffer = false;
    //
    public byte[] depthFrameBuffer = new byte[1];
    private int mDepthWidth, mDepthHeight, mDepthStride;
    private boolean mHaveCachedDepthFrameBuffer = false;

    private int gain = -1;
    private int exp = -1;

    public ConfigurableRealSenseCamera(HardwareMap hardwareMap, BooleanSupplier isStopRequested) throws DisconnectedCameraException, InterruptedException {
        //DEBUG: System.out.println("constructor called");
        // -System.out.println("initializing context");
        RsContext.init(hardwareMap.appContext);
        //DEBUG: System.out.println("sleeping for two seconds");
        for(int i = 0; i < 200; i++) {
            Thread.sleep(10);
            //DEBUG: System.out.println("testing if stop is requested");
            if(!isStopRequested.getAsBoolean()){
                //DEBUG: System.out.println("breaking, stop was requested");
                break;
            }
        }
        //DEBUG: System.out.println("getting context");
        RsContext context = new RsContext();
        //DEBUG: System.out.println("checking that we don't have zero devices");
        if(context.queryDevices().getDeviceCount() == 0) throw new DisconnectedCameraException();
    }

    public void enableAutoExposure(Boolean enabled){
        //DEBUG: System.out.println("enableAutoExposure() called");
        System.out.println("Setting auto exposure");
        mSensor.setValue(Option.ENABLE_AUTO_EXPOSURE, enabled? 1 : 0);
        if(enabled) return;
        System.out.println("Setting gain");
        mSensor.setValue(Option.GAIN, gain);
        System.out.println("Setting exposure");
        mSensor.setValue(Option.EXPOSURE, exp);
    }

    private void start() throws CameraStartException, CameraStopException {
        //DEBUG: System.out.println("start() called");
        // -     System.out.println("checking if pipeline running");
        if(!mPipelineStopped) stop();
        PipelineProfile pipelineProfile;
        assert mHaveConfig: "A config must be set before calling start";
        try{
            //DEBUG: System.out.println("starting pipeline");
            pipelineProfile = mPipeline.start(mConfig, mFrameQueue::Enqueue);
        }catch (Exception e) {
            e.printStackTrace();
            throw new CameraStartException();
        }
        //DEBUG: System.out.println("getting device");
        mDevice = pipelineProfile.getDevice();
        //DEBUG: System.out.println("setting advanced mode");
        if(!mDevice.isInAdvancedMode())  mDevice.toggleAdvancedMode(true); // If advanced mode is already enabled, setting advanced mode will crash the library.

        //DEBUG: System.out.println("getting sensor");
        mSensor = mDevice.querySensors().get(0);
        mSensor.setValue(Option.FRAMES_QUEUE_SIZE, 12); // How many frames can exist at any one time. (REALLY BADLY NAMED OPTION). (FrameSet (1) + Current stream data (3)) * (1 -> us + 1 -> lib + 1 -> align) = 12
        //DEBUG: List<StreamProfile> allActive= mSensor.getActiveStreams();
        // -     for(StreamProfile streamProfile : allActive) {
        // -         System.out.println(
        // -                "Active profile: unique id:"+streamProfile.getUniqueId() +
        // -                " Handle:" + streamProfile.getHandle() +
        // -                " Index:" + streamProfile.getIndex() +
        // -                " Type:" + streamProfile.getType() +
        // -                " Format:" + streamProfile.getFormat()+
        // -                " Frame rate:" + streamProfile.getFrameRate());
        // -      }
        mPipelineStopped = false;
    }

    public void stop() throws CameraStopException {
        //DEBUG: System.out.println("stop() called");
        if(mPipelineStopped) return;
        try{
            //DEBUG: System.out.println("Stopping pipeline");
            mPipeline.stop();
            //DEBUG: System.out.println("Completed stop");
        } catch (Exception e){
            throw new CameraStopException();
        }
        mPipelineStopped = true;
    }

    private Device getDevice(){
        return mDevice;
    }

    private Sensor getSensor(){
        return mSensor;
    }

    public void switchConfig(Config toSwitch) throws CameraStartException, CameraStopException {
        //DEBUG: System.out.println("switchConfig called()");
        // -     System.out.println("calling stop()");
        if(!mPipelineStopped) stop();
        //DEBUG: System.out.println("getting reference to config");
        mConfig = toSwitch;
        //DEBUG: System.out.println("calling start()");
        mHaveConfig = true;
        if(mPipelineStopped) start();
    }

    public void setGain(int gain){
        //DEBUG: System.out.println("setGain()");
        this.gain = gain;
        //DEBUG: System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public void setExp(int exp){
        //DEBUG: System.out.println("setExp()");
        this.exp = exp;
        //DEBUG: System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public boolean updateFrameSet(){
        FrameSet newUnalignedFrameSet = mFrameQueue.pollForFrames();
        if(newUnalignedFrameSet == null) {
            return false;
        }
        //DEBUG: System.out.println("New frame set");
        // -     FrameSet newColorProcessedFrameSet = newUnalignedFrameSet.applyFilter(align);
        // -     System.out.println("closing old depthFrame");
        if(mHaveCachedFrameSet) {
            mCachedFrameSet.close();
            //DEBUG: System.out.println("Freed a processedDepthFrameSet frame");
        }
        if (mHaveCachedDepthFrame) {
            mUnCastedDepthFrame.close();
            mHaveCachedDepthFrame = false;
        }

        //DEBUG: System.out.println("saving new frameSet");
        mCachedFrameSet = newUnalignedFrameSet;
        mHaveCachedFrameSet = true;
        mHaveCachedColourFrameBuffer = false;
        mHaveCachedInfraredFrameBuffer = false;
        mHaveCachedDepthFrameBuffer = false;
        return true;
    }

    public void cacheDepthFrameIfNecessary() throws NoFrameSetYetAcquiredException, StreamTypeNotEnabledException {
        if(mHaveCachedDepthFrame)return;
        if(!mHaveCachedFrameSet) throw new NoFrameSetYetAcquiredException();
        mUnCastedDepthFrame = mCachedFrameSet.first(StreamType.DEPTH);
        if(mUnCastedDepthFrame == null) throw new StreamTypeNotEnabledException();
        mCachedDepthFrame = mUnCastedDepthFrame.as(Extension.DEPTH_FRAME);
        mHaveCachedDepthFrame = true;
    }

    public float getDistance(int x, int y) throws StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        //DEBUG: System.out.println("getDistance("+x+", "+y+")"); //848x480
        // -     System.out.println("checking to see if depth frame needs updating");
        cacheDepthFrameIfNecessary();
        //DEBUG: if(x==600) System.out.println("getDistance("+x+", "+y+") width: " +depthFrame.getWidth() +" height: "+depthFrame.getHeight() + "=" + depthFrame.getDistance(x, y));
        return mCachedDepthFrame.getDistance(x, y);
    }

    public FrameData getImageFrame(StreamType type) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        //System.out.println("getImageFrame()");
        if (!mHaveCachedFrameSet) throw new NoFrameSetYetAcquiredException();
        //System.out.println("checking For Frame");
        switch (type) {
            case DEPTH:
                if(!mHaveCachedDepthFrameBuffer) {
                    cacheDepthFrameIfNecessary();
                    //DEBUG: System.out.println("testing frame buffer length");
                    if (depthFrameBuffer.length < mCachedDepthFrame.getDataSize()) {
                        System.out.println("creating frame buffer");
                        depthFrameBuffer = new byte[mCachedDepthFrame.getDataSize()];
                    }
                    //DEBUG: System.out.println("getting frame buffer data");
                    mCachedDepthFrame.getData(depthFrameBuffer);
                    mDepthWidth = mCachedDepthFrame.getWidth();
                    mDepthHeight = mCachedDepthFrame.getHeight();
                    mDepthStride = mCachedDepthFrame.getStride();
                    mHaveCachedDepthFrameBuffer = true;
                }
                return new FrameData(depthFrameBuffer,
                        mDepthWidth,
                        mDepthHeight,
                        mDepthStride);
            case COLOR:
                if (!mHaveCachedColourFrameBuffer) {
                    try(Frame frame = mCachedFrameSet.first(type)) {
                        //DEBUG: System.out.println("testing if we support it");
                        // -     System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            throw new StreamTypeNotEnabledException();
                        }
                        //DEBUG: System.out.println("color frame found");
                        // -     System.out.println("is a color frame");
                        if (colourFrameBuffer.length < frame.getDataSize()) {
                            //DEBUG: System.out.println("creating frame buffer");
                            colourFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //DEBUG: System.out.println("getting frame buffer data");
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(colourFrameBuffer);
                        mColourWidth = videoFrame.getWidth();
                        mColourHeight = videoFrame.getHeight();
                        mColourStride = videoFrame.getStride();
                        mColourStreamFormat = videoFrame.getProfile().getFormat();
                        mHaveCachedColourFrameBuffer = true;
                        //UNNEEDED: videoFrame.close(); Not needed because it does not own frame
                    }
                }
                return new FrameData(colourFrameBuffer,
                        mColourWidth,
                        mColourHeight,
                        mColourStride);
            case INFRARED:
                if(!mHaveCachedInfraredFrameBuffer) {
                    try(Frame frame = mCachedFrameSet.first(type)) {
                        //DEBUG: System.out.println("testing if we support it");
                        //DEBUG: System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            throw new StreamTypeNotEnabledException();
                        }
                        //DEBUG: System.out.println("is a depth frame");
                        //DEBUG: System.out.println("is a infrared frame");
                        if (infraredFrameBuffer.length < frame.getDataSize()) {
                            //DEBUG: System.out.println("creating frame buffer");
                            infraredFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //DEBUG: System.out.println("getting frame buffer data");
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(infraredFrameBuffer);
                        mInfraredWidth = videoFrame.getWidth();
                        mInfraredHeight = videoFrame.getHeight();
                        mInfraredStride = videoFrame.getStride();
                        mHaveCachedInfraredFrameBuffer = true;
                        //UNNEEDED: videoFrame.close(); Not needed because it does not own frame
                    }
                }
                return new FrameData(infraredFrameBuffer,
                        mInfraredWidth,
                        mInfraredHeight,
                        mInfraredStride);
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public int getARGB(int x, int y) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        //System.out.println("GetARGB("+x+","+y+")");
        if(!mHaveCachedColourFrameBuffer) {
            //System.out.println("Getting color frame");
            getImageFrame(StreamType.COLOR);
        }

        // see https://www.wikiwand.com/en/YUV#Y%E2%80%B2UV444_to_RGB888_conversion
        int c, d, e;
        int index;
        switch (mColourStreamFormat)
        {
            case UYVY:
                //System.out.println("Getting UYVY");
                // UYVY UYVY UYVY UYVYUYVYUYVY
                //
                // 1/2 -> 0 * 2 -> 0
                index = y*mColourStride+(x>>1<<1)*2; // Remove the LSB from the horizontal pixel address, then multiply by 2
                c = byteToInt(colourFrameBuffer[y*mColourStride+(x<<1)+1])-16;
                d = byteToInt(colourFrameBuffer[index])-128;
                e = byteToInt(colourFrameBuffer[index+2])-128;
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case YUYV:
                //System.out.println("Getting YUYV");
                index = y*mColourStride+(x>>1<<1)*2; // Remove the LSB from the horizontal pixel address, then multiply by 2
                c = byteToInt(colourFrameBuffer[y*mColourStride+(x<<1)])-16;
                d = byteToInt(colourFrameBuffer[index+1])-128;
                e = byteToInt(colourFrameBuffer[index+3])-128;
                //System.out.println(
                //        " stride: "+ mColourStride +
                //        " index: "+index+
                //        " index of y: "+(y* mColourStride +((x>>1)<<2))+
                //        " x:"+x+
                //        " y:"+y+
                //        " c:"+c+
                //        " d:"+d+
                //        " e:"+e+
                //        " Y:"+byteToInt(colourFrameBuffer[index+0])+
                //        " U:"+byteToInt(colourFrameBuffer[index+1])+
                //        " Y:"+byteToInt(colourFrameBuffer[index+2])+
                //        " V:"+byteToInt(colourFrameBuffer[index+3]));
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case RGB8:
                System.out.println("Getting RGB");
                index = y*mColourStride + x*3; // Get pixel index
                return Color.argb(0,
                        byteToInt(colourFrameBuffer[index]),
                        byteToInt(colourFrameBuffer[index+1]),
                        byteToInt(colourFrameBuffer[index+2]));
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public void close() throws FrameQueueCloseException, CameraStopException {
        //DEBUG: System.out.println("close() called");
        //DEBUG: System.out.println("calling stop()");
        stop();
        //DEBUG: System.out.println("closing pipeline");
        if(mHaveCachedFrameSet) mCachedFrameSet.close();
        //DEBUG: System.out.println("closing depthFrame");
        if(mHaveCachedDepthFrame) mUnCastedDepthFrame.close();
        //DEBUG: System.out.println("closing queue");
        if(!mPipelineStopped) mPipeline.close();
        //DEBUG: System.out.println("closing frameSet");
        try {
            mFrameQueue.close();
        } catch (Exception e){
            throw new FrameQueueCloseException();
        }
    }

    private int byteToInt(byte x) {return x & 0xff;}
}
