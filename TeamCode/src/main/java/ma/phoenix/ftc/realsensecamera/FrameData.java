package ma.phoenix.ftc.realsensecamera;


import com.google.zxing.Dimension;

public class FrameData {
    private byte[] frameBuffer;
    private Dimension frameDimensions;
    private int stride;

    public FrameData(byte[] frameBuffer, Dimension frameDimensions, int stride){
        this.frameBuffer = frameBuffer;
        this.frameDimensions = frameDimensions;
        this.stride = stride;
    }

    public FrameData(byte[] frameBuffer, int width, int height, int stride){
        this.frameBuffer = frameBuffer;
        this.frameDimensions = new Dimension(width, height);
        this.stride=stride;
    }

    public byte[] getFrameBuffer(){
        return frameBuffer;
    }

    public Dimension getFrameDimensions(){
        return frameDimensions;
    }

    public int getWidth(){
        return frameDimensions.getWidth();
    }

    public int getHeight(){
        return frameDimensions.getHeight();
    }

    public int getStride() {
        return stride;
    }
}
