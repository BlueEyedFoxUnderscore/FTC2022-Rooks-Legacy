package ma.phoenix.ftc.realsensecamera;


import com.google.zxing.Dimension;

public class FrameData {
    private byte[] frameBuffer;
    private Dimension frameDimensions;

    public FrameData(byte[] frameBuffer, Dimension frameDimensions){
        this.frameBuffer = frameBuffer;
        this.frameDimensions = frameDimensions;
    }

    public FrameData(byte[] frameBuffer, int width, int height){
        this.frameBuffer = frameBuffer;
        this.frameDimensions = new Dimension(width, height);
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
}
