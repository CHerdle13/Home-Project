package com.ch.robofollow;

// libraries
import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import java.io.IOException;
import static android.graphics.Color.blue;
import static android.graphics.Color.colorToHSV;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640,480,Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;

    static long prevtime = 0; // for FPS calculation

    public boolean isRed(int pixel) {
        float [] hsv = new float [3];
        colorToHSV(pixel,hsv);
        if (hsv[0] > 340 && hsv [1] > .75 && hsv[2] > .5){
            return true;
        }
        else if (hsv[0] < 10 && hsv [1] > .75 && hsv[2] > .5){
            return true;
        }
        else {
            return false;
        }
        /*if (red(pixel) > 125 && green(pixel) < 100 && blue(pixel) < 100)
        {
            return true;
        }
        else if (red(pixel) > 200 && green(pixel) < 150 && blue(pixel) < 150)
        {
            return true;
        }
        else {
            return false;
        }
        */
    }
    public boolean isBlue(int pixel) {
        float [] hsv = new float [3];
        colorToHSV(pixel,hsv);
        if (hsv[0] > 200 && hsv[0] < 250 && hsv [1] > .5 && hsv[2] > .3){
            return true;
        }
        else {
            return false;
        }
    }

    public int centerOfMass(int[] pixels){
        int[] pixelWeights = new int[pixels.length];
        int wbTotal = 0; // total mass
        int wbCOM = 0; // total (mass time position)
        for (int i = 0; i < pixels.length; i++) {
            // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
            // if it is greater than some value (600 here), consider it black
            // play with the 600 value if you are having issues reliably seeing the line

            if (isBlue(pixels[i])) {
                pixelWeights[i] = 255*3;
            }
            else {
                pixelWeights[i] = 0;
            }
            wbTotal = wbTotal + pixelWeights[i];
            wbCOM = wbCOM + pixelWeights[i]*i;
        }
        int COM;
        //watch out for divide by 0
        if (wbTotal<=0) {
            COM = pixels.length/2;
        }
        else {
            COM = wbCOM/wbTotal;
        }
        return COM;
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
        mSurfaceHolder = mSurfaceView.getHolder();

        mTextureView = (TextureView) findViewById(R.id.textureview);
        mTextureView.setSurfaceTextureListener(this);

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        paint1.setColor(0xffff0000); // red
        paint1.setTextSize(24);
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
       // parameters.setColorEffect(Camera.Parameters.EFFECT_MONO); // black and white
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    public int startY = 15;

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // Invoked every time there's a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {

            int[] pixels = new int[bmp.getWidth()];
             // which row in the bitmap to analyse to read
            // only look at one row in the image
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1); // (array name, offset inside array, stride (size of row), start x, start y, num pixels to read per row, num rows to read)

            // pixels[] is the RGBA data (in black an white).
            // instead of doing center of mass on it, decide if each pixel is dark enough to consider black or white
            // then do a center of mass on the thresholded array
            /*int[] thresholdedPixels = new int[bmp.getWidth()];
            int wbTotal = 0; // total mass
            int wbCOM = 0; // total (mass time position)
            for (int i = 0; i < bmp.getWidth(); i++) {
                // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                // if it is greater than some value (600 here), consider it black
                // play with the 600 value if you are having issues reliably seeing the line
                if (255*3-(red(pixels[i])+green(pixels[i])+blue(pixels[i])) > 600) {
                    thresholdedPixels[i] = 255*3;
                }
                else {
                    thresholdedPixels[i] = 0;
                }
                wbTotal = wbTotal + thresholdedPixels[i];
                wbCOM = wbCOM + thresholdedPixels[i]*i;
            }
            int COM;
            //watch out for divide by 0
            if (wbTotal<=0) {
                COM = bmp.getWidth()/2;
            }
            else {
                COM = wbCOM/wbTotal;
            }*/

            int COM;
            COM = centerOfMass(pixels);

            int [] Ypixels = new int [bmp.getHeight()];
            bmp.getPixels(Ypixels, 0, 1, COM, 0, 1, bmp.getHeight());

            int Y;
            Y = centerOfMass(Ypixels);

            // draw a circle where you think the COM is
            canvas.drawCircle(COM, Y, 5, paint1);

            // also write the value as text
            canvas.drawText("COM = " + COM, 10, 200, paint1);
            c.drawBitmap(bmp, 0, 0, null);
            mSurfaceHolder.unlockCanvasAndPost(c);

            // calculate the FPS to see how fast the code is running
            long nowtime = System.currentTimeMillis();
            long diff = nowtime - prevtime;
            mTextView.setText("FPS " + 1000/diff);
            prevtime = nowtime;
        }
    }
}