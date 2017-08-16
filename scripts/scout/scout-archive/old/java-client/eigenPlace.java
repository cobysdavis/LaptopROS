import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;

public class EigenPlace {

  Image[] images;        // the raw image
  RobotPose poses[];     // pose associated with same
  
  float[][] corr;        // correlation matrix
  float[][] wts;         // the image data partially collapsed by eignenspace
  
  EigenPlace(int n) {
    images = new Image[n];
    poses = new RobotPose[n];
    corr = new float[n][n];
  }
  
  public void addPair(int id, Image im, RobotPose rp) {
    images[id] = im;
    poses[id] = rp;
  }
  
  public void addPair(int id, String ppmfile, float x, float y, float t) throws IOException {
    PPMImagery ppm = new PPMImagery(ppmfile);
    this.addPair(id, ppm.getImage(), new RobotPose(x, y, t));
  }
  
  public void readPairs(){
    System.out.println("Establishing eigenplace from disk....");
    for(int id=0;id<images.length;id++){
      System.out.println("Loading image Frame." + id);
      try {
        this.addPair(id, "Frame." + id, 0.0f, 0.0f, 0.0f);
      } catch(IOException e){
        System.out.println("Load failed with expection " + e);
      }
    }
  }
  
  private float eigenv(float[][] a, float[] b){ //compute largest eigenvector/value of a
    final int maxiter = 80;
    int n = b.length;
    float v = -1.0f;
    float[] c = new float[n];
 
    //guess at eigenvector (b) and eigenvalue (oldv)
    for(int i=0;i<n;i++)
      b[i] = (float)(1.0f / Math.sqrt((float)b.length));
    float oldv = -1.0f; 

    for(int iter=0;iter<maxiter;iter++){
      float sum = 0.0f;
      v = 0.0f;
      for(int i=0;i<n;i++){
        c[i] = 0;
        for(int j=0;j<n;j++)
	      c[i] += a[i][j] * b[j];
        sum += c[i] * c[i];
        v += b[i] * c[i];
      }

      // normalize the vector 
      for(int i=0;i<n;i++)
        b[i] = (float)(c[i] / Math.sqrt(sum));
      if(Math.abs(v - oldv) < 1e-12){
        System.out.println("Found at iter " + iter);
        return v;
      }
      oldv = v;
    }
    return v;
  }
  
  public void computeBestNEigenvectors(int n){
    float b[] = new float[images.length];
    
    for(int i=0;i<n;i++){
      float v = this.eigenv(this.corr, b);
      System.out.println("" + i + "th Largest eigenvalue is " + v);
      System.out.println("Eigenvector is " + b);
      
      
    }
  }
 /* 
  public void initializeWts(){ // initialize the wts matrix
    int width = images[0].getWidth(null);
    int height = images[0].getHeight(null);
    int size = width * height;
   // wts = new float[images.length];
  */
  
  public void computeCovariance(){ // compute transpose(p)p rather than p transpose(p)   
    // get all imagery into one array Mx(NxNx3)
    int width = images[0].getWidth(null);
    int height = images[0].getHeight(null);
    int size = width * height;
   
    System.out.println("Computing mean cell");
    Image mean = this.makeMeanImage();
    int mvalues[] = new int[width * height];
    PixelGrabber mgrabber = new PixelGrabber(mean, 0, 0, width, height, mvalues, 0, width);
	try {
	  if(mgrabber.grabPixels() != true)
		System.out.println("PixelGrabber failed");
	} catch (InterruptedException e) { ; }
    
    System.out.println("Computing the  wts matrix");
    for(int i=0;i<images.length;i++){
      int values[] = new int[width * height];
	   PixelGrabber grabber = new PixelGrabber(images[i], 0, 0, width, height, values, 0, width);
	   try {
	     if(grabber.grabPixels() != true)
		   System.out.println("PixelGrabber failed");
	   } catch (InterruptedException e) { ; }
       for(int j=0;j<width*height;j++){
         wts[i][j] = (values[j] & 0x00ff) - (mvalues[j] & 0x00ff);
         wts[i][j] = ((values[j] >> 8) & 0x00ff) - ((mvalues[j] >> 8) & 0x00ff);
         wts[i][j] = ((values[j] >> 16) & 0x00ff) - ((mvalues[j] >> 16) & 0x00ff);
       }
    }  
    
    System.out.println("Computing covariance matrix");
    for(int i=0;i<images.length;i++)for(int j=0;j<images.length;j++){
      float sum = 0.0f;
      for(int k=0;k<width*height*3;k++)
        sum += wts[i][k] * wts[j][k];
      corr[i][j] = sum;
    }  
  }
    
    
  /*
   * given a set of images, this returns the mean image
   */
  public Image makeMeanImage() {
    int width = images[0].getWidth(null);
    int height = images[0].getHeight(null);
    int rcount[] = new int[width*height];
    int gcount[] = new int[width*height];
    int bcount[] = new int[width*height];
    
    for(int i=0;i<width*height;i++)
      rcount[i] = gcount[i] = bcount[i] = 0;
    
    // compute the mean r, g, b counts 
    for(int i=0;i<images.length;i++){
      int values[] = new int[width * height];
	  PixelGrabber grabber = new PixelGrabber(images[i], 0, 0, width, height, values, 0, width);
	  try {
	    if(grabber.grabPixels() != true)
	      System.out.println("Pixelgrabber failed "  + grabber.status());
	  } catch (InterruptedException e) { ; }
      for(int j=0;j<width*height;j++){
        int r = (byte)((values[j] >> 16) & 0x00FF);
	    int g = (byte)((values[j] >> 8) & 0x00FF);
	    int b = (byte)((values[j]) & 0x00FF); 
	    rcount[j] += r;
	    gcount[j] += g;
	    bcount[j] += b;
	  }
	}
	
	// normalize and then create the output image
	for(int i=0;i<width*height;i++){
	  rcount[i] /= images.length;
	  gcount[i] /= images.length;
	  bcount[i] /= images.length;
	}
	
	// create output image
	int[] pixels = new int[width*height];
    for(int i=0;i<width*height;i++)
      pixels[i] = (255<<24)|(0xff0000 & (rcount[i]<<16))|(0xff00 & (gcount[i]<<8))|(0xff & bcount[i]);
    MemoryImageSource mis = new MemoryImageSource(width, height, pixels, 0, width);
    return Toolkit.getDefaultToolkit().createImage(mis);
  }
}