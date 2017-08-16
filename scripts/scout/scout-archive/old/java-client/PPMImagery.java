import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;

public class PPMImagery {
  // supported image types
  static final int IS_GREYMAP    = 1;
  static final int IS_FULLCOLOUR = 2;
  
  private byte[] red, green, blue;                 // the image (as 8 bit colour chanels)
  private int width, height;                       // image dimensions (in pixels)
  private int type;                                // the image type
  
  PPMImagery(String filename) throws IOException { // constructor which takes a filename
    this(new FileInputStream(filename));
  }
    
  PPMImagery(InputStream in) throws IOException { // constructor which takes an InputStream
    String s = "";
    String  t;
    StringTokenizer st = new StringTokenizer(s);
    
    // get the tokens which make up the header. Ignore any comments
    while(st.countTokens() != 4){  
      t = getline(in);
      if(t.startsWith("#"))
        continue;  
      s = s + t + " ";
      st = new StringTokenizer(s);
    }
    
    String header = st.nextToken();
    int width = Integer.parseInt(st.nextToken());
    int height = Integer.parseInt(st.nextToken());
    int depth = Integer.parseInt(st.nextToken());
    
    if(header.equals("P5"))
      P5reader(width, height, depth, in);
    else if(header.equals("P6"))
      P6reader(width, height, depth, in);
    else

      throw new IOException("Unsupported PPM file type");
  }
  
  PPMImagery(Image im) throws IOException { // constructor that takes an image
    this.type = IS_FULLCOLOUR;
    this.width = im.getWidth(null);
	this.height = im.getHeight(null);

	int values[] = new int[width * height];
	PixelGrabber grabber = new PixelGrabber(im, 0, 0, this.width, this.height, values, 0, this.width);
	try {
	  if(grabber.grabPixels() != true)
		throw new IOException("Grabber returned false: " + grabber.status());
	} catch (InterruptedException e) { ; }
	
	red = new byte[width*height];
	green  = new byte[width*height];
	blue = new byte[width*height];
	for(int i=0;i<width*height;i++){
	  red[i] = (byte)((values[i] >> 16) & 0xFF);
	  green[i] = (byte)((values[i] >> 8) & 0xFF);
	  blue[i] = (byte)((values[i]) & 0xFF);  
    }
  }
  
  public Image getImage() { // turn the current image into a Java Image
    int[] pixels = new int[width*height];
    for(int i=0;i<width*height;i++)
      pixels[i] = (255<<24)|(0xff0000 & (red[i]<<16))|(0xff00 & (green[i]<<8))|(0xff & blue[i]);
    MemoryImageSource mis = new MemoryImageSource(width, height, pixels, 0, width);
    return Toolkit.getDefaultToolkit().createImage(mis);
  }
  
  public int getImageType() {  // return the image type
    return this.type;
  }
  
  public byte[] getGrey () { // only defined for IS_GREYMAP images
    return this.red;
  }  
  
  public byte[] getRed () { // only defined for IS_FULLCOLOUR images
    return this.red;
  }  
  
  public byte[] getGreen () { // only defined for IS_FULLCOLOUR images
    return this.green;
  }  

  public byte[] getBlue () { // only defined for IS_FULLCOLOUR images
    return this.blue;
  }  
  
  public void writeImage(String s) throws IOException {
    this.writeImage(new FileOutputStream(s));
  }
  
  public void writeImage(OutputStream out) throws IOException { 
    switch(this.type){
    case IS_GREYMAP:
      P5writer(width, height, 255, out);
      break;
    case IS_FULLCOLOUR:
      P6writer(width, height, 255, out);
      break;
    default:
      throw new IOException("Unsupported PPM file type");
    }
  }
    
  private void P6writer(int width, int height, int depth, OutputStream out) throws IOException {
    writeline(out, "P6 " + width + " " + height + " " + depth);
    
    byte[] tmp = new byte[width*height*3];
    for(int i=0;i<width*height;i++){
      tmp[3*i] = red[i];
      tmp[3*i+1] = green[i];
      tmp[3*i+2] = blue[i];
    }
    out.write(tmp);
    out.close();
  }
 
  private void P5writer(int width, int height, int depth, OutputStream out) throws IOException {
    writeline(out, "P5 " + width + " " + height + " " + depth);
    out.write(this.red);
    out.close();
  }
  
  private void writeline(OutputStream out, String s) throws IOException { // write a line
    while(s.length() > 0){
      out.write((byte)s.charAt(0));
      s = s.substring(1);
    }
    out.write((byte) 10);
  }
  
  private String getline(InputStream in) throws IOException { // read one line of the header
    String s = "";
    int ch;
    
    while((ch = in.read()) != -1){
      if(ch == 10)
        break;
      if(ch ==  13)
        continue;
      s = s + (char) ch;
    }    
    return s;
  }


  // read in a P6 PPM (raw colourmap)
  private void P6reader(int width, int height, int depth, InputStream in) throws IOException {
    int bytesread, n;
    if(depth > 255)
      throw new IOException("Unsupported PPM file type (> 8 bits/pixel)");
    this.type = IS_FULLCOLOUR;
    this.width = width;
    this.height =  height;
    
    red = new byte[width*height];
    green = new byte[width*height];
    blue = new byte[width*height];
    
    byte[] temp = new byte[width*height*3];
    bytesread = 0;
    while((n = in.read(temp, bytesread, 3*height*width-bytesread)) > 0){
     // System.out.println("read " + (int)(bytesread+n)  + " of " + 3 * width * height);
      bytesread += n;
        
    }
    if(bytesread != 3 * height * width)
        throw new IOException("Early end of file on PPM file");
    
    for(int i=0;i<width*height;i++){
      red[i] = temp[3*i+0];
      green[i] = temp[3*i+1];
      blue[i] = temp[3*i+2];
    }
      
    if(depth != 255) { // scale all values into range 0..255
      for(int i=0;i<width*height;i++){
        red[i] = (byte)((float)red[i] * 255.0 / depth);
        green[i] = (byte)((float)red[i] * 255.0 / depth);
        blue[i] = (byte)((float)red[i] * 255.0 / depth);
      }
    }
    in.close();
  }

  // read in a P5 PPM (raw greymap)
  private void P5reader(int width, int height, int depth, InputStream in) throws IOException {
    int bytesread, n;
    
    if(depth > 255)
      throw new IOException("Unsupported PPM file type (> 8 bits/pixel)");
    this.type = IS_GREYMAP;
    this.width = width;
    this.height =  height;
    
    red = new byte[width*height];
    green = red;
    blue = red;
    
    bytesread = 0;
    while((n = in.read(red, bytesread, width*height-bytesread)) > 0)
      bytesread += n;
    if(bytesread != width * height)
      throw new IOException("Early end of file on PPM file");
      
    if(depth != 255) { // scale all values into range 0..255
      for(int i=0;i<width*height;i++)
        red[i] = (byte)((float)red[i] * 255.0 / depth);
    }
    in.close();
  }
}
