import java.awt.*;
import java.awt.event.*;

public class ImageCanvas extends Canvas {
  private Image image;
  
  ImageCanvas(Image image) {
    setImage(image);
  }
  
  ImageCanvas(String fileName) {
    Image image = Toolkit.getDefaultToolkit().getImage(fileName);
    MediaTracker tracker = new MediaTracker(this);
    try {
      tracker.addImage(image, 0);
      tracker.waitForID(0);
    } catch(InterruptedException e) {}
    setImage(image);
  }
  
  public void setImage(Image image) {
    this.image = image;
    setSize(image.getWidth(this), image.getHeight(this));
  }
  
  public Image getImage() {
    return this.image;
  }
    
  public void paint(Graphics g) {
    g.drawImage(image, 0, 0, this);
  }
  
  public void update(Graphics g) {
    paint(g);
  }
  
  public Dimension getPreferredSize() {
    return new Dimension(image.getWidth(this), image.getHeight(this));
  }
}