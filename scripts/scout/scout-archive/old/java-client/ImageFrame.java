import java.awt.*;
import java.awt.event.*;
import java.applet.*;
import java.net.*;

// adapted from Java Examples in a nutshell
public class ImageFrame extends Frame implements WindowListener {
  private ImageCanvas imc;
  
  
  ImageFrame(String title, Image im) {
    super(title); 
  
    // with the label
    this.setLayout(new BorderLayout(15,15));
    imc = new ImageCanvas(im);
   
    this.add("Center", imc);
    this.setResizable(false);
    this.pack();
    this.addWindowListener(this);
    
    Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
    move((screenSize.width-size().width)/2, (screenSize.height-size().height)/2);
  }
 
  public void setVisible(boolean v) { // make it visible
   super.setVisible(v);
  }
  
  
  // winodw events (most ignored)
  public void windowClosing(WindowEvent e) { 
    this.dispose();
  }
  public void windowOpened(WindowEvent e) {}
  public void windowClosed(WindowEvent e) {}
  public void windowIconified(WindowEvent e) {}
  public void windowDeiconified(WindowEvent e) {}
  public void windowActivated(WindowEvent e) {}
  public void windowDeactivated(WindowEvent e) {} 
}
  
  
    
