import processing.serial.*;
import java.util.List;
import java.util.ArrayList;
import java.awt.AWTException;
import java.awt.Robot;
import java.awt.PointerInfo;
import java.awt.MouseInfo;
import java.awt.Point;
import java.awt.event.KeyEvent;
import controlP5.*;
import papaya.*;

int r,g,b;    // Used to color background
Serial port;  // The serial port object
Robot robot; //Create object from Robot class

int i = 0;  //This is an incrementing counter that increments everytime something is read from serialEvent
int mode = 0; //This variable is changed when keys are pressed and used by serialEvent() to check what to do with incoming data
int j = 0; //This is an incrementer used for populated arrays with a fixed number of samples (neutralY, proSamplesY, etc)

//Declare variables for calculating rotation angles and visualizing on cube
float xZero=1640; //center value of x channel. Determined with controller resting "housing down" on a table
float yZero=2010; //center value of y channel
float zZero=2050; // center value of z channel
float Scale=1/300.0; //sensitivity (g/mV) of accelerometer

float roll=0; //initialize variable for roll angle
float pitch=0; //initialize variable for pitch angle
float neutralRoll=0; //variable for the roll angle of neutral postion
float proRoll=0; //variable for the roll angle of pronation threshold
float supRoll=0; //variable for the roll angle of supination threshold
int[] rxheading = {0, 0}; //Used for telling GUI and Emulator when threshold has been reached

//Initialize float lists for all acceleration channels. Values will be appended 
//to these lists everytime a non null string is written to the port
FloatList x_vals;
FloatList y_vals;
FloatList z_vals;

//initialize variables to store the x,y,z accelerations when thresholds are set
float[] neutralX = new float[10]; //used to collect 10 x values for neutral...
float[] neutralY = new float[10];
float[] neutralZ = new float[10];
float[] proSamplesX = new float[10]; //Used to store 10 x values for pronation to average for a threshold
float[] proSamplesY = new float[10]; //Used to store 10 y values for pronation to average for a threshold
float[] proSamplesZ = new float[10]; //Used to store 10 z values for pronation to average for a threshold
float[] supSamplesX = new float[10]; //Used to store 10 x values for supinaton to average for a threshold
float[] supSamplesY = new float[10]; //Used to store 10 y values for supinaton to average for a threshold
float[] supSamplesZ = new float[10]; //Used to store 10 y values for supinaton to average for a threshold

// For Mouse control
int x = 0;     // Mouse pointer position
int y = 0;     // Mouse pointer position
int new_x = 0; // New mouse pointer position
int new_y = 0; // New mouse pointer position
int old_rxheading = 0;  // Previous rxheading value...

int x_window_limit = 1;
int y_window_limit = 0;

int myColor = color(255);
int c1, c2;
float n, n1;

// GUI
ControlP5 cp5;
DropdownList ddl_ports;             // available serial ports as a DropdownList
//controlP5.Button b_comm_type;
controlP5.Button b_emulator_on;
//controlP5.Button b_direction_invert;
controlP5.Button b_keyboard_mouse;
controlP5.Button b_heading_left;
controlP5.Button b_heading_stop;
controlP5.Button b_heading_right;

controlP5.Textlabel t_port_status;
controlP5.Textlabel t_heading_status;
controlP5.Textlabel t_desc_emulator_on;
//controlP5.Textlabel t_desc_direction_invert;
//controlP5.Textlabel t_desc_comm_type;
controlP5.Textlabel t_desc_emulator_type;
controlP5.Textlabel t_mouse_x;
controlP5.Textlabel t_mouse_x_value;
controlP5.Textlabel t_mouse_y;
controlP5.Textlabel t_mouse_y_value;
controlP5.Textlabel t_desc_x_min;
controlP5.Textlabel t_desc_x_max;

controlP5.Textfield t_in_x_min;
controlP5.Textfield t_in_x_max;

//controlP5.Textlabel t_desc_sensitivity;
//controlP5.Textlabel t_desc_speed;

// these variables will all get toggled when they are called to update the button text...
// so fill them with the opposite of the desired value
boolean emulator_on_toggle = true;        // 0 = off, 1 = on
//boolean direction_invert_toggle = true;   // 0 = regular, 1 = inverted
//boolean comm_type_toggle = false;         // 0 = RC, 1 = dongle
boolean keyboard_mouse_toggle = true;     // 0 = mouse, 1 = keyboard
int mouse_speed = 6;                      // fraction of display; movement per heading

float controller_sensitivity_value = 1;   // default value for sensitivity
float controller_speed_value = 5;         // default value for speed

String[] port_list;                // 
int port_number;                   // selected port
boolean port_selected = false;     // if a port has been chosen
boolean port_setup = false;        // if a port has been set up

void setup() {
//  size(1000,400);
//  background(255);
//  size(800,600,P3D); //These are for drawing Cube
//  smooth();
//  fill(255,228,225); //initialize fill color to neutral color
  x_vals = new FloatList();
  y_vals = new FloatList();
  z_vals = new FloatList();
  println(Serial.list());
//  port = new Serial(this, Serial.list()[0], 115200); //USE THIS IF NOT USING PORT SELECT DROPDOWN
  frameCount = 1; //enable use of delay()

  size(400,500);
  //frameRate(30);  // how often draw() gets called...
  
  // -------------------
  // GUI
   
  noStroke();
  
  cp5 = new ControlP5(this);
  
  // Text label for heading status
  t_heading_status = cp5.addTextlabel("heading_status", "", 10, 60); // 10, 160
  
  // Buttons to show status of command being received
  b_heading_left = cp5.addButton("left")
    .setValue(0)
    .setPosition(240, 57) // 240, 157
    .setSize(20, 20)
    .setId(1);
  
  b_heading_stop = cp5.addButton("stop")
    .setValue(0)
    .setPosition(270, 57) // 270, 157
    .setSize(20, 20)
    .setId(2);
  
  b_heading_right = cp5.addButton("right")
    .setValue(0)
    .setPosition(300, 57) // 300, 157
    .setSize(20, 20)
    .setId(3);
  
  // Text label for emulator on/off
  t_desc_emulator_on = cp5.addTextlabel("desc_emulator_on", "", 10, 100);
  
  // Button to start/stop toy controller emulation
  b_emulator_on = cp5.addButton("emulator_on")
    .setValue(0)
    .setPosition(240, 92)
    .setSize(50, 30)
    .setId(0);
  
  // Text label for direction invert
//  t_desc_direction_invert = cp5.addTextlabel("desc_direction_invert", "", 10, 140);
  
  //
//  b_direction_invert = cp5.addButton("direction_invert")
//    .setValue(0)
//    .setPosition(240, 132)
//    .setSize(100, 30)
//    .setId(0);
  
  // Text label for communication type
//  t_desc_comm_type = cp5.addTextlabel("desc_comm_type", "", 10, 180); // 10, 140
  
  // Button to toggle between communication type - rc or toy
//  b_comm_type = cp5.addButton("comm_type")
//    .setValue(0)
//    .setPosition(240, 172) // 240, 112
//    .setSize(100, 30)
//    .setId(4);
  
  // Text label for emulator type
  t_desc_emulator_type = cp5.addTextlabel("desc_emulator_type", "", 10, 220); // 10, 180
  
  // Button to toggle between keyboard and mouse emulation, 'keyboard_mouse'
  b_keyboard_mouse = cp5.addButton("keyboard_mouse")
    .setValue(0)
    .setPosition(240, 212) // 240, 112
    .setSize(100, 30)
    .setId(4);
  
  // text labels for current mouse position
  t_mouse_x = cp5.addTextlabel("mouse_x", "", 240, 252); // 212
  t_mouse_x_value = cp5.addTextlabel("mouse_x_value", "", 265, 252);
  t_mouse_y = cp5.addTextlabel("mouse_y", "", 240, 272);
  t_mouse_y_value = cp5.addTextlabel("mouse_y_value", "", 265, 272);
  
  // text label for x-min
  t_desc_x_min = cp5.addTextlabel("desc_x_min", "", 10, 320); // 280
  
  // text input for x-min
  t_in_x_min = cp5.addTextfield("in_x_min")
    .setValue(0)
    .setPosition(240, 312)
    .setSize(100, 30);
  t_in_x_min.setInputFilter(ControlP5.INTEGER);
  
  // text label for x-max
  t_desc_x_max = cp5.addTextlabel("desc_x_max", "", 10, 360); // 320
  
  // text input for x-max
  t_in_x_max = cp5.addTextfield("in_x_max")
    .setValue(0)
    .setPosition(240, 352)
    .setSize(100, 30);
  t_in_x_max.setInputFilter(ControlP5.INTEGER);
  
  
  // Text label for mouse sensitivity/ response speed
//  t_desc_sensitivity = cp5.addTextlabel("desc_sensitivity","", 10, 420); // 10, 220
  
  // Slider to toggle mouse responsiveness
  cp5.addSlider("controller_sensitivity")
    .setMin(0)
    .setMax(10)
    .setValue(controller_sensitivity_value)
    .setSize(140, 30)
    .setPosition(240, 412) // 240, 212
    .setNumberOfTickMarks(11);
    
  // Text label for speed
//  t_desc_speed = cp5.addTextlabel("desc_speed","", 10, 460); // 10, 220
  
  // Slider to toggle mouse responsiveness
  cp5.addSlider("controller_speed")
    .setMin(0)
    .setMax(10)
    .setValue(controller_speed_value)
    .setSize(140, 30)
    .setPosition(240, 452) // 240, 212
    .setNumberOfTickMarks(11);   


  // --- this must be down here...
  
  // Text label for status of port  
  t_port_status = cp5.addTextlabel("port_status","",10,35);
  
  // DropdownList to display the values
  //ports = cp5.addDropdownList("ports-list",10,30,180,84);
  ddl_ports = cp5.addDropdownList("dropdownlist_ports")
    .setPosition(10,30)
    .setWidth(380)
    .setHeight(140);
  
  // This function can be used to refresh the COM ports in the list
  customize_dropdownlist(ddl_ports);
  
  
  PFont pfont = createFont("Arial", 24, false);
  ControlFont font = new ControlFont(pfont, 12);
  cp5.setControlFont(font);
  

  
  t_port_status.setColorValue(0xFF0303);
  t_port_status.setValue("Not connected");

  t_heading_status.setColorValue(0xFF0303);
  t_heading_status.setValue("Controller heading status: ");
  
//  t_desc_comm_type.setColorValue(0xFF0303);
//  t_desc_comm_type.setValue("Toggle communication (not being used)");
  
  t_desc_emulator_on.setColorValue(0xFF0303);
  t_desc_emulator_on.setValue("Toggle emulator ON/OFF");
  
//  t_desc_direction_invert.setColorValue(0xFF0303);
//  t_desc_direction_invert.setValue("Invert direction?");
  
  t_desc_emulator_type.setColorValue(0xFF0303);
  t_desc_emulator_type.setValue("Toggle emulator Keyboard/Mouse");
  
  t_mouse_x.setColorValue(0xFF0303);
  t_mouse_x.setValue("x:");
  
  t_mouse_x_value.setColorValue(0xFF0303);
  t_mouse_x_value.setValue("0");
  
  t_mouse_y.setColorValue(0xFF0303);
  t_mouse_y.setValue("y:");
  
  t_mouse_y_value.setColorValue(0xFF0303);
  t_mouse_y_value.setValue("0");
  
  t_desc_x_min.setColorValue(0xFF0303);
  t_desc_x_min.setValue("Mouse horizontal left limit (x min):");
  
  t_in_x_min.setValue("0");
  
  t_desc_x_max.setColorValue(0xFF0303);
  t_desc_x_max.setValue("Mouse horizontal right limit (x max):");
  
  t_in_x_max.setValue("" + displayWidth);
  
//  t_desc_sensitivity.setColorValue(0xFF0303);
//  t_desc_sensitivity.setValue("Sensitivity (not implemented)");
//  
//  t_desc_speed.setColorValue(0xFF0303);
//  t_desc_speed.setValue("Speed (not implemented)");
   
  //b_emulator_on.captionLabel()
  //  .setText("ON");
  
  cp5.getController("emulator_on")
    .getCaptionLabel();
  
//  cp5.getController("direction_invert")
//    .getCaptionLabel();
  
  //b_comm_type.captionLabel()
  //  .setText("Dongle");
  
//  cp5.getController("comm_type")
//    .getCaptionLabel();
  
  //b_keyboard_mouse.captionLabel()
  //  .setText("Keyboard");

  cp5.getController("keyboard_mouse")
    .getCaptionLabel();
  
  cp5.getController("left")
    .getCaptionLabel();
  b_heading_left.captionLabel()
    .setText("");
    
  cp5.getController("stop")
    .getCaptionLabel();
  b_heading_stop.captionLabel()
    .setText("");
    
  cp5.getController("right")
    .getCaptionLabel();
  b_heading_right.captionLabel()
    .setText("");
  
  /*
  cp5.getController("dropdownlist_ports")
    //.getCaptionLabel();
    .setFont(font)
    .toUpperCase(false);
   */
     
  // -------------------
  
  // DEBUG
  //println("Display width:     " + displayWidth);
  //println("Cursor increment:  " + displayWidth/mouse_speed);
  
  
  // -------------------
  
  // Setup the robot to move the mouse around...
  try {
    robot = new Robot();
  }
  catch(AWTException e) {
    e.printStackTrace();
  }
  
  // Center the mouse
  robot.mouseMove(displayWidth/2, displayHeight/2);

}

void draw() {
  background(255);
  //println(rxheading);
//  int[] rxheading = {0, 0}; //Defined globally instead
  int heading = 0;
  
  while(port_selected == true && port_setup == false)
  {
    // DEBUG
    //println("starting serial port");
    start_serial(port_list);
  }
  
    /**
   * Heading codes:
   *
   * 0: Nothing
   * 3: Forward (right, because we are in the Northern Hemisphere...)
   * 6: Reverse (left)
   */  
  if (rxheading[0] >= 0) {
    if (rxheading[1] == 3) {
      heading = 1;
      b_heading_right.setColorBackground(color(13, 208, 255));  // Active
      b_heading_left.setColorBackground(color(90));   // Inactive
      b_heading_stop.setColorBackground(color(90));   // Inactive
    } else if (rxheading[1] == 6) {
      heading = -1;
      b_heading_right.setColorBackground(color(90));  // Inactive
      b_heading_left.setColorBackground(color(13, 208, 255));   // Active
      b_heading_stop.setColorBackground(color(90));   // Inactive
    } else {
      heading = 0;
      b_heading_right.setColorBackground(color(90));  // Inactive
      b_heading_left.setColorBackground(color(90));   // Inactive
      b_heading_stop.setColorBackground(color(13, 208, 255));   // Active
    }
  }

    /**
   * Control the Keyboard (0) or the Mouse (1) depending on the state
   * of the variable keyboard_mouse_toggle
   *
   */
  
  // If the emulator is not running, do nothing...
  if(!emulator_on_toggle) {
    return;
  }
  
  // Check if we are going to emulate the mouse or the keyboard...
  if (!keyboard_mouse_toggle) {
    // If we are moving the mouse pointer...
    PointerInfo a = MouseInfo.getPointerInfo();
    // Catch NullPointerException in case the pointer disappears...
    if (a != null) {
      Point b = a.getLocation();
      x = (int) b.getX();
      y = (int) b.getY();
      
      /*
      // Move the x-coordinate of the mouse baed on the controller
      new_x = x + (heading * mouse_speed * (displayWidth / 100));
      if (new_x < x_window_limit) {
        new_x = x_window_limit;
      } else if (new_x > (displayWidth - x_window_limit)) {
        new_x = displayWidth - x_window_limit;
      }
      */                 
      
      // Move the x-coordinate of the mouse 0-99
      if (rxheading[0] >= 0) {
        
        //if (abs(rxheading[0] - old_rxheading) > controller_sensitivity_value) {
        if (abs(rxheading[0] - old_rxheading) > 1) {
          
          new_x = (displayWidth * rxheading[0] / 100);
          
          //println("" + cp5.getController("in_x_min").getValue());
          
          //if (new_x < (int) cp5.getController("in_x_min").getText()) {
          //  new_x = (int) cp5.getController("in_x_min").getText();
          //} else if (new_x > (int) cp5.getController("in_x_max").getText()) {
          //  new_x = (int) cp5.getController("in_x_max").getText();
          //}
          
          if (new_x < Integer.parseInt(t_in_x_min.getText())) {
            new_x = Integer.parseInt(t_in_x_min.getText());
          } else if (new_x > Integer.parseInt(t_in_x_max.getText())) {
            new_x = Integer.parseInt(t_in_x_max.getText());
          }
          
          old_rxheading = rxheading[0];
        
      } else {
          new_x = x;
        }
        //println(new_x);
        //new_x = x;
      } else {
        new_x = x;
      }
      //if (new_x < x_window_limit) {
      //  new_x = x_window_limit;
      //} else if (new_x > (displayWidth - x_window_limit)) {
      //  new_x = displayWidth - x_window_limit;
      //}
      
      // Keep the y-coordinate the same
      new_y = y;
      
      t_mouse_x_value.setValue(""+new_x);
      t_mouse_y_value.setValue(""+new_y);
      
      // This is a little choppy...
      robot.mouseMove(new_x, new_y);
    }
  } else if (keyboard_mouse_toggle) {
    // Toggle the keys
    if (heading == 1) {
      robot.keyPress(KeyEvent.VK_RIGHT);
      robot.keyRelease(KeyEvent.VK_RIGHT);
    } else if (heading == -1) {
      robot.keyPress(KeyEvent.VK_LEFT);
      robot.keyRelease(KeyEvent.VK_LEFT);
    } else {
      // Do nothing...
      ;
    }
  }
  
}
void keyReleased() {
    if (key =='1') {
    mode=1;
    j=0; //starts j over to be incremented with each iteration of serialEvent()
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Neutral Samples (y):");
  } else if (key == '2') {
    mode=2;
    j=0; //starts j over to be incremented with each iteration of serialEvent()
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Pronation Samples (y):");
  } else if (key == '3') {
    mode=3;
    j=0; //starts j over to be incremented with each iteration of serialEvent()
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Supination Samples (y):");
  } else if (key == '4') { //Starts "continuous" collection of acceleration data
    j=0;
    mode = 4; //Tells serialEvent we are in collectDynamic() mode
    port.write("adcplay"); //Tells controller to collect data continuously
    port.bufferUntil('\n');
    port.write("\n");
  } else {
    j=0; //starts j over to be incremented with each iteration of serialEvent()
    mode=5;
    port.write("stop");
    port.bufferUntil('\n'); 
    port.write("\n");
  }
}

void collectDynamic() {
  float x1 = 0;
  float y1 = 0;
  float z1 = 0;
  float xAcc=0; //x acceleration in g's
  float yAcc=0; //y acceleration in g's 
  float zAcc=0; //z acceleration in g's

    x1=x_vals.get(i-1);
    y1=y_vals.get(i-1);
    z1=z_vals.get(i-1);
    xAcc=(x1-xZero)*Scale; //Converts voltage to acceleration in g's
    yAcc=(y1-yZero)*Scale; //Converts voltage to acceleration in g's
    zAcc=(z1-zZero)*Scale; //Converts voltage to acceleration in g's

//Calculation of roll and pitch angle (4 options using    
//  //Aerospace rotation sequence
    roll=atan(yAcc/zAcc); //Approximation of roll angle in radians based on aerospace rotation sequence
    pitch=atan(-xAcc/sqrt(pow(yAcc,2)+pow(zAcc,2)));
//  //Aerospace rotation sequence (corrected)
//    roll=atan(yAcc/(zAcc/abs(zAcc)*sqrt(pow(zAcc,2)+.01*pow(xAcc,2)))); //Approximation of roll angle in radians based on aerospace rotation sequence
//    pitch=atan(-xAcc/sqrt(pow(yAcc,2)+pow(zAcc,2))); //Approximation of roll angle in radians
//  //Non-Aerospace rotation sequence    
//    roll=atan(yAcc/sqrt(pow(xAcc,2)+pow(zAcc,2))); //Approximation of roll angle in radians based on aerospace rotation sequence
//    pitch=atan(-xAcc/zAcc); 
//  //Non-Aerospace rotation sequence (corrected)
//    pitch=atan(-xAcc/(zAcc/abs(zAcc)*sqrt(pow(zAcc,2)+.01*pow(yAcc,2)))); //Approximation of roll angle in radians based on aerospace rotation sequence
//    roll=atan(yAcc/sqrt(pow(xAcc,2)+pow(zAcc,2))); //Approximation of roll angle in radians
    
//println(pitch*180/PI); //prints pitch angle in degrees
println(roll*180/PI); //prints roll (supination/pronation) angle in degrees

//Check current roll angle against thresholds
    if ((roll < proRoll || roll>supRoll) && yAcc<0) { //This OR statement is simply to adress the fact the pronation past 90degrees should still count.  yAcc<0 excludes actual supination
      //println("Pronated");
      //fill(255,0,0);
      rxheading[1] = 6; // CURRENTLY CORRESPONDS to RIGHT HEADER
    } else if (roll > supRoll && zAcc>0) {
      //println("Supinated");
      //fill(0,0,255);
      rxheading[1] = 3; // CURRENTLY CORRESPONDS TO LEFT HEADER
    } else {
      //println("neutral");
      //fill(255,228,225);
      rxheading[1] = 0; // CURRENTLY CORRESPONDS TO LEFT HEADER
    }

}

void serialEvent (Serial myPort) {
  // Read string until carriage return and save as accelString
  String accelString = myPort.readStringUntil('\n'); //defines accelString as a single line of output from the terminal
//Parsing
  if (accelString != null) {
    try {
      float[] accelVals = float(split(accelString, ',')); //splits line based on comma delimiter
      println(accelVals);
      if (!Float.isNaN(accelVals[0])) { //If a value for the acceleration was output 
        if mode=4 { //If we are using adcplay
          x_vals.append(accelVals[1]);
          y_vals.append(accelVals[2]);
          z_vals.append(accelVals[3]);
        } else {
          x_vals.append(accelVals[0]);
          y_vals.append(accelVals[1]);
          z_vals.append(accelVals[2]);
        }
          
        i = i + 1;   //Increments list index if actual acceleration values were appended
        if (mode==1) {
          neutralX[j]=x_vals.get(i-1);
          neutralY[j]=y_vals.get(i-1);
          neutralZ[j]=z_vals.get(i-1);
          if (j==9) { //if all 10 samples have been collected
            float[] neutralAvg={Descriptive.mean(neutralX), Descriptive.mean(neutralY), Descriptive.mean(neutralZ)};
            float[] neutralAcc={(neutralAvg[0]-xZero)*Scale, (neutralAvg[1]-yZero)*Scale, (neutralAvg[2]-zZero)*Scale};
//            neutralRoll=atan(neutralAcc[1]/(neutralAcc[2]/abs(neutralAcc[2])*sqrt(pow(neutralAcc[2],2)+.01*pow(neutralAcc[0],2)))); //Approximation of roll angle in radians based on corrected aerospace rotation sequence
            neutralRoll=atan(neutralAcc[1]/neutralAcc[2]); //uncorrected aerospace 
//            neutralRoll=atan(neutralAcc[1]/sqrt(pow(neutralAcc[0],2)+pow(neutralAcc[2],2))); //Approximation of roll angle in radians based on aerospace rotation sequence 
            println(neutralRoll*180/PI);
          }
          j=j+1;
        } else if (mode==2) {
          proSamplesX[j]=x_vals.get(i-1);
          proSamplesY[j]=y_vals.get(i-1);
          proSamplesZ[j]=z_vals.get(i-1);
          if (j==9) { //if all 10 samples have been collected
            float[] proAvg={Descriptive.mean(proSamplesX), Descriptive.mean(proSamplesY), Descriptive.mean(proSamplesZ)};
            float[] proAcc={(proAvg[0]-xZero)*Scale, (proAvg[1]-yZero)*Scale, (proAvg[2]-zZero)*Scale};
//            proRoll=atan(proAcc[1]/(proAcc[2]/abs(proAcc[2])*sqrt(pow(proAcc[2],2)+.01*pow(proAcc[0],2)))); //Approximation of roll angle in radians based on corrected aerospace rotation sequence
            proRoll=atan(proAcc[1]/proAcc[2]); //uncorrected aerospace rotation sequence
//            proRoll=atan(proAcc[1]/sqrt(pow(proAcc[0],2)+pow(proAcc[2],2)));
            println(proRoll*180/PI);
          }
          j=j+1;
        } else if (mode==3) {
          supSamplesX[j]=x_vals.get(i-1);
          supSamplesY[j]=y_vals.get(i-1);
          supSamplesZ[j]=z_vals.get(i-1);
          if (j==9) { //if all 10 samples have been collected
            float[] supAvg={Descriptive.mean(supSamplesX), Descriptive.mean(supSamplesY), Descriptive.mean(supSamplesZ)};
            float[] supAcc={(supAvg[0]-xZero)*Scale, (supAvg[1]-yZero)*Scale, (supAvg[2]-zZero)*Scale};
//            supRoll=atan(supAcc[1]/(supAcc[2]/abs(supAcc[2])*sqrt(pow(supAcc[2],2)+.01*pow(supAcc[0],2)))); //Approximation of roll angle in radians based on aerospace rotation sequence
            supRoll=atan(supAcc[1]/supAcc[2]); //uncorrected aerospace
//            supRoll=atan(supAcc[1]/sqrt(pow(supAcc[0],2)+pow(supAcc[2],2)));
            println(supRoll*180/PI);
          }
          j=j+1;
        } else if (mode==4) {
          collectDynamic();
        }
      }
      
    }    catch(Exception e) {
      //println(e);
    }
  }

}
  
/** 
 * The function start_serial initializes the selected port
 *
 */
void start_serial(String[] port_list)
{  
  port_setup = true;
  println(port_list[port_number]);
  try {
    port = new Serial(this, port_list[port_number], 115200);
    //port.bufferUntil(',');
  }
  catch(RuntimeException e) {
    port_setup = false;
    port_selected = false;
    // This color does not update...
    t_port_status.setColorValue(0xFF0000);
    t_port_status.setValue("Error: cannot connect to port");
  }
  
  // Update port settings
  if (port_setup == true) {
    t_port_status.setColorValue(0x030303);
    t_port_status.setValue("Connected to port " + port_list[port_number]);
    // Send a carriage return to the port because sometimes it stops printing...
    port.write(13);
  }
}

/** 
 * GUI features
 *
 */
/* Dropdownlist event */
public void controlEvent(ControlEvent theEvent)
{
  //println(theEvent.getController().getName());
  
  if (theEvent.isGroup())
  {
    // Store the value of which box was selected
    float p = theEvent.group().value();
    port_number = int(p);
    println(port_number);
    // TODO: stop the serial connection and start a new one
    port_selected = true;
  }
}

/* Function emulator_on */
public void emulator_on(int theValue)
{
  emulator_on_toggle = !emulator_on_toggle;
  cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
  //println("on ?:" + emulator_on_toggle);
  if (emulator_on_toggle)
  {
    // The emulator is turned ON, send a keystroke to the serial port in case data has paused...
    port.write(13);
  }
}

///* Function direction_invert */
//public void direction_invert(int theValue)
//{
////  direction_invert_toggle = !direction_invert_toggle;
//  cp5.controller("direction_invert").setCaptionLabel((direction_invert_toggle == true) ? "INVERT":"NORMAL");
//  //println("dongle ?:" + comm_type_toggle);
//}
//
///* Function comm_type */
//public void comm_type(int theValue)
//{
//  comm_type_toggle = !comm_type_toggle;
//  cp5.controller("comm_type").setCaptionLabel((comm_type_toggle == true) ? "DONGLE":"RC TOY");
//  //println("dongle ?:" + comm_type_toggle);
//}


/* Function keyboard_mouse */
public void keyboard_mouse(int theValue)
{
  keyboard_mouse_toggle = !keyboard_mouse_toggle;
  cp5.controller("keyboard_mouse").setCaptionLabel((keyboard_mouse_toggle == true) ? "Keyboard":"Mouse");
  //println("keyboard ?:" + keyboard_mouse_toggle);
}

/* Function in_x_min text field */
public void in_x_min(float theValue)
{
  //cp5.controller("in_x_min").setValue("" + theValue);
  t_in_x_min.setValue("" + theValue);
}

/* Function in_x_max text field */
public void in_x_max(float theValue)
{
  //cp5.controller("in_x_max").setValue("" + theValue);
  t_in_x_max.setValue("" + theValue);
}

///* Function mouse sensitivity */
//public void controller_sensitivity(float controller_sensitivity_value)
//{
//  controller_sensitivity_value = cp5.getController("controller_sensitivity").getValue();
//}
//
//public void controller_speed(float controller_speed_value)
//{
//  controller_speed_value = cp5.getController("controller_speed").getValue();
//}


/* Setup the DropdownList */
void customize_dropdownlist(DropdownList ddl)
{
  //
  ddl.setBackgroundColor(color(200));
  ddl.setItemHeight(20);
  ddl.setBarHeight(20);
  ddl.captionLabel().set("Select COM port");
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
  
  // Store the serial port list in the string port_list (char array)
  port_list = port.list();
  
  for (int i = 0; i < port_list.length; i++) {
    ddl.addItem(port_list[i], i);
  }
  
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}
