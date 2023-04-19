// need to import these to make buttons and use wifi
import controlP5.*;
import processing.net.*;


// sets up pointers to what will be your buttons and your client
ControlP5 buttons;
// using 3 clients, 1 for start/stop and distances, 2 for buggySpeed, 3 for angle
Client Client1 ;
Client Client2 ;


// If obstacle is a new thing or not
int ch2;

int buggySpeed = 0;
int angle = 0;
int hght = 0;

// Strings of variables
String bugSpeed, hight, ang;

// Strings of variables with respective units
String Buggy, heightFinal, angleFinal ;

// Strings of units
String cmps = "cm/s";
String cm = "cm" ;
String deg = "°" ;

// Declare ackground image
PImage bg;

float DistGraphmm = 0;
float DistChangemm;
float distGraph;
int wdth = 184;
int SIZE = 2000;
float[] grapHeight = new float[SIZE];
float[] graphDist = new float[SIZE];
int index = 0;
void setup() {
  //Must be size of bg
  size(512, 512);

  frameRate(20);
  strokeWeight(2);


  bg = loadImage("C:\\Users\\matth\\Downloads\\Idea_1.png");

  buttons = new ControlP5(this);
  //plug your ip address from your Client1 in here, make sure the final port numbers match
  Client1 = new Client(this, "192.168.72.185", 5700);
  Client2 = new Client(this, "192.168.72.185", 6800);



  //create new buttons for go and stop
  buttons.addButton("Start")
    .setColorBackground(color(50, 50, 50))
    .setValue(0)
    .setPosition(322, 278)
    .setSize(25, 25);

  buttons.addButton("Finish")
    .setColorBackground(color(50, 50, 50))
    .setValue(0)
    .setPosition(441, 278)
    .setSize(25, 25);

  buttons.addButton("RESET")
    .setColorBackground(color(41, 41, 41))
    .setValue(0)
    .setPosition(279, 285)
    .setSize(20, 20);
}

void draw() {
  background(bg);

  while (Client1.available() > 0) {
    ch2 = Client1.read();
    println(ch2);
  }

  if (Client2 != null) {
    Client2.write('2');
  }

  while (Client2.available() > 0) {
    String data = Client2.readStringUntil('\n');
    if (data != null) {
      String[] values = split(data, ',');
      buggySpeed = int(values[0]);
      angle = int(values[1]);
      hght = int(values[2]);
    }
  }

  // Fill arrays
  DistChangemm = float(buggySpeed / 2);
  DistGraphmm = DistGraphmm + DistChangemm;
  distGraph = DistGraphmm /10 ;
  //println(distGraph);
  grapHeight[index] = (hght) + (distGraph * .184);
  graphDist[index] = distGraph ;
  // Show buggy speed
  fill(205);
  stroke(205);
  rect(363, 264, 59, 26);

  bugSpeed = str(buggySpeed) ;
  Buggy = bugSpeed + cmps;
  textSize(15);
  textAlign(RIGHT);
  fill(50);
  text(Buggy, 413, 282);

  // Show elevation angle
  textSize(15);
  textAlign(LEFT);
  fill(92, 5, 18);
  text("Incline angle:", 365, 130);

  ang = str(angle) ;
  angleFinal = ang + deg;
  textAlign(RIGHT);
  fill(92, 5, 18);
  text(angleFinal, 474, 130);

  // Show altitude
  textAlign(LEFT);
  fill(92, 5, 18);
  text("Altitude:", 395, 145);

  hight = str(hght) ;
  heightFinal = hight + cm;
  textAlign(RIGHT);
  fill(92, 5, 18);
  text(heightFinal, 489, 145);

  // DESIGN OF GRAPHS
  beginShape();
  fill(96);
  stroke(96);
  vertex(25, 130);
  vertex(209, 137);
  vertex(209, 218);
  vertex(25, 252);
  endShape();

  beginShape(LINES);
  stroke(255);
  for (int i = 1; i < index; i++ ) {
    line(graphDist[i-1] + 25, height - (grapHeight[i-1] + 260), graphDist[i] + 25, height - (grapHeight[i] + 260)) ;
  }
  endShape(OPEN);





  strokeWeight(3);
  line (20, 258, 209, 223);
  line (20, 258, 20, 130);

  pushMatrix();
  textSize(12);
  textAlign(LEFT);
  rotate(-.18);
  fill(255);
  text("Distance (cm)", 50, 270);
  popMatrix();

  pushMatrix();
  textSize(12);
  textAlign(LEFT);
  //rotate(-1.572);
  rotate(-1.572);
  fill(255);
  text("Height (cm)", -230, 15);
  popMatrix();

  pushMatrix();
  textSize(12);
  textAlign(LEFT);
  rotate(-.18);
  fill(255);
  text("180", 150, 270);
  popMatrix();

  pushMatrix();
  textSize(12);
  textAlign(LEFT);
  //rotate(-1.572);
  rotate(-1.572);
  fill(255);
  text("115", -150, 15);
  popMatrix();

  pushMatrix();
  textSize(13);
  textAlign(LEFT);
  fill(255);
  text("0", 15, 270);
  popMatrix();


  index = index + 1;
  //Make arrays right
  if (index >= SIZE - 1||  distGraph >= 200) {
    for (int i = 1; i <  -1; i++ ) {
      grapHeight[i] = 0;
      graphDist[i] = 0 ;
    }
    index = 0;
  }
}


//functions for what processing sends to Client1 when go and stop are pressed
public void Start() {

  if (Client1.active()) {
    Client1.write("w");
    println("go") ;
  }
}

public void Finish() {
  if (Client1.active()) {
    Client1.write("s");
    println("stop") ;
  }
}

public void RESET() {
  if (Client1.active()) {
    Client1.write("r");
    index = 0;
    DistGraphmm = 0;
    distGraph = 0;

    for (int i = 1; i < graphDist.length -1; i++ ) {
      grapHeight[i] = 0;
      graphDist[i] = 0 ;
    }
  }
}
