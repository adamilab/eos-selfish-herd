String data;
String[] L;

class Boid
{
  //fields
  PVector pos,vel,acc,ali,coh,sep; //pos, velocity, and acceleration in a vector datatype
  float neighborhoodRadius; //radius in which it looks for fellow boids
  float maxSpeed = 4; //maximum magnitude for the velocity vector
  float maxSteerForce = .1; //maximum magnitude of the steering vector
  float h; //hue
  float sc=3; //scale factor for the render of the boid
  float flap = 0;
  float t=0;
  boolean avoidWalls = false;
   
  //constructors
  Boid(PVector inPos)
  {
    pos = new PVector();
    pos.set(inPos);
    vel = new PVector(random(-1,1),random(-1,1),random(1,-1));
    acc = new PVector(0,0,0);
    neighborhoodRadius = 100;
  }
  Boid(PVector inPos,PVector inVel,float r)
  {
    pos = new PVector();
    pos.set(inPos);
    vel = new PVector();
    vel.set(inVel);
    acc = new PVector(0,0);
    neighborhoodRadius = r;
  }
   
  void run(ArrayList bl)
  {
    t+=.1;
    flap = 10*sin(t);
    //acc.add(steer(new PVector(mouseX,mouseY,300),true));
    //acc.add(new PVector(0,.05,0));
    if(avoidWalls)
    {
      acc.add(PVector.mult(avoid(new PVector(pos.x,height,pos.z),true),5));
      acc.add(PVector.mult(avoid(new PVector(pos.x,0,pos.z),true),5));
      acc.add(PVector.mult(avoid(new PVector(width,pos.y,pos.z),true),5));
      acc.add(PVector.mult(avoid(new PVector(0,pos.y,pos.z),true),5));
      acc.add(PVector.mult(avoid(new PVector(pos.x,pos.y,300),true),5));
      acc.add(PVector.mult(avoid(new PVector(pos.x,pos.y,900),true),5));
    }
    flock(bl);
    move();
    checkBounds();
    render();
  }
   
  /////-----------behaviors---------------
  void flock(ArrayList bl)
  {
    ali = alignment(bl);
    coh = cohesion(bl);
    sep = seperation(bl);
    acc.add(PVector.mult(ali,1));
    acc.add(PVector.mult(coh,3));
    acc.add(PVector.mult(sep,1));
  }
   
  void scatter()
  {
     
  }
  ////------------------------------------
     
  void move()
  {
    vel.add(acc); //add acceleration to velocity
    vel.limit(maxSpeed); //make sure the velocity vector magnitude does not exceed maxSpeed
    pos.add(vel); //add velocity to position
    acc.mult(0); //reset acceleration
  }
   
  void checkBounds()
  {
    if(pos.x>width) pos.x=0;
    if(pos.x<0) pos.x=width;
    if(pos.y>height) pos.y=0;
    if(pos.y<0) pos.y=height;
    if(pos.z>900) pos.z=300;
    if(pos.z<300) pos.z=900;
  }
   
  void render()
  {
     
    pushMatrix();
    translate(pos.x,pos.y,pos.z);
    rotateY(atan2(-vel.z,vel.x));
    rotateZ(asin(vel.y/vel.mag()));
    stroke(h);
    noFill();
    noStroke();
    fill(h);
    //draw bird
    beginShape(TRIANGLES);
    vertex(3*sc,0,0);
    vertex(-3*sc,2*sc,0);
    vertex(-3*sc,-2*sc,0);
     
    vertex(3*sc,0,0);
    vertex(-3*sc,2*sc,0);
    vertex(-3*sc,0,2*sc);
     
    vertex(3*sc,0,0);
    vertex(-3*sc,0,2*sc);
    vertex(-3*sc,-2*sc,0);
     
    /* wings
    vertex(2*sc,0,0);
    vertex(-1*sc,0,0);
    vertex(-1*sc,-8*sc,flap);
     
    vertex(2*sc,0,0);
    vertex(-1*sc,0,0);
    vertex(-1*sc,8*sc,flap);
    */
     
    vertex(-3*sc,0,2*sc);
    vertex(-3*sc,2*sc,0);
    vertex(-3*sc,-2*sc,0);
    endShape();
    //box(10);
    popMatrix();
  }
   
  //steering. If arrival==true, the boid slows to meet the target. Credit to Craig Reynolds
  PVector steer(PVector target,boolean arrival)
  {
    PVector steer = new PVector(); //creates vector for steering
    if(!arrival)
    {
      steer.set(PVector.sub(target,pos)); //steering vector points towards target (switch target and pos for avoiding)
      steer.limit(maxSteerForce); //limits the steering force to maxSteerForce
    }
    else
    {
      PVector targetOffset = PVector.sub(target,pos);
      float distance=targetOffset.mag();
      float rampedSpeed = maxSpeed*(distance/100);
      float clippedSpeed = min(rampedSpeed,maxSpeed);
      PVector desiredVelocity = PVector.mult(targetOffset,(clippedSpeed/distance));
      steer.set(PVector.sub(desiredVelocity,vel));
    }
    return steer;
  }
   
  //avoid. If weight == true avoidance vector is larger the closer the boid is to the target
  PVector avoid(PVector target,boolean weight)
  {
    PVector steer = new PVector(); //creates vector for steering
    steer.set(PVector.sub(pos,target)); //steering vector points away from target
    if(weight)
      steer.mult(1/sq(PVector.dist(pos,target)));
    //steer.limit(maxSteerForce); //limits the steering force to maxSteerForce
    return steer;
  }
   
  PVector seperation(ArrayList boids)
  {
    PVector posSum = new PVector(0,0,0);
    PVector repulse;
    for(int i=0;i<boids.size();i++)
    {
      Boid b = (Boid)boids.get(i);
      float d = PVector.dist(pos,b.pos);
      if(d>0&&d<=neighborhoodRadius)
      {
        repulse = PVector.sub(pos,b.pos);
        repulse.normalize();
        repulse.div(d);
        posSum.add(repulse);
      }
    }
    return posSum;
  }
   
  PVector alignment(ArrayList boids)
  {
    PVector velSum = new PVector(0,0,0);
    int count = 0;
    for(int i=0;i<boids.size();i++)
    {
      Boid b = (Boid)boids.get(i);
      float d = PVector.dist(pos,b.pos);
      if(d>0&&d<=neighborhoodRadius)
      {
        velSum.add(b.vel);
        count++;
      }
    }
    if(count>0)
    {
      velSum.div((float)count);
      velSum.limit(maxSteerForce);
    }
    return velSum;
  }
   
  PVector cohesion(ArrayList boids)
  {
    PVector posSum = new PVector(0,0,0);
    PVector steer = new PVector(0,0,0);
    int count = 0;
    for(int i=0;i<boids.size();i++)
    {
      Boid b = (Boid)boids.get(i);
      float d = dist(pos.x,pos.y,b.pos.x,b.pos.y);
      if(d>0&&d<=neighborhoodRadius)
      {
        posSum.add(b.pos);
        count++;
      }
    }
    if(count>0)
    {
      posSum.div((float)count);
    }
    steer = PVector.sub(posSum,pos);
    steer.limit(maxSteerForce);
    return steer;
  }
}

class BoidList
{
  ArrayList boids; //will hold the boids in this BoidList
  float h; //for color
   
  BoidList(int n,float ih)
  {
    boids = new ArrayList();
    h = ih;
    for(int i=0;i<n;i++)
      boids.add(new Boid(new PVector(width/2,height/2,600)));
  }
   
  void add()
  {
    boids.add(new Boid(new PVector(width/2,height/2,600)));
  }
   
  void addBoid(Boid b)
  {
    boids.add(b);
  }
   
  void run(boolean aW)
  {
    for(int i=0;i<boids.size();i++) //iterate through the list of boids
    {
      Boid tempBoid = (Boid)boids.get(i); //create a temporary boid to process and make it the current boid in the list
      tempBoid.h = h;
      tempBoid.avoidWalls = aW;
      tempBoid.run(boids); //tell the temporary boid to execute its run method
    }
  }
   
  Boid getBoid(int n)
  {
    if(n<boids.size())
      return (Boid)boids.get(n);
    return null;
  }
   
  void remove(int n)
  {
    if(n<boids.size())
      boids.remove(n);
  }
   
  void remove()
  {
    if(boids.size()>0)
      boids.remove(boids.size()-1);
  }
}

int initBoidNum = 200; //amount of boids to start the program with
BoidList flock1;//,flock2,flock3;
float zoom=800,rX=0,rY=0; //camera variables
boolean smoothEdges = false,avoidWalls = false,lighting = false,movingCamera=false; //some booleans for user settings
int spaceCounter = 0; //counter variables
 
 
void setup()
{
  size(600,600,P3D);
  
  BufferedReader in;
  Socket s;
  String host = "127.0.0.1";
  
  while(data == null)
  {
    try
    {
      s = new Socket(host, 2002);
      in = new BufferedReader( new InputStreamReader( s.getInputStream() ) );
      data = in.readLine();
      L = splitTokens(data, "N");
      in.close();
    }
    catch (Exception e) { }
  }
  
  //create and fill the list of boids
  flock1 = new BoidList(initBoidNum, 255);
  //flock2 = new BoidList(100,0);
  //flock3 = new BoidList(100,128);
   
  //mousewheel code. Hacky...
  addMouseWheelListener(new java.awt.event.MouseWheelListener() {
    public void mouseWheelMoved(java.awt.event.MouseWheelEvent evt) {
      mouseWheel(evt.getWheelRotation());
  }});
}
 
void draw()
{ 
  handleCounters(); //for keypresses, etc
  handleCamera(); //all the camera code.
  background(205); //clear screen
  if(lighting) //if the user wants some lighting...
    directionalLight(255,255,255, 0, 1, -100); //...give him some lighting
  noFill();
  stroke(0);
  drawBounds();
  flock1.run(avoidWalls);
  //flock2.run(avoidWalls);
  //flock3.run(avoidWalls);
  if(smoothEdges) //if the user wants antialiasing...
    smooth();
  else
    noSmooth();
  //println(frameRate); //print the framerate
}
 
//--------------------------------
//handle keys
void keyPressed()
{
  switch (keyCode)
  {
    case UP: zoom-=10; break;
    case DOWN: zoom+=10; break;
  }
  switch (key)
  {
    case 'd': lighting = !lighting; break;
  }
}
 
void keyReleased()
{
  switch(key)
  {
  }
}
//--------------------------------
//handle mouse events
void mousePressed()
{
  switch(mouseButton)
  {
    case RIGHT: if(spaceCounter>0 && !movingCamera){rX=0;rY=0;} if(!movingCamera)spaceCounter = 30; movingCamera = true; break;
  }
}
 
void mouseReleased()
{
  switch(mouseButton)
  {
    case RIGHT: movingCamera = false; break;
  }
}
 
void mouseWheel(int delta)
{
  zoom+=delta*100;
}
//--------------------------------
 
void drawBounds()
{
  stroke(0,255,0);
  line(0,0,300,  0,height,300);
  stroke(0);
  line(0,0,900,  0,height,900);
  stroke(255,0,0);
  line(0,0,300,  width,0,300);
  stroke(0);
  line(0,0,900,  width,0,900);
   
  line(width,0,300,  width,height,300);
  line(width,0,900,  width,height,900);
  line(0,height,300,  width,height,300);
  line(0,height,900,  width,height,900);
   
  stroke(0,0,255);
  line(0,0,300,  0,0,900);
  stroke(0);
  line(0,height,300,  0,height,900);
  line(width,0,300,  width,0,900);
  line(width,height,300,  width,height,900);
}
 
void handleCounters()
{
  if(spaceCounter>0)
    spaceCounter--;
}
 
void handleCamera()
{
  //camera controls
  beginCamera();
  camera();
  //translate(b.pos.x+10,b.pos.y,b.pos.z);
  //rotateY(atan2(-b.vel.z,b.vel.x));
  //rotateZ(asin(b.vel.y/b.vel.mag()));
  if(movingCamera)
  {
    rY -= (mouseX-pmouseX)*.01;
    rX += (mouseY-pmouseY)*.01;
    //rX = (map(mouseY,0,height,0,TWO_PI)); //old camera controls
    //rY = (map(mouseX,width,0,0,TWO_PI));  //kept just in case...
  }
  rotateX(rX);
  rotateY(rY);
  translate(0,0,zoom);
  endCamera();
}

