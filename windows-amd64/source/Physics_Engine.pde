import java.util.Arrays;
float xForce = 100;
float yForce = 100;
color[] colors = {color(79, 77, 191), color(16, 213, 228), color(236, 124, 38), color(40, 230, 125), color(62, 240, 90),
                  color(250, 210, 1), color(133, 85, 171), color(149, 25, 185), color(86, 242, 28), color(2, 208, 221),
                  color(239, 202, 146), color(186, 116, 216), color(92, 168, 255), color(215, 240, 178), color(255, 47, 0),
                  color(230, 95, 202), color(109, 48, 21), color(155, 236, 181), color(245, 49, 200), color(55, 142, 28),
                  color(246, 255, 0), color(0, 251, 255), color(0, 255, 51), color(0, 255, 255), color(59, 255, 0)};
Shape[] objects;
PVector tangent = new PVector(0, 0);
PVector normal = new PVector(0, 0);
PVector direction = new PVector(0, 0);
PVector[] impulseList = {new PVector(0, 0), new PVector(0, 0)};
PVector[] frictionImpulseList = {new PVector(0, 0), new PVector(0, 0)};
float[] jList = {0.0, 0.0};
PVector[] raList = {new PVector(0, 0), new PVector(0, 0)};
PVector[] rbList = {new PVector(0, 0), new PVector(0, 0)};
PVector[] contactPointsArr = {new PVector(0, 0), new PVector(0, 0)};
float magnitude = 0;
float updateTime = 1/frameRate;
float gravity = 9.81;
boolean gravityMode = true;
int wallThickness = 30;
float circleRadius = 30;
float squareSide = pow((circleRadius * circleRadius * PI), 0.5);
int iterations = 20;
boolean collisionState = false;
int[][] contactPairsList = {};
//used for performance measurement
PFont metricsDisplay;
float currTimer = 0;
long stepTimer = 0; //counts the number of milliseconds between each update
float currUpdateTime = 0;
float currSampleCount = 0;
String stepTimerString = "0.0";
int terrainOption = 0;

float lengthOfVector(PVector a){
  if(abs(a.x) == 0.0){
    return a.y;
  }
  if(abs(a.y) == 0.0){
    return a.x;
  }
  return pow(a.x * a.x + a.y * a.y, 0.5);
}

float lengthOfVectorSquared(PVector a){
  return a.x * a.x + a.y * a.y;
}

float lengthSquared(float a, float b){
  return a * a + b * b;
}

float crossProduct(PVector a, PVector b){
  return a.x * b.y - a.y * b.x;
}

float dotProduct(PVector a, PVector b){
  return a.x * b.x + a.y * b.y;
}

float dotProductFloats(float ax, float ay, float bx, float by){
  return ax * bx + ay * by;
}

boolean intersectingAABBs(PVector[] shapeA, PVector[] shapeB){
  //conditions for intersecting AABBs:
  //(L1 <= R2 && L2 <= R1) && (B1 >= T2 && B2 >= T1) 
  //it would usually be B1 <= T2 && B2 <= T1, but that only works if y increases upwards. So if y increases downwards, flip the comparisons

  //checks if any max value is less than any min value
  return (shapeA[0].x <= shapeB[1].x && shapeB[0].x <= shapeA[1].x && shapeA[1].y >= shapeB[0].y && shapeB[1].y >= shapeA[0].y); 
}

PVector[] getAABB(Shape shape){
  //res stores top left and bottom right coordinates for each shape
  PVector res[] = new PVector[2];
  
  //gets the max and min vectors
  if(shape.shapeType == "Circle"){
    res[0] = new PVector(shape.center.x - shape.r, shape.center.y - shape.r);
    res[1] = new PVector(shape.center.x + shape.r, shape.center.y + shape.r);
  }else if(shape.shapeType == "Box"){
    PVector minVal = new PVector(pow(2, 31) - 1, pow(2, 31) - 1);
    PVector maxVal = new PVector(-1 * pow(2, 31), -1 * pow(2, 31));
    
    for(int i = 0; i < shape.vertices.length; i++){
      if(maxVal.x < shape.transformedVertices[i].x){
        maxVal.x = shape.transformedVertices[i].x;
      }
      if(maxVal.y < shape.transformedVertices[i].y){
        maxVal.y = shape.transformedVertices[i].y;
      }
      if(minVal.x > shape.transformedVertices[i].x){
        minVal.x = shape.transformedVertices[i].x;
      }
      if(minVal.y > shape.transformedVertices[i].y){
        minVal.y = shape.transformedVertices[i].y;
      }
    }
    
    res[0] = minVal; //Top Left
    res[1] = maxVal; //Bottom Right
  }
  return res;
}

boolean outOfBounds(Shape shape){
  //gets the AABB of the current shape (min[0] and max[1] points for the x and y coordinates)
  PVector[] currAABB = getAABB(shape);
  //checks the leftmost point, rightmost point, and topmost point
  return (currAABB[0].x >= width) || (currAABB[1].x <= 0) || (currAABB[0].y >= height);
}

void resetAndUpdateBodies(){
  //resets each object and moves it accordingly from the last update
  for(int i = 0; i < objects.length; i++){
    //skips static objects
    if(!objects[i].isMovable){
      continue;
    }
    ///gets the AABB for non-static objects
    objects[i].currAABB = getAABB(objects[i]);
    objects[i].getTransformedVertices();
        
    //checks if the object is still moving
    if(objects[i].force.x != 0 || objects[i].force.y != 0 || gravityMode == true){
      objects[i].updateForceVelocityAngle(updateTime, iterations);
    }
    //removes objects that are out of bounds
    if(outOfBounds(objects[i])){
      for(int j = i; j < objects.length - 1; j++){
        objects[j] = objects[j + 1];
      }
      objects = Arrays.copyOf(objects, objects.length - 1);
    }
  }
}

void generateStaticObjects(int configNum){
  switch(configNum){
    case 1: 
      objects = new Shape[3];
      //float x, float y, color c, boolean movable, float re, float radius, float width_, float height_, String type
      //bottom floor
      objects[0] = new Shape(width/2, height - wallThickness * 3, color(0, 150, 0), false, 0.5, 0, width - wallThickness * 4, wallThickness, "Box");
      objects[0].currAABB = getAABB(objects[0]);
      //left slope
      objects[1] = new Shape(300, 450, color(150, 0, 0), false, 0.5, 0, wallThickness * 16, wallThickness, "Box");
      objects[1].turn(20);
      objects[1].getTransformedVertices();
      objects[1].currAABB = getAABB(objects[1]);
      //right slope
      objects[2] = new Shape(700, 200, color(150, 150, 150), false, 0.5, 0, wallThickness * 16, wallThickness, "Box");
      objects[2].turn(-20);
      objects[2].getTransformedVertices();
      objects[2].currAABB = getAABB(objects[2]);
      break;
    case 2: 
      objects = new Shape[10];
      //float x, float y, color c, boolean movable, float re, float radius, float width_, float height_, String type
      //left steep slope
      objects[0] = new Shape(55, height/2 - 60, color(0, 150, 150), false, 0.5, 0, height - wallThickness * 18, wallThickness, "Box");
      objects[0].turn(-95);
      objects[0].getTransformedVertices();
      objects[0].currAABB = getAABB(objects[0]);
      //left slope
      objects[1] = new Shape(110, height - 280, color(0, 255, 150), false, 0.5, 0, wallThickness * 6, wallThickness, "Box");
      objects[1].turn(-120);
      objects[1].getTransformedVertices();
      objects[1].currAABB = getAABB(objects[1]);
      //left gradual slope
      objects[2] = new Shape(222, height - 174, color(150, 0, 0), false, 0.5, 0, wallThickness * 6, wallThickness, "Box");
      objects[2].turn(20);
      objects[2].getTransformedVertices();
      objects[2].currAABB = getAABB(objects[2]);
      //bottom middle slope (left)
      objects[3] = new Shape(500, height - 150, color(255, 150, 150), false, 0.5, 0, wallThickness * 14, wallThickness, "Box");
      objects[3].currAABB = getAABB(objects[3]);
      //bottom right gradual slope
      objects[4] = new Shape(width - 222, height - 174, color(255, 150, 0), false, 0.5, 0, wallThickness * 6, wallThickness, "Box");
      objects[4].turn(-20);
      objects[4].getTransformedVertices();
      objects[4].currAABB = getAABB(objects[4]);
      
      //right steep slope
      objects[5] = new Shape(width - 157, height/2 - 240, color(150, 150, 0), false, 0.5, 0, height - wallThickness * 22, wallThickness, "Box");
      objects[5].turn(95);
      objects[5].getTransformedVertices();
      objects[5].currAABB = getAABB(objects[5]);
      //right slope
      objects[6] = new Shape(width - 210, height - 500, color(150, 255, 0), false, 0.5, 0, wallThickness * 6, wallThickness, "Box");
      objects[6].turn(120);
      objects[6].getTransformedVertices();
      objects[6].currAABB = getAABB(objects[6]);
      //right gradual slope
      objects[7] = new Shape(width - 322, height - 394, color(0, 0, 150), false, 0.5, 0, wallThickness * 6, wallThickness, "Box");
      objects[7].turn(-20);
      objects[7].getTransformedVertices();
      objects[7].currAABB = getAABB(objects[7]);
      //bottom middle slope (right)
      objects[8] = new Shape(width - 500, height - 370, color(150, 150, 255), false, 0.5, 0, wallThickness * 8, wallThickness, "Box");
      objects[8].currAABB = getAABB(objects[8]);
      
      
      //top middle slope
      objects[9] = new Shape(400, 200, color(150, 150, 150), false, 0.5, 0, wallThickness * 16, wallThickness, "Box");
      objects[9].turn(-20);
      objects[9].getTransformedVertices();
      objects[9].currAABB = getAABB(objects[9]);
      break;
    default:
      objects = new Shape[1];
      //bottom floor
      objects[0] = new Shape(width/2, height - wallThickness * 3, color(0, 150, 0), false, 0.5, 0, width - wallThickness * 4, wallThickness, "Box");
      objects[0].currAABB = getAABB(objects[0]);
  }
}

void setup(){
  size(1000, 800);
  stroke(250);
  strokeWeight(2);
  metricsDisplay = createFont("Helvetica", 20, true);
  textFont(metricsDisplay, 20);
  rectMode(CENTER);
  
  generateStaticObjects(terrainOption);
}

void draw(){
  background(170, 225, 225);
  //used for performance measurement
  currTimer++;
  if(currTimer >= frameRate){  
    stepTimerString = str(currUpdateTime/currSampleCount);
    currSampleCount = 0;
    currUpdateTime = 0;
    currTimer = 0;
  }
  
  stepTimer = millis();

  //goes through iterations (the iterations chop up big movements into smaller ones)
  for(int it = 0; it < iterations; it++){
    
    resetAndUpdateBodies();
    
    //resets the lists of contacts every cycle
    contactPairsList = Arrays.copyOf(contactPairsList, 0);
    
    findPotentialCollisions();
    checkPotentialCollisions();
  }
  
  //shows and updates all objects within the space
  for(int i = 0; i < objects.length; i++){
    objects[i].show();
    objects[i].update();
  }
  
  //used for measuring performance
  stepTimer = millis() - stepTimer;
  currUpdateTime += stepTimer;
  currSampleCount++;
  fill(0);
  text("Body Count: " + objects.length, 20, height - 20);
  text("World Step Time: " + stepTimerString + "ms", 20, height - 50);
}


void keyPressed(){
  if(key == '1'){
    float w = random(squareSide * 3/4, squareSide * 3/2);
    float h = random(squareSide * 3/4, squareSide * 3/2);
    objects = Arrays.copyOf(objects, objects.length + 1);
    objects[objects.length - 1] = new Shape(mouseX, mouseY, colors[int(random(colors.length))], true, 0.5, 0, w, h, "Box");
    objects[objects.length - 1].currAABB = getAABB(objects[objects.length - 1]);
  }
  if(key == '2'){
    float r = random(circleRadius * 3/4, circleRadius * 3/2);
    objects = Arrays.copyOf(objects, objects.length + 1);
    objects[objects.length - 1] = new Shape(mouseX, mouseY, colors[int(random(colors.length))], true, 0.5, r, 0, 0, "Circle");
    objects[objects.length - 1].currAABB = getAABB(objects[objects.length - 1]);
  }
  if(key == 'c'){
    objects = Arrays.copyOf(objects, 0);
    generateStaticObjects(terrainOption);
  }
  if(keyCode == RIGHT){
    terrainOption = (terrainOption + 1) % 3;
    generateStaticObjects(terrainOption);
  }
  if(keyCode == LEFT){
    terrainOption = (terrainOption - 1) % 3;
    generateStaticObjects(terrainOption);
  }
}

/* 
current benchmark:
100 boxes in the world (20 iterations, 60 fps)
- before caching AABBs: ~75ms per step
- after caching AABBs: ~23ms per step

Notes: 
- 26 Jan: made the Circle and Rectangle Classes, implemented the SAT algorithm for polygons
- 27 Jan: implemented the SAT algorithm for circles and mixed shapes (circle & polygon)
- 27 Jan: added collision resolution for all three cases, fixed some bugs, added movement options
- 28 Jan: added forces and velocities, finished collision resolutions for all cases (added extra movement after collision)
- 28 Jan: added some helper functions
- 29 Jan: added the ability to generate random sizes of boxes and circles, increased stability of engine (added iterations)
- 29 Jan: implemented gravity system
- 29 Jan: restructured the main function, added getAABB and outOfBounds, added manifold class and metric display
- 30 Jan: added recognition for contact points for all classes
- 30 Jan: added rotational inertia for all objects, added line on circle to show its rotation state
- 30 Jan: added two slopes, fixed the finding contact points function (was comparing the contact to the distance (wrong value))
- 31 Jan: optimized the code so now the program runs over 3 times faster using the current benchmark 
(cached the AABBs of every object at the beginning instead of recomputing them every time in the double for loop)
- 1 Feb: Restructured the code, added rotational physics
- 2 Feb: attempted to add friction
- 3 Feb: finished adding friction

https://youtube.com/playlist?list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs&feature=shared (main source used for this project)

extra note: some aspects are different from the original source to suit my style of coding
*/
