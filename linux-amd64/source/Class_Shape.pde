class Shape{
  PVector center;
  float angle = 0;
  PVector linearVelocity = new PVector(0, 0);
  float angularVelocity = 0;
  PVector force = new PVector(0, 0);
  PVector forceDirection = new PVector(0, 0);
  color currColor;
  boolean isMovable;
  float restitution; //coefficient that determines the elasticity of the collision between two objects
  float mass = 0;
  float invMass = 0;
  float density = 1;
  float r;
  PVector edgePercent = new PVector(1, 0); //edgePercent is only used to draw the line on the circle
  float w, h;
  PVector[] vertices;
  PVector[] transformedVertices;
  String shapeType;
  float rotationalInertia = 0;
  float invRotationalInertia = 0;
  PVector[] currAABB;
  float staticFriction = 0.6;
  float dynamicFriction = 0.4;
  
  
  Shape(float x, float y, color c, boolean movable, float re, float radius, float width_, float height_, String type){
    center = new PVector(x, y);
    currColor = c;
    isMovable = movable;
    shapeType = type;
    restitution = re;
    
    if(type == "Circle"){
      r = radius;
      mass = r * r * PI * density;
      rotationalInertia = 1.0/2.0 * mass * (r * r);
    }else if(type == "Box"){
      w = width_;
      h = height_;
      vertices = new PVector[4];
      vertices[0] = new PVector(x - w/2, y - h/2);
      vertices[1] = new PVector(x + w/2, y - h/2);
      vertices[2] = new PVector(x + w/2, y + h/2);
      vertices[3] = new PVector(x - w/2, y + h/2);
      transformedVertices = new PVector[4];
      transformedVertices[0] = new PVector(x - w/2, y - h/2);
      transformedVertices[1] = new PVector(x + w/2, y - h/2);
      transformedVertices[2] = new PVector(x + w/2, y + h/2);
      transformedVertices[3] = new PVector(x - w/2, y + h/2);
      mass = w * h * density;
      rotationalInertia = 1.0/12.0 * mass * (h * h + w * w);
    }
        
    if(movable){
      invMass = 1 / mass;
      invRotationalInertia = 1 / rotationalInertia;
    }
  }
  
  void show(){
    fill(currColor);
    stroke(250);
    //if(isColliding && isMovable){
    //  //fill(red);
    //}
    if(shapeType == "Circle"){
      circle(center.x, center.y, r * 2);
      line(center.x, center.y, center.x + edgePercent.x * r, center.y + edgePercent.y * r);
    }else if(shapeType == "Box"){
      quad(transformedVertices[0].x, transformedVertices[0].y, 
           transformedVertices[1].x, transformedVertices[1].y, 
           transformedVertices[2].x, transformedVertices[2].y, 
           transformedVertices[3].x, transformedVertices[3].y);
    }
    //stroke(0);
    //noFill();
    //rect(currAABB[0].x + (currAABB[1].x - currAABB[0].x)/2, currAABB[0].y + (currAABB[1].y - currAABB[0].y)/2, 
    //     currAABB[1].x - currAABB[0].x, currAABB[1].y - currAABB[0].y);
  }
  
  void update(){
    center.x += linearVelocity.x;
    if(shapeType == "Box"){
      for(int i = 0; i < vertices.length; i++){
        vertices[i].x += linearVelocity.x;
      }
    }
    center.y += linearVelocity.y;
    if(shapeType == "Box"){
      for(int i = 0; i < vertices.length; i++){
        vertices[i].y += linearVelocity.y;
      }
    }
  }
  
  void move(float xDisp, float yDisp){
    center.x += xDisp;
    center.y += yDisp;
    if(shapeType == "Box"){
      for(int i = 0; i < vertices.length; i++){
        vertices[i].x += xDisp;
        vertices[i].y += yDisp;
      }
    }
    updateAABB(xDisp, yDisp);
  }
  
  void turn(float degrees){
    float temp; //temp ensures that the y component is still using the original x component (not the transformed one)
    if(shapeType == "Circle"){
      temp = edgePercent.x * cos(radians(degrees)) - edgePercent.y * sin(radians(degrees));
      edgePercent.y = edgePercent.x * sin(radians(degrees)) + edgePercent.y * cos(radians(degrees));
      edgePercent.x = temp;
    }else if(shapeType == "Box"){
      for(int i = 0; i < vertices.length; i++){
        temp = (vertices[i].x - center.x) * cos(radians(degrees)) - (vertices[i].y - center.y) * sin(radians(degrees)) + center.x;
        vertices[i].y = (vertices[i].x - center.x) * sin(radians(degrees)) + (vertices[i].y - center.y) * cos(radians(degrees)) + center.y;
        vertices[i].x = temp;
      }
    }
  }
  
  void getTransformedVertices(){
    if(shapeType == "Box"){
      for(int i = 0; i < vertices.length; i++){
        transformedVertices[i].x = (vertices[i].x - center.x) * cos(angle) - (vertices[i].y - center.y) * sin(angle) + center.x;
        transformedVertices[i].y = (vertices[i].x - center.x) * sin(angle) + (vertices[i].y - center.y) * cos(angle) + center.y;
      }
    }
    else if(shapeType == "Circle"){
      edgePercent.x = cos(angle);
      edgePercent.y = sin(angle);
    }
  }
  
  void addForce(float x, float y){
    force.x += x;
    force.y += y;
  }
  
  void updateForceVelocityAngle(float time, int iterations){
    //force = mass * acceleration
    //acceleration = force / mass
    
    float updateTime = time / (float) iterations;
    
    if(gravityMode == true){ //gravity is acceleration, so no need to divide by mass again
      linearVelocity.y += gravity * updateTime;
    }
    
    linearVelocity.x += force.x * invMass * updateTime;
    linearVelocity.y += force.y * invMass * updateTime;

    angle += angularVelocity * time;
    
    force.x = 0;
    force.y = 0;
  }
  
  void updateAABB(float xDisp, float yDisp){
    currAABB[0].x += xDisp;
    currAABB[0].y += yDisp;
    currAABB[1].x += xDisp;
    currAABB[1].y += yDisp;
  }
  
  void damping(){
    linearVelocity.x *= 0.99; 
    linearVelocity.y *= 0.99;
  }
}
