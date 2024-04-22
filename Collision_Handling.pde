void findPotentialCollisions(){
  //goes through each object
  for(int i = 0; i < objects.length - 1; i++){
    
    //goes through every other object
    for(int j = i + 1; j < objects.length; j++){
      //skips both objects if they are both static
      if(!objects[i].isMovable && !objects[j].isMovable){
        continue;
      }
      //skips if the AABBs of both objects aren't intersecting
      if(!intersectingAABBs(objects[i].currAABB, objects[j].currAABB)){
        continue;
      }
      
      contactPairsList = Arrays.copyOf(contactPairsList, contactPairsList.length + 1);
      contactPairsList[contactPairsList.length - 1] = new int[2];
      contactPairsList[contactPairsList.length - 1][0] = i;
      contactPairsList[contactPairsList.length - 1][1] = j;
    }
  }
}

void checkPotentialCollisions(){
  Shape objectA;
  Shape objectB;
  for(int i = 0; i < contactPairsList.length; i++){
    objectA = objects[contactPairsList[i][0]];
    objectB = objects[contactPairsList[i][1]];
    //checks for all collision cases (box & box, circle & circle, box & circle)
    if(objectA.shapeType == "Box" && objectB.shapeType == "Box"){
      collisionState = isIntersecting(objectA.transformedVertices, objectB.transformedVertices, objectA.center, objectB.center);

    }else if(objectA.shapeType == "Circle" && objectB.shapeType == "Circle"){
      collisionState = areCirclesIntersecting(objectA.center, objectB.center, objectA.r, objectB.r);

    }else{
      //objectA is the circle, objectB is the box
      if(objectB.shapeType == "Circle"){
        //swaps the objects so the references line up with the next bit of code
        Shape temp = objectB;
        objectB = objectA;
        objectA = temp;
        int temp2 = contactPairsList[i][1];
        contactPairsList[i][1] = contactPairsList[i][0];
        contactPairsList[i][0] = temp2;
      }
      collisionState = isCircleAndPolygonIntersecting(objectA.center, objectA.r, objectB.transformedVertices, objectB.center);
    }
    //checks if a collision has happened
    if(collisionState){
      Manifold contact = handleCollision(contactPairsList[i][0], contactPairsList[i][1]);
      resolveCollisionComplexWithFriction(contact);
    }
  }
}

Manifold handleCollision(int i, int j){

  //checks if both objects are movable or if one or the other is static (moves both objects out of each other)
  if(!objects[i].isMovable){
    objects[j].move(direction.x * magnitude, direction.y * magnitude);
  }else if(!objects[j].isMovable){
    objects[i].move(-1 * direction.x * magnitude, -1 * direction.y * magnitude);
  }else{
    objects[i].move(-1 * direction.x * magnitude / 2.0, -1 * direction.y * magnitude / 2.0);
    objects[j].move(direction.x * magnitude / 2.0, direction.y * magnitude / 2.0);
  }
  
  PVector[] pointsAndCount = findContactPoints(objects[i], objects[j]);
  
  return new Manifold(objects[i], objects[j], direction, magnitude, pointsAndCount, pointsAndCount.length);
}

void resolveCollisionBasic(Manifold contact){
  //unpacks all the variables for use
  Shape shapeA = contact.shapeA;
  Shape shapeB = contact.shapeB;
  PVector currDirection = contact.direction;
  
  //returns early if both shapes are static (no velocities will be applied)
  if(!shapeA.isMovable && !shapeB.isMovable){
    return;
  }
  
  //complex math stuffs
  //equation 1: v1 = v0 + j / m * n;     note: j = impulse, n = normal
  //equation 2: j = -(1 + e) * vAB1 * n / dotProduct(n, n(1/mA + 1/mB));
  //note: vAB1 = relative velocity, e = restitution coefficient
  
  //gets the difference in velocities between both shapes
  PVector relativeVelocity = new PVector(shapeB.linearVelocity.x - shapeA.linearVelocity.x, shapeB.linearVelocity.y - shapeA.linearVelocity.y);
  
  if(dotProduct(relativeVelocity, currDirection) > 0){
    return;
  }
  
  float e = min(shapeA.restitution, shapeB.restitution);
  float j = -(1 + e) * dotProduct(relativeVelocity, currDirection) / (shapeA.invMass + shapeB.invMass);
  PVector impulse = new PVector(j * currDirection.x, j * currDirection.y);
     
  //applies the velocity to shapeA if it's movable
  if(shapeA.isMovable){
    shapeA.linearVelocity.x += -1 * impulse.x * shapeA.invMass;
    shapeA.linearVelocity.y += -1 * impulse.y * shapeA.invMass;
  }
  //applies the velocity to shapeB if it's movable
  if(shapeB.isMovable){
    shapeB.linearVelocity.x += impulse.x * shapeB.invMass;
    shapeB.linearVelocity.y += impulse.y * shapeB.invMass;
  }
}

void resolveCollisionComplex(Manifold contact){
  Shape shapeA = contact.shapeA;
  Shape shapeB = contact.shapeB;
  PVector currDirection = contact.direction;
  contactPointsArr[0] = contact.contactPoints[0];
  if(contact.contactCount > 1){
    contactPointsArr[1] = contact.contactPoints[1];
  }
  
  PVector ra;
  PVector rb;
  PVector raPerpendicular;
  PVector rbPerpendicular;

  for(int i = 0; i < 2; i++){
    impulseList[i].x = 0;
    impulseList[i].y = 0;
    raList[i].x = 0;
    raList[i].y = 0;
    rbList[i].x = 0;
    rbList[i].y = 0;
  }
  
  PVector impulse;
  
  float e = min(shapeA.restitution, shapeB.restitution);
  
  for(int i = 0; i < contact.contactCount; i++){
    ra = new PVector(contactPointsArr[i].x - shapeA.center.x, contactPointsArr[i].y - shapeA.center.y);
    rb = new PVector(contactPointsArr[i].x - shapeB.center.x, contactPointsArr[i].y - shapeB.center.y);
    
    raList[i] = ra;
    rbList[i] = rb;
    
    raPerpendicular = new PVector(-1 * ra.y, ra.x);
    rbPerpendicular = new PVector(-1 * rb.y, rb.x);

    PVector combinedVelocityA = new PVector(raPerpendicular.x * shapeA.angularVelocity, raPerpendicular.y * shapeA.angularVelocity);
    PVector combinedVelocityB = new PVector(rbPerpendicular.x * shapeB.angularVelocity, rbPerpendicular.y * shapeB.angularVelocity);

    PVector relativeVelocity = new PVector((shapeB.linearVelocity.x + combinedVelocityB.x) - (shapeA.linearVelocity.x + combinedVelocityA.x), 
                                           (shapeB.linearVelocity.y + combinedVelocityB.y) - (shapeA.linearVelocity.y + combinedVelocityA.y));
  
    float raPerpDotN = dotProduct(raPerpendicular, currDirection);
    float rbPerpDotN = dotProduct(rbPerpendicular, currDirection);
    
    float contactVelocityMag = dotProduct(relativeVelocity, currDirection);
    
    if(contactVelocityMag > 0){
      continue;
    }

    float j = -(1 + e) * contactVelocityMag;
    j /= shapeA.invMass + shapeB.invMass + 
         (raPerpDotN * raPerpDotN) * shapeA.invRotationalInertia + 
         (rbPerpDotN * rbPerpDotN) * shapeB.invRotationalInertia;
         
    j /= contact.contactCount;      
         
     impulse = new PVector(j * currDirection.x, j * currDirection.y);
    
    impulseList[i] = impulse; 
  }
  
  for(int i = 0; i < contact.contactCount; i++){
    impulse = impulseList[i];
    ra = raList[i];
    rb = rbList[i];
    
    //applies the velocity to shapeA if it's movable
    if(shapeA.isMovable){
      shapeA.linearVelocity.x += -1 * impulse.x * shapeA.invMass;
      shapeA.linearVelocity.y += -1 * impulse.y * shapeA.invMass;
      
      shapeA.angularVelocity += -1 * crossProduct(ra, impulse) * shapeA.invRotationalInertia;
    }
    //applies the velocity to shapeB if it's movable
    if(shapeB.isMovable){
      shapeB.linearVelocity.x += impulse.x * shapeB.invMass;
      shapeB.linearVelocity.y += impulse.y * shapeB.invMass;
      
      shapeB.angularVelocity += crossProduct(rb, impulse) * shapeB.invRotationalInertia;
    }
  }
}

//doesn't work
void resolveCollisionComplexWithFriction(Manifold contact){
  Shape shapeA = contact.shapeA;
  Shape shapeB = contact.shapeB;
  PVector currDirection = contact.direction;
  contactPointsArr[0] = contact.contactPoints[0];
  if(contact.contactCount > 1){
    contactPointsArr[1] = contact.contactPoints[1];
  }
    
  PVector ra;
  PVector rb;
  PVector raPerpendicular;
  PVector rbPerpendicular;

  for(int i = 0; i < 2; i++){
    impulseList[i].x = 0;
    impulseList[i].y = 0;
    frictionImpulseList[i].x = 0;
    frictionImpulseList[i].y = 0;
    jList[i] = 0.0;
    raList[i].x = 0;
    raList[i].y = 0;
    rbList[i].x = 0;
    rbList[i].y = 0;
  }
  
  PVector impulse;
  PVector frictionImpulse;
  
  float e = min(shapeA.restitution, shapeB.restitution);
  
  float staticFrictionConst = (shapeA.staticFriction + shapeB.staticFriction) * 0.5;
  float dynamicFrictionConst = (shapeA.dynamicFriction + shapeB.dynamicFriction) * 0.5;
    
  for(int i = 0; i < contact.contactCount; i++){
    ra = new PVector(contactPointsArr[i].x - shapeA.center.x, contactPointsArr[i].y - shapeA.center.y);
    rb = new PVector(contactPointsArr[i].x - shapeB.center.x, contactPointsArr[i].y - shapeB.center.y);
    
    raList[i] = ra;
    rbList[i] = rb;
    
    raPerpendicular = new PVector(-1 * ra.y, ra.x);
    rbPerpendicular = new PVector(-1 * rb.y, rb.x);
    
    PVector combinedVelocityA = new PVector(raPerpendicular.x * shapeA.angularVelocity, raPerpendicular.y * shapeA.angularVelocity);
    PVector combinedVelocityB = new PVector(rbPerpendicular.x * shapeB.angularVelocity, rbPerpendicular.y * shapeB.angularVelocity);

    PVector relativeVelocity = new PVector((shapeB.linearVelocity.x + combinedVelocityB.x) - (shapeA.linearVelocity.x + combinedVelocityA.x), 
                                           (shapeB.linearVelocity.y + combinedVelocityB.y) - (shapeA.linearVelocity.y + combinedVelocityA.y));
  
    float raPerpDotN = dotProduct(raPerpendicular, currDirection);
    float rbPerpDotN = dotProduct(rbPerpendicular, currDirection);
    
    float contactVelocityMag = dotProduct(relativeVelocity, currDirection);
        
    if(contactVelocityMag > 0){
      continue;
    }

    float j = -(1 + e) * contactVelocityMag;
    j /= shapeA.invMass + shapeB.invMass + 
         (raPerpDotN * raPerpDotN) * shapeA.invRotationalInertia + 
         (rbPerpDotN * rbPerpDotN) * shapeB.invRotationalInertia;
         
    j /= contact.contactCount;   
    
    jList[i] = j;
         
    impulse = new PVector(j * currDirection.x, j * currDirection.y);
    impulseList[i] = impulse; 
  }
  
  for(int i = 0; i < contact.contactCount; i++){
    impulse = impulseList[i];
    ra = raList[i];
    rb = rbList[i];
    
    //applies the velocity to shapeA if it's movable
    if(shapeA.isMovable){
      shapeA.linearVelocity.x += -1 * impulse.x * shapeA.invMass;
      shapeA.linearVelocity.y += -1 * impulse.y * shapeA.invMass;
      
      shapeA.angularVelocity += -1 * crossProduct(ra, impulse) * shapeA.invRotationalInertia;
    }
    //applies the velocity to shapeB if it's movable
    if(shapeB.isMovable){
      shapeB.linearVelocity.x += impulse.x * shapeB.invMass;
      shapeB.linearVelocity.y += impulse.y * shapeB.invMass;
      
      shapeB.angularVelocity += crossProduct(rb, impulse) * shapeB.invRotationalInertia;
    }
  }

  //calculates friction
  for(int i = 0; i < contact.contactCount; i++){
    ra = new PVector(contactPointsArr[i].x - shapeA.center.x, contactPointsArr[i].y - shapeA.center.y);
    rb = new PVector(contactPointsArr[i].x - shapeB.center.x, contactPointsArr[i].y - shapeB.center.y);
    
    raList[i] = ra;
    rbList[i] = rb;
    
    raPerpendicular = new PVector(-1 * ra.y, ra.x);
    rbPerpendicular = new PVector(-1 * rb.y, rb.x);
    
    PVector combinedVelocityA = new PVector(raPerpendicular.x * shapeA.angularVelocity, raPerpendicular.y * shapeA.angularVelocity);
    PVector combinedVelocityB = new PVector(rbPerpendicular.x * shapeB.angularVelocity, rbPerpendicular.y * shapeB.angularVelocity);

    PVector relativeVelocity = new PVector((shapeB.linearVelocity.x + combinedVelocityB.x) - (shapeA.linearVelocity.x + combinedVelocityA.x), 
                                           (shapeB.linearVelocity.y + combinedVelocityB.y) - (shapeA.linearVelocity.y + combinedVelocityA.y));
    
    float projectedVelocity = dotProduct(relativeVelocity, currDirection);
    
    PVector tangent = new PVector(relativeVelocity.x - projectedVelocity * currDirection.x,
                                  relativeVelocity.y - projectedVelocity * currDirection.y);
                                                                                            
    if(almostEqualVectors(tangent, new PVector(0, 0))){
      continue;
    }else{
      float tangentLen = lengthOfVector(tangent);
      tangent.x /= tangentLen;
      tangent.y /= tangentLen;
    }
        
    float raPerpDotT = dotProduct(raPerpendicular, tangent);
    float rbPerpDotT = dotProduct(rbPerpendicular, tangent);

    float contactVelocityMagT = dotProduct(relativeVelocity, tangent);

    float jT = -1 * contactVelocityMagT;
    jT /= shapeA.invMass + shapeB.invMass + 
         (raPerpDotT * raPerpDotT) * shapeA.invRotationalInertia + 
         (rbPerpDotT * rbPerpDotT) * shapeB.invRotationalInertia;
         
    jT /= contact.contactCount;
    
    float j = jList[i];

    //Coulomb's Law
    if(abs(jT) <= j * staticFrictionConst){
      frictionImpulse = new PVector(jT * tangent.x, jT * tangent.y);
    }else{
      frictionImpulse = new PVector(abs(jT)/jT * j * tangent.x * dynamicFrictionConst, abs(jT)/jT * j * tangent.y * dynamicFrictionConst);
    }
    
    frictionImpulseList[i] = frictionImpulse; 
  }
    

  for(int i = 0; i < contact.contactCount; i++){
    frictionImpulse = frictionImpulseList[i];
    ra = raList[i];
    rb = rbList[i];
    //applies the velocity to shapeA if it's movable
    if(shapeA.isMovable){
      shapeA.linearVelocity.x += -1 * frictionImpulse.x * shapeA.invMass;
      shapeA.linearVelocity.y += -1 * frictionImpulse.y * shapeA.invMass;
      
      shapeA.angularVelocity += -1 * crossProduct(ra, frictionImpulse) * shapeA.invRotationalInertia;
    }
        
    //applies the velocity to shapeB if it's movable
    if(shapeB.isMovable){
      shapeB.linearVelocity.x += frictionImpulse.x * shapeB.invMass;
      shapeB.linearVelocity.y += frictionImpulse.y * shapeB.invMass;
      
      shapeB.angularVelocity += crossProduct(rb, frictionImpulse) * shapeB.invRotationalInertia;
    }
  }
}
