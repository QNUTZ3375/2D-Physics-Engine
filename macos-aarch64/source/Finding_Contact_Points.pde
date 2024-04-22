PVector[] findContactPoints(Shape shapeA, Shape shapeB){
  PVector[] res = new PVector[0];
  
  //checks for the shape type of both shapes before doing the calculations
  if(shapeA.shapeType == "Box" && shapeB.shapeType == "Box"){
    PVector[] temp = findContactPointBoxes(shapeA.transformedVertices, shapeB.transformedVertices);
    res = Arrays.copyOf(res, temp.length);
    for(int i = 0; i < temp.length; i++){
      res[i] = temp[i];
    }
  }else if(shapeA.shapeType == "Circle" && shapeB.shapeType == "Circle"){
    res = Arrays.copyOf(res, res.length + 1);
    res[res.length - 1] = findContactPointCircles(shapeA.center, shapeA.r, shapeB.center);
  }else{
    if(shapeA.shapeType == "Circle"){
      res = Arrays.copyOf(res, res.length + 1);
      res[res.length - 1] = findContactPointCircleBox(shapeA.center, shapeB.transformedVertices);
    }else if(shapeB.shapeType == "Circle"){
      res = Arrays.copyOf(res, res.length + 1);
      res[res.length - 1] = findContactPointCircleBox(shapeB.center, shapeA.transformedVertices);
    }
  }
  return res;
}

PVector findContactPointCircles(PVector centerA, float radiusA, PVector centerB){
  PVector contactPoint = new PVector(0, 0);
  //finds the normalized direction vector between both centers
  PVector aToBDist = new PVector(centerB.x - centerA.x, centerB.y - centerA.y);
  float totalLength = lengthOfVector(aToBDist);
  PVector dir = new PVector(aToBDist.x / totalLength, aToBDist.y / totalLength);
  //multiplies it by the radius of shape A (because the vector points from A to B) to get the contact point
  contactPoint.x = centerA.x + dir.x * radiusA;
  contactPoint.y = centerA.y + dir.y * radiusA;
  
  return contactPoint;
}

PVector findContactPointCircleBox(PVector centerA, PVector[] verticesB){
  PVector contactPoint = new PVector(0, 0);
  float minDistSq = pow(2, 31) - 1;
  //goes through each vertex
  for(int i = 0; i < verticesB.length; i++){
    PVector vertexA = verticesB[i];
    PVector vertexB = verticesB[(i + 1) % verticesB.length];
    //finids the distance from each line of the box to the center of the circle
    PVector[] contactAndDist = pointSegmentDistance(centerA, vertexA, vertexB);
    float distSq = lengthOfVectorSquared(contactAndDist[1]);
    //updates the values if necessary
    if(distSq < minDistSq){
      minDistSq = distSq;
      contactPoint = contactAndDist[0];
    }
  }
  return contactPoint;
}

PVector[] pointSegmentDistance(PVector p, PVector a, PVector b){
  //makes two vectors using the three points given (p = point of interest on a shape, a & b are the two vertices of the other shape)
  PVector ab = new PVector(b.x - a.x, b.y - a.y);
  PVector ap = new PVector(p.x - a.x, p.y - a.y);
  PVector contact;
  PVector distances;
  //gets the dot product between ap and ab then "normalizes" it using ab (without the square root)
  float projection = dotProduct(ap, ab);
  float lenSquared = lengthOfVectorSquared(ab);
  float d = projection / lenSquared;
  
  if(d <= 0){  //ab points the opposite direction of ap
    contact = a;
  }else if(d >= 1){ //ab and ap point to the same direction
    contact = b;
  }else{ //somewhere in between
    contact = new PVector(a.x + ab.x * d, a.y + ab.y * d); //finds the middle ground between both points using point A as the reference
  }
  //gets the shortest distance between the contact point and the line
  distances = new PVector(p.x - contact.x, p.y - contact.y);
  
  PVector[] res = {contact, distances};
  
  return res;
}

PVector[] findContactPointBoxes(PVector[] verticesA, PVector[] verticesB){
  PVector contact1 = new PVector(pow(2, 31) - 1, pow(2, 31) - 1);
  PVector contact2 = new PVector(pow(2, 31) - 1, pow(2, 31) - 1);
  PVector point = new PVector(0, 0);
  float minDistSq = pow(2, 31) - 1;
  //loops through all vertices of shape A
  for(int i = 0; i < verticesA.length; i++){
    point = verticesA[i];
    //loops through all vertices of shape B
    for(int j = 0; j < verticesB.length; j++){
      PVector va = verticesB[j];
      PVector vb = verticesB[(j + 1) % verticesB.length];
      //checks for the distance between line B and point A
      PVector[] contactAndDist = pointSegmentDistance(point, va, vb);
      float distSq = lengthOfVectorSquared(contactAndDist[1]);
      //checks if the distance is almost equal to the current minimum
      if(almostEqual(distSq, minDistSq)){
        //checks if the vectors aren't the same
        if(!almostEqualVectors(contactAndDist[0], contact1)){
          contact2 = contactAndDist[0];
        }
      }else if(distSq < minDistSq){
        minDistSq = distSq;
        contact1 = contactAndDist[0];
      }
    }
  }
  //loops through all the vertices of B
  for(int i = 0; i < verticesB.length; i++){
    point = verticesB[i];
    //loops through all the vertices of A
    for(int j = 0; j < verticesA.length; j++){
      PVector va = verticesA[j];
      PVector vb = verticesA[(j + 1) % verticesA.length];
      //finds the distance between line A and point B
      PVector[] contactAndDist = pointSegmentDistance(point, va, vb);
      float distSq = lengthOfVectorSquared(contactAndDist[1]);
      //checks if the distance is almost equal to the current minimum
      if(almostEqual(distSq, minDistSq)){
        //checks if the vectors aren't the same
        if(!almostEqualVectors(contactAndDist[0], contact1)){
          contact2 = contactAndDist[0];
        }
      }else if(distSq < minDistSq){
        minDistSq = distSq;
        contact1 = contactAndDist[0];
      }
    }
  }
  PVector[] res = {};
  //initializes res according to the number of contact points found
  if(contact2.x != pow(2, 31) - 1 && contact2.y != pow(2, 31) - 1){
    res = new PVector[2];
    res[0] = contact1;
    res[1] = contact2;
  }else if(contact1.x != pow(2, 31) - 1 && contact1.y != pow(2, 31) - 1){
    res = new PVector[1];
    res[0] = contact1;
  }
  return res;
}

boolean almostEqual(float a, float b){
  float threshold = 0.005;
  return abs(a - b) < threshold;
}

boolean almostEqualVectors(PVector a, PVector b){
  float threshold = 0.005;
  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) < threshold * threshold;
}
