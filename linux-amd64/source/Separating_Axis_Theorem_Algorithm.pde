boolean isCircleAndPolygonIntersecting(PVector centerCircle, float radius, PVector[] vertices, PVector centerPolygon){
  //resets the direction and magnitude for comparisons
  direction.x = 0;
  direction.y = 0;
  magnitude = pow(2, 31) - 1;
  tangent.x = 0;
  tangent.y = 0;
  normal.x = 0;
  normal.y = 0;
  float[] projectedPolygon;
  float[] projectedCircle;
  
  for(int i = 0; i < vertices.length; i++){
    //the vectors will be cycled clockwise
    tangent.x = vertices[(i + 1) % vertices.length].x - vertices[i].x; 
    tangent.y = vertices[(i + 1) % vertices.length].y - vertices[i].y;
    //M_tangent * M_normal = -1
    normal.x = -1 * tangent.y;
    normal.y = tangent.x;
    
    float normalLength = lengthOfVector(normal);
    //normalizes the axis for accurate comparisons
    normal.x /= normalLength;
    normal.y /= normalLength;
    
    //finds the maximum and minimum values after projection to the axis 
    //(moving the shortest distance from point to axis) for both sets of vertices
    projectedPolygon = projectToAxis(vertices, normal);
    projectedCircle = projectCircleToAxis(centerCircle, radius, normal);
    //checks if the minimum of either value is greater than the maximum of the other (meaning no intersection)
    if(projectedPolygon[0] >= projectedCircle[1] || projectedCircle[0] >= projectedPolygon[1]){
      return false;
    }
    //gets the magnitude of the current distance to move if there is no gap
    float axisDepth = min(projectedCircle[1] - projectedPolygon[0], projectedPolygon[1] - projectedCircle[0]);
    //checks if there is a new shortest magnitude
    if(axisDepth < magnitude){
      //updates the direction of the normal and the magnitude
      direction.x = normal.x;
      direction.y = normal.y;
      magnitude = axisDepth;
    }
  }
  //finds the closest point of the polygon to the circle
  PVector closestPoint = vertices[closestVertexToCircle(centerCircle, vertices)];
  //gets the axis that goes through both the circle's center and the closest point
  normal.x = closestPoint.x - centerCircle.x;
  normal.y = closestPoint.y - centerCircle.y;
  
  float axisLength = lengthOfVector(normal);
  //normalizes the axis for accurate comparisons
  normal.x /= axisLength;
  normal.y /= axisLength;
  
  //finds the maximum and minimum values after projection to the axis 
  //(moving the shortest distance from point to axis) for both sets of vertices
  projectedPolygon = projectToAxis(vertices, normal);
  projectedCircle = projectCircleToAxis(centerCircle, radius, normal);
  //checks if the minimum of either value is greater than the maximum of the other (meaning no intersection)
  if(projectedPolygon[0] >= projectedCircle[1] || projectedCircle[0] >= projectedPolygon[1]){
    return false;
  }
  //gets the magnitude of the current distance to move if there is no gap
  float axisDepth = min(projectedCircle[1] - projectedPolygon[0], projectedPolygon[1] - projectedCircle[0]);
  //checks if there is a new shortest magnitude
  if(axisDepth < magnitude){
    //updates the direction of the normal and the magnitude
    direction.x = normal.x;
    direction.y = normal.y;
    magnitude = axisDepth;
  }
  
  //gets the direction between the centers of both shapes (polygon to circle) and  
  //checks if the current direction is pointing the same way as the intended normalDirection
  if(dotProductFloats(centerPolygon.x - centerCircle.x, centerPolygon.y - centerCircle.y, direction.x, direction.y) < 0){
    direction.x *= -1;
    direction.y *= -1;
  }
  return true;
}

int closestVertexToCircle(PVector center, PVector[] vertices){
  int res = -1;
  float minDistance = pow(2, 31) - 1;
  float currDistance = 0;
  //basically checks if the distance between the current vertex of the point is closer to the circle's center 
  //than the current closest vertex
  for(int i = 0; i < vertices.length; i++){
    //no need to do the square root, the value is only used for comparisons, not calculations
    currDistance = lengthSquared(vertices[i].x - center.x, vertices[i].y - center.y);
    if(currDistance < minDistance){
      minDistance = currDistance;
      res = i;
    }
  }
  return res;
}

float[] projectCircleToAxis(PVector center, float radius, PVector axis){
  //gets the scalar length of the axis from the origin
  float lenAxis = lengthOfVector(axis);
  //direction normalizes the axis
  PVector directionAndRadius = new PVector(radius * (axis.x / lenAxis), radius * (axis.y / lenAxis));
  //gets the two max and min points of the circle (min being the left-most and top-most, max being the right-most and bottom-most) and
  //gets the dot product of max and min values according to the axis (projects both points to the axis without normalization)
  float min = dotProductFloats(center.x - directionAndRadius.x, center.y - directionAndRadius.y, axis.x, axis.y);
  float max = dotProductFloats(center.x + directionAndRadius.x, center.y + directionAndRadius.y, axis.x, axis.y);
  //checks if min is actually greater than max
  if(min > max){
    float temp = min;
    min = max;
    max = temp;
  }
  float[] res = {min, max};

  return res;
}


boolean areCirclesIntersecting(PVector centerA, PVector centerB, float radiusA, float radiusB){
  //gets the x and y distance between both centers
  float distanceX = centerA.x - centerB.x;
  float distanceY = centerA.y - centerB.y;
  //uses Pythagorean theorem (without square root to reduce computation time) 
  //to compare the current distance with the sum of the radii
  if(distanceX * distanceX + distanceY * distanceY >= (radiusA + radiusB) * (radiusA + radiusB)){
    return false;
  } 
  //performs the square root after the sum of the squares (case where there is collision)
  float totalDistance = pow(distanceX * distanceX + distanceY * distanceY, 0.5);
  //gets the direction and magnitude of the collision to resolve
  direction.x = (centerB.x - centerA.x) / totalDistance;
  direction.y = (centerB.y - centerA.y) / totalDistance;
  magnitude = (radiusA + radiusB) - totalDistance;
  
  return true;
}

boolean isIntersecting(PVector[] verticesA, PVector[] verticesB, PVector centerA, PVector centerB){
  //resets the direction of the normal and the magnitude
  direction.x = 0;
  direction.y = 0;
  magnitude = pow(2, 31) - 1;
  tangent.x = 0;
  tangent.y = 0;
  normal.x = 0;
  normal.y = 0;
  float[] projectedA;
  float[] projectedB;
  
  //goes through the vertices of the first shape
  for(int i = 0; i < verticesA.length; i++){
    //the vectors will be cycled clockwise
    //M_tangent * M_normal = -1
    tangent.x = verticesA[(i + 1) % verticesA.length].x - verticesA[i].x;
    tangent.y = verticesA[(i + 1) % verticesA.length].y - verticesA[i].y;
    normal.x = -1 * tangent.y;
    normal.y = tangent.x;
    
    float normalLength = lengthOfVector(normal);
    //normalizes the axis for accurate comparisons
    normal.x /= normalLength;
    normal.y /= normalLength;
    
    //finds the maximum and minimum values after projection to the axis 
    //(moving the shortest distance from point to axis) for both sets of vertices
    projectedA = projectToAxis(verticesA, normal);
    projectedB = projectToAxis(verticesB, normal);
    //checks if the minimum of either value is greater than the maximum of the other (meaning no intersection)
    if(projectedA[0] >= projectedB[1] || projectedB[0] >= projectedA[1]){
      return false;
    }

    //gets the smallest distance between both shapes
    float axisDepth = min(projectedB[1] - projectedA[0], projectedA[1] - projectedB[0]);
    //checks if there is a new shortest distance between both shapes
    if(axisDepth < magnitude){
      //updates the direction of the normal and the magnitude
      direction.x = normal.x;
      direction.y = normal.y;
      magnitude = axisDepth;
    }
  }
  
  //goes through the vertices of the second shape
  for(int i = 0; i < verticesB.length; i++){
    //the vectors will be cycled clockwise
    //M_tangent * M_normal = -1
    tangent.x = verticesB[(i + 1) % verticesB.length].x - verticesB[i].x;
    tangent.y = verticesB[(i + 1) % verticesB.length].y - verticesB[i].y;
    normal.x = -1 * tangent.y;
    normal.y = tangent.x;
    
    float normalLength = lengthOfVector(normal);
    //normalizes the axis for accurate comparisons
    normal.x /= normalLength;
    normal.y /= normalLength;

    //finds the maximum and minimum values after projection to the axis 
    //(moving the shortest distance from point to axis) for both sets of vertices
    projectedA = projectToAxis(verticesA, normal);
    projectedB = projectToAxis(verticesB, normal);
    //checks if the minimum of either value is greater than the maximum of the other (meaning no intersection)
    if(projectedA[0] >= projectedB[1] || projectedB[0] >= projectedA[1]){
      return false;
    }
    //gets the smallest distance between both shapes
    float axisDepth = min(projectedB[1] - projectedA[0], projectedA[1] - projectedB[0]);
    //checks if there is a new shortest distance between both shapes
    if(axisDepth < magnitude){
      //updates the direction of the normal and the magnitude
      direction.x = normal.x;
      direction.y = normal.y;
      magnitude = axisDepth;
    }
  }
  
  //gets the direction between the centers of both shapes (from B to A) and
  //checks if the current direction is pointing the same way as the intended normalDirection
  if(dotProductFloats(centerB.x - centerA.x, centerB.y - centerA.y, direction.x, direction.y) < 0){
    direction.x *= -1;
    direction.y *= -1;
  }
  //case where no gaps are found (there is an intersection)
  return true;
}

float[] projectToAxis(PVector[] vertices, PVector axis){
  //[0] = min (starts with the maximum value), and [1] = max (starts with the minimum value)
  float[] res = {pow(2, 31) - 1, -1 * pow(2, 31)};

  for(int i = 0; i < vertices.length; i++){
    //finds the dot product of the axis and the current point (vertex)
    //this basically translates the point to the axis in the shortest distance possible (I still don't fully understand it)*
    float projection = dotProduct(vertices[i], axis);
    //replaces the min and maximum values if either condition is true
    res[0] = (projection < res[0]) ? projection : res[0];
    res[1] = (projection > res[1]) ? projection : res[1];
  }
  return res;
}

/*
So, it basically calculates the width of the line (distance) 
from the origin to a specific point after being projected along the axis.
It then basically reduces into the same algorithm used for checking AABBs (Axis-Aligned Boundary Boxes) 
or straightened rectangles, but after a rotation operation has been done. 

AABB check: 
//L2 < R1 and L1 < R2 and B2 < T1 and B1 < T2

The whole algorithm takes advantage of this fact and applies it to all of the faces' normals 
(gradients perpendicular to the surface) as it eliminates the need to calculate angles. 
It then repeats this process for the other polygon to thoroughly check 
if there are any gaps (no intersection) or not (yes intersection).
*/
