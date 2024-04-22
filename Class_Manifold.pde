class Manifold{ //class used to store contacts between objects
  Shape shapeA;
  Shape shapeB;
  PVector direction;
  float magnitude;
  PVector[] contactPoints;
  PVector contactP2;
  int contactCount;
  
  Manifold(Shape a, Shape b, PVector normal, float mag, PVector[] points, int count){
    shapeA = a;
    shapeB = b;
    direction = new PVector(normal.x, normal.y);
    magnitude = mag;
    contactPoints = points;
    contactCount = count;
  }
}
