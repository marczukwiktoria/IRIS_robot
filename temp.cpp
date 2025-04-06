void makeDecision() {
  int y, x, lowestCost = 9999;
  
  // Znajdź najbliższą puszkę
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      if (distanceCost[i][j] < lowestCost && distanceCost[i][j] != 0 && cantrix[i][j] == 1) {
        y = i;
        x = j;
        lowestCost = distanceCost[i][j];
      }
    }
  }

  // Oblicz odległości w osiach X i Y
  int y_dist = y - robotPosition.posY;
  int x_dist = x - robotPosition.posX;
  
  Serial.print("Target: X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" Lowest cost: ");
  Serial.println(lowestCost);

  // Sprawdź czy jesteśmy już w docelowej pozycji
  if (x_dist == 0 && y_dist == 0) {
    Serial.println("Already at target position");
    return;
  }

  // Najpierw spróbuj poruszać się w aktualnym kierunku
  if (robotPosition.rot == 0 && y_dist > 0) { // Obrót 0° - ruch w górę (Y+)
    remainingCrossings = y_dist;
    CurrentAction = Straighten;
    Serial.print("Moving forward (Y+), remaining: ");
    Serial.println(remainingCrossings);
    return;
  }
  else if (robotPosition.rot == 180 && y_dist < 0) { // Obrót 180° - ruch w dół (Y-)
    remainingCrossings = -y_dist;
    CurrentAction = Straighten;
    Serial.print("Moving forward (Y-), remaining: ");
    Serial.println(remainingCrossings);
    return;
  }
  else if (robotPosition.rot == 90 && x_dist > 0) { // Obrót 90° - ruch w prawo (X+)
    remainingCrossings = x_dist;
    CurrentAction = Straighten;
    Serial.print("Moving forward (X+), remaining: ");
    Serial.println(remainingCrossings);
    return;
  }
  else if (robotPosition.rot == -90 && x_dist < 0) { // Obrót -90° - ruch w lewo (X-)
    remainingCrossings = -x_dist;
    CurrentAction = Straighten;
    Serial.print("Moving forward (X-), remaining: ");
    Serial.println(remainingCrossings);
    return;
  }

  // Jeśli nie możemy jechać prosto w aktualnym kierunku, wykonaj obrót
  if (abs(y_dist) >= abs(x_dist)) { // Priorytet dla osi Y jeśli odległość jest większa lub równa
    if (y_dist > 0) { // Potrzebny ruch w górę (Y+)
      if (robotPosition.rot == 180) {
        CurrentAction = RotateRight; // Obrót o 180°
      }
      else if (robotPosition.rot == 90) {
        CurrentAction = RotateLeft; // Obrót w lewo (90° -> 0°)
      }
      else if (robotPosition.rot == -90) {
        CurrentAction = RotateRight; // Obrót w prawo (-90° -> 0°)
      }
      remainingCrossings = y_dist;
    }
    else if (y_dist < 0) { // Potrzebny ruch w dół (Y-)
      if (robotPosition.rot == 0) {
        CurrentAction = RotateRight; // Obrót o 180°
      }
      else if (robotPosition.rot == 90) {
        CurrentAction = RotateRight; // Obrót w prawo (90° -> 180°)
      }
      else if (robotPosition.rot == -90) {
        CurrentAction = RotateLeft; // Obrót w lewo (-90° -> 180°)
      }
      remainingCrossings = -y_dist;
    }
  }
  else { // Priorytet dla osi X
    if (x_dist > 0) { // Potrzebny ruch w prawo (X+)
      if (robotPosition.rot == 0) {
        CurrentAction = RotateRight; // Obrót w prawo (0° -> 90°)
      }
      else if (robotPosition.rot == 180) {
        CurrentAction = RotateLeft; // Obrót w lewo (180° -> 90°)
      }
      else if (robotPosition.rot == -90) {
        CurrentAction = RotateRight; // Obrót o 180°
      }
      remainingCrossings = x_dist;
    }
    else if (x_dist < 0) { // Potrzebny ruch w lewo (X-)
      if (robotPosition.rot == 0) {
        CurrentAction = RotateLeft; // Obrót w lewo (0° -> -90°)
      }
      else if (robotPosition.rot == 180) {
        CurrentAction = RotateRight; // Obrót w prawo (180° -> -90°)
      }
      else if (robotPosition.rot == 90) {
        CurrentAction = RotateRight; // Obrót o 180°
      }
      remainingCrossings = -x_dist;
    }
  }

  Serial.print("Rotating to new direction, remaining after: ");
  Serial.println(remainingCrossings);
}