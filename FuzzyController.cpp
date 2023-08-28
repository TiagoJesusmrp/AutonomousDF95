#include "Arduino.h"
#include "Fuzzy.h"

Fuzzy *RudderFuzzy = new Fuzzy();
Fuzzy *WinchFuzzy = new Fuzzy();

float calculateHeadingError(float currentHeading, float desiredHeadingToTarget){
  // Returns a value between -180 and 180 corresponding to the shortest turn amount to the desired Heading
  float error = desiredHeadingToTarget - currentHeading;
  if (error > 180) {
    error -= 360; // Normalize error to -180 to 180 range
  } else if (error < -180) {
    error += 360; // Normalize error to -180 to 180 range
  }
  return error;
}

int mapToNegative(int val){ // Takes windreading between 0 and 360 and transfers to -180 to 180
  if (val > 180) {
    return val - 360;
  } else {
    return val;
  }
}

void WinchControllerSetup(){
  // Build the Relative Wind Direction Input Object

  // Instantiating a FuzzyInput object
  FuzzyInput *relativeWindDirection = new FuzzyInput(2);

  // Instantiating a FuzzySet object
  FuzzySet *NoGoZone = new FuzzySet(0, 0, 30, 65); // Trapezoidal
  FuzzySet *Behind = new FuzzySet(100, 150, 180, 190);
  FuzzySet *Side = new FuzzySet(30, 65, 115, 160);
  // Including the FuzzySet into FuzzyInput
  relativeWindDirection->addFuzzySet(NoGoZone);
  relativeWindDirection->addFuzzySet(Behind);
  relativeWindDirection->addFuzzySet(Side);

  // Including the FuzzyInput relativeWind into Winch Fuzzy Object
  WinchFuzzy->addFuzzyInput(relativeWindDirection);

  // Build the RudderAngle Output Object
  FuzzyOutput *WinchAngle = new FuzzyOutput(2);

  // Instantiating a FuzzySet object
  FuzzySet *Haul = new FuzzySet(10, 10, 10, 10); // Singleton
  FuzzySet *Reach = new FuzzySet(45, 45, 45, 45);
  FuzzySet *Run = new FuzzySet(90, 90, 90, 90);

  // Including the FuzzySet into FuzzyOutput
  WinchAngle->addFuzzySet(Haul);
  WinchAngle->addFuzzySet(Reach);
  WinchAngle->addFuzzySet(Run);

  // Including the FuzzyOutput Winch Angle into Rudder Fuzzy Object
  WinchFuzzy->addFuzzyOutput(WinchAngle);

  //Building Fuzzy Rules
  // IF relativeWindDirection is NoGoZone then winchAngle is Haul
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifwindNoGoZone = new FuzzyRuleAntecedent();
  ifwindNoGoZone->joinSingle(NoGoZone);
  FuzzyRuleConsequent *thenHaul = new FuzzyRuleConsequent();
  thenHaul->addOutput(Haul);
  FuzzyRule *winchRule01 = new FuzzyRule(6, ifwindNoGoZone, thenHaul);
  WinchFuzzy->addFuzzyRule(winchRule01);

  // IF relativeWindDirection is Side then winchAngle is Reach
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifwindSide = new FuzzyRuleAntecedent();
  ifwindSide->joinSingle(Side);
  FuzzyRuleConsequent *thenReach = new FuzzyRuleConsequent();
  thenReach->addOutput(Reach);
  FuzzyRule *winchRule02 = new FuzzyRule(7, ifwindSide, thenReach);
  WinchFuzzy->addFuzzyRule(winchRule02);

  // IF relativeWindDirection is Behind then winchAngle is Run
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifwindBehind = new FuzzyRuleAntecedent();
  ifwindBehind->joinSingle(Behind);
  FuzzyRuleConsequent *thenRun = new FuzzyRuleConsequent();
  thenRun->addOutput(Run);
  FuzzyRule *winchRule03 = new FuzzyRule(8, ifwindBehind, thenRun);
  WinchFuzzy->addFuzzyRule(winchRule03);
}

float GetWinchControlAction(float relativeWind){
  int windInput = abs(mapToNegative(relativeWind)); // Takes the read windReading (0,360) and converts to (-180,180) and then to (0, 180) because of simmetry
  // Set calculated windRelativeDirection as an input
  WinchFuzzy->setInput(2, windInput);
  // Running the Fuzzification
  WinchFuzzy->fuzzify();
  float output = WinchFuzzy->defuzzify(2);
  return output; // Output is the winch angle in degrees [0, 90]
} 


void RudderControllerSetup(){

  // Build the Heading Error Input Object
  // Instantiating a FuzzyInput object
  FuzzyInput *HeadingError = new FuzzyInput(1);

  // Instantiating a FuzzySet object
  FuzzySet *LeftSide = new FuzzySet(-65, -45, -30, -5); // Trapezoidal // Same as V1 controller but frontSide is smaller
  FuzzySet *FrontSide = new FuzzySet(-10, -3, 3, 10);
  FuzzySet *RightSide = new FuzzySet(5, 30, 45, 65);
  FuzzySet *FullRight = new FuzzySet(45, 90, 200, 350);
  FuzzySet *FullLeft = new FuzzySet(-350, -200, -90, -45);

  // Including the FuzzySet into FuzzyInput
  HeadingError->addFuzzySet(LeftSide);
  HeadingError->addFuzzySet(FrontSide);
  HeadingError->addFuzzySet(RightSide);
  HeadingError->addFuzzySet(FullRight);
  HeadingError->addFuzzySet(FullLeft);

  // Including the FuzzyInput HeadingError into Rudder Fuzzy Object
  RudderFuzzy->addFuzzyInput(HeadingError);

  // Build the RudderAngle Output Object
  FuzzyOutput *RudderAngle = new FuzzyOutput(1);

  // Instantiating a FuzzySet object
  FuzzySet *TurnLeft = new FuzzySet(-30, -30, -30, -30); // Singleton
  FuzzySet *Keep = new FuzzySet(0, 0, 0, 0);
  FuzzySet *TurnRight = new FuzzySet(30, 30, 30 ,30);
  FuzzySet *HardRight = new FuzzySet(45, 45, 45, 45);
  FuzzySet *HardLeft = new FuzzySet(-45, -45, -45, -45); 

  // Including the FuzzySet into FuzzyOutput
  RudderAngle->addFuzzySet(TurnLeft);
  RudderAngle->addFuzzySet(Keep);
  RudderAngle->addFuzzySet(TurnRight);
  RudderAngle->addFuzzySet(HardRight);
  RudderAngle->addFuzzySet(HardLeft);

  // Including the FuzzyOutput Rudder Angle into Rudder Fuzzy Object
  RudderFuzzy->addFuzzyOutput(RudderAngle);


  //Building Fuzzy Rules
  // IF HeadingError is LeftSide then RudderAngle is TurnLeft Erro negativo - objetivo a esquerda - virar para a esquerda
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorLeft = new FuzzyRuleAntecedent();
  ifErrorLeft->joinSingle(LeftSide);
  FuzzyRuleConsequent *thenTurnLeft = new FuzzyRuleConsequent();
  thenTurnLeft->addOutput(TurnLeft);
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifErrorLeft, thenTurnLeft);
  RudderFuzzy->addFuzzyRule(fuzzyRule01);

  // IF HeadingError is FrontSide then RudderAngle is Keep
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFront = new FuzzyRuleAntecedent();
  ifErrorFront->joinSingle(FrontSide);
  FuzzyRuleConsequent *thenKeep = new FuzzyRuleConsequent();
  thenKeep->addOutput(Keep);
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifErrorFront, thenKeep);
  RudderFuzzy->addFuzzyRule(fuzzyRule02);

  // IF HeadingError is RightSide then RudderAngle is TurnRight
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorRight = new FuzzyRuleAntecedent();
  ifErrorRight->joinSingle(RightSide);
  FuzzyRuleConsequent *thenTurnRight = new FuzzyRuleConsequent();
  thenTurnRight->addOutput(TurnRight);
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifErrorRight, thenTurnRight);
  RudderFuzzy->addFuzzyRule(fuzzyRule03);

  //IF HeadingError is Full Right then RudderAngle is HardRight
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFullRight = new FuzzyRuleAntecedent();
  ifErrorFullRight->joinSingle(FullRight);
  FuzzyRuleConsequent *thenTurnHardRight = new FuzzyRuleConsequent();
  thenTurnHardRight->addOutput(HardRight);
  FuzzyRule *fuzzyRule04 = new FuzzyRule(4, ifErrorFullRight, thenTurnHardRight);
  RudderFuzzy->addFuzzyRule(fuzzyRule04);

  //IF HeadingError is FullLeft then RudderAngle is HardLeft
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFullLeft = new FuzzyRuleAntecedent();
  ifErrorFullLeft->joinSingle(FullLeft);
  FuzzyRuleConsequent *thenTurnHardLeft = new FuzzyRuleConsequent();
  thenTurnHardLeft->addOutput(HardLeft);
  FuzzyRule *fuzzyRule05 = new FuzzyRule(5, ifErrorFullLeft, thenTurnHardLeft);
  RudderFuzzy->addFuzzyRule(fuzzyRule05);
}


float GetRudderControlAction(float inputError){
  // Set the calculated Heading Error as an input
  RudderFuzzy->setInput(1, inputError);
  // Running the Fuzzification
  RudderFuzzy->fuzzify();
  float output = RudderFuzzy->defuzzify(1);
  return output; // Output is the rudder angle in degrees [-45, 45]
} 

