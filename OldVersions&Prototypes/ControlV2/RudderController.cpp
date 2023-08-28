#include "Arduino.h"
#include "Fuzzy.h"

Fuzzy *RudderFuzzy = new Fuzzy();

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

void RudderControllerSetup(){

  // Build the Heading Error Input Object
  // Instantiating a FuzzyInput object
  FuzzyInput *HeadingError = new FuzzyInput(1);

  // Instantiating a FuzzySet object
  FuzzySet *LeftSide = new FuzzySet(-65, -45, -10, 0); // Trapezoidal
  FuzzySet *FrontSide = new FuzzySet(-3.5, 0, 0, 3.5);
  FuzzySet *RightSide = new FuzzySet(0, 10, 45, 65);
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
  FuzzySet *TurnLeft = new FuzzySet(-25, -25, -25, -25); // Singleton
  FuzzySet *Keep = new FuzzySet(0, 0, 0, 0);
  FuzzySet *TurnRight = new FuzzySet(25, 25, 25 ,25);
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

