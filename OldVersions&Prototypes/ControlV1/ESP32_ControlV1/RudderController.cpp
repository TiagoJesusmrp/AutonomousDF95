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
  FuzzySet *LeftSide = new FuzzySet(-65, -45, -30, -5); // Trapezoidal
  // Including the FuzzySet into FuzzyInput
  HeadingError->addFuzzySet(LeftSide);

  FuzzySet *FrontSide = new FuzzySet(-12, -5, 5, 12);
  HeadingError->addFuzzySet(FrontSide);

  FuzzySet *RightSide = new FuzzySet(5, 30, 45, 65);
  HeadingError->addFuzzySet(RightSide);

  FuzzySet *FullRight = new FuzzySet(45, 90, 200, 350);
  HeadingError->addFuzzySet(FullRight);

  FuzzySet *FullLeft = new FuzzySet(-350, -200, -90, -45);
  HeadingError->addFuzzySet(FullLeft);

  // Including the FuzzyInput HeadingError into Rudder Fuzzy Object
  RudderFuzzy->addFuzzyInput(HeadingError);



  // Build the RudderAngle Output Object
  FuzzyOutput *RudderAngle = new FuzzyOutput(1);

  // Instantiating a FuzzySet object
  FuzzySet *TurnLeft = new FuzzySet(-25, -25, -25, -25); // Singleton
  // Including the FuzzySet into FuzzyOutput
  RudderAngle->addFuzzySet(TurnLeft);

  FuzzySet *Keep = new FuzzySet(0, 0, 0, 0);
  RudderAngle->addFuzzySet(Keep);

  FuzzySet *TurnRight = new FuzzySet(25, 25, 25 ,25);
  RudderAngle->addFuzzySet(TurnRight);

  FuzzySet *HardRight = new FuzzySet(45, 45, 45, 45);
  RudderAngle->addFuzzySet(HardRight);

  FuzzySet *HardLeft = new FuzzySet(-45, -45, -45, -45);
  RudderAngle->addFuzzySet(HardLeft);

  // Including the FuzzyOutput Rudder Angle into Rudder Fuzzy Object
  RudderFuzzy->addFuzzyOutput(RudderAngle);


  //Building Fuzzy Rules
  // IF HeadingError is LeftSide then RudderAngle is TurnLeft Erro negativo - objetivo a esquerda - virar para a esquerda
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorLeft = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifErrorLeft->joinSingle(LeftSide);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenTurnLeft = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenTurnLeft->addOutput(TurnLeft);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifErrorLeft, thenTurnLeft);
  // Including the FuzzyRule into Fuzzy
  RudderFuzzy->addFuzzyRule(fuzzyRule01);

  // IF HeadingError is FrontSide then RudderAngle is Keep
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFront = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifErrorFront->joinSingle(FrontSide);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenKeep = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenKeep->addOutput(Keep);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifErrorFront, thenKeep);
  // Including the FuzzyRule into Fuzzy
  RudderFuzzy->addFuzzyRule(fuzzyRule02);

  // IF HeadingError is RightSide then RudderAngle is TurnRight
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorRight = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifErrorRight->joinSingle(RightSide);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenTurnRight = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenTurnRight->addOutput(TurnRight);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifErrorRight, thenTurnRight);
  // Including the FuzzyRule into Fuzzy
  RudderFuzzy->addFuzzyRule(fuzzyRule03);

  //IF HeadingError is Full Right then RudderAngle is HardRight
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFullRight = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifErrorFullRight->joinSingle(FullRight);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenTurnHardRight = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenTurnHardRight->addOutput(HardRight);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule04 = new FuzzyRule(4, ifErrorFullRight, thenTurnHardRight);
  // Including the FuzzyRule into Fuzzy
  RudderFuzzy->addFuzzyRule(fuzzyRule04);

  //IF HeadingError is FullLeft then RudderAngle is HardLeft
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifErrorFullLeft = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifErrorFullLeft->joinSingle(FullLeft);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenTurnHardLeft = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenTurnHardLeft->addOutput(HardLeft);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule05 = new FuzzyRule(5, ifErrorFullLeft, thenTurnHardLeft);
  // Including the FuzzyRule into Fuzzy
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

