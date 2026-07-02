#include <Servo.h>

Servo myServoShoulder;  
Servo myServoForearm;
Servo myServoThumb;
Servo myServoGripper;
Servo myServoBase;

int del = 1500; 
int delTick = 15;

int servoPinShoulder = 5;
int servoPinForearm = 6;
int servoPinThumb = 9;
int servoPinGripper = 10;
int servoPinBase = 11;

// Default rest/safe positions
int defShoulder = 50;
int defForearm = 75;
int defThumb = 0;
int defGripper = 50;
int defBase = 90;

int angleShoulder = defShoulder;
int angleForearm = defForearm;
int angleThumb = defThumb; 
int angleGripper = defGripper;
int angleBase = defBase;

void setup() {
  Serial.begin(9600); // Initialize hardware serial bus

  myServoShoulder.write(angleShoulder); 
  myServoForearm.write(angleForearm);   
  myServoThumb.write(angleThumb);       
  myServoGripper.write(angleGripper); 
  myServoBase.write(angleBase);   

  myServoShoulder.attach(servoPinShoulder);
  myServoForearm.attach(servoPinForearm); 
  myServoThumb.attach(servoPinThumb); 
  myServoGripper.attach(servoPinGripper); 
  myServoBase.attach(servoPinBase);

  delay(500); 
}

void loop() {
  // Wait until Python sends a complete angle data packet
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    // Parse the comma-separated string packet
    // 1. Extract Shoulder
    int targetShoulder = data.substring(0, data.indexOf(',')).toInt();
    data = data.substring(data.indexOf(',') + 1);
    
    // 2. Extract Forearm
    int targetForearm = data.substring(0, data.indexOf(',')).toInt();
    data = data.substring(data.indexOf(',') + 1);
    
    // 3. Extract Thumb
    int targetThumb = data.substring(0, data.indexOf(',')).toInt();
    data = data.substring(data.indexOf(',') + 1);
    
    // 4. Extract Gripper
    int targetGripper = data.substring(0, data.indexOf(',')).toInt();
    data = data.substring(data.indexOf(',') + 1);
    
    // 5. Extract Base
    int targetBase = data.substring(0, data.indexOf(',')).toInt();
    
    // 6. Extract Color
    String color = data.substring(data.indexOf(',') + 1);
    color.trim(); // Clean off any accidental whitespace or hidden characters

    // FIXED: Passed 'color' down to the function to prevent compilation error
    executePickSequence(targetShoulder, targetForearm, targetThumb, targetGripper, targetBase, color);

    // Sequence finished, return home, then alert Python to unlock camera
    Serial.println("DONE");
  }
}

void executePickSequence(int targetShoulder, int targetForearm, int targetThumb, int targetGripper, int targetBase, String color) {
  // Move Base first to align with coordinate line
  for (; angleBase <= targetBase; angleBase++) { myServoBase.write(angleBase); delay(delTick); }
  for (; angleBase >= targetBase; angleBase--) { myServoBase.write(angleBase); delay(delTick); }
  delay(500);

  // Structural step 1: Thumb Move
  if (targetThumb >= 100) {
    for (; angleThumb <= 33 + targetThumb; angleThumb++) { myServoThumb.write(angleThumb); delay(delTick); }
  } else if (targetThumb >= 80 && targetThumb < 100) {
    for (; angleThumb <= 35 + targetThumb; angleThumb++) { myServoThumb.write(angleThumb); delay(delTick); }
  } else {
    for (; angleThumb <= 30 + targetThumb; angleThumb++) { myServoThumb.write(angleThumb); delay(delTick); }
  }
  delay(del);

  // Original step 2: Forearm Move
  for (; angleForearm <= targetForearm; angleForearm++) { myServoForearm.write(angleForearm); delay(delTick); }
  for (; angleForearm >= targetForearm; angleForearm--) { myServoForearm.write(angleForearm); delay(delTick); }
  delay(del);

  // Original step 3: Shoulder Move
  for (; angleShoulder <= 180 - targetShoulder; angleShoulder++) { myServoShoulder.write(angleShoulder); delay(delTick); }
  for (; angleShoulder >= 180 - targetShoulder; angleShoulder--) { myServoShoulder.write(angleShoulder); delay(delTick); }
  delay(del);

  // Original step 4: Grip execution (Clamping onto box)
  for (; angleGripper <= targetGripper ; angleGripper++) { myServoGripper.write(angleGripper); delay(delTick); }
  delay(del);

  // --- SAFE LIFT BACK TO DEFAULT PIVOTS (EXCEPT BASE) ---
  // Bring the armature links up to clear the ground before rotating base to sort
  for (; angleShoulder >= defShoulder; angleShoulder--) { myServoShoulder.write(angleShoulder); delay(delTick); }
  for (; angleShoulder <= defShoulder; angleShoulder++) { myServoShoulder.write(angleShoulder); delay(delTick); }
  delay(del);

  for (; angleThumb >= defThumb; angleThumb--) { myServoThumb.write(angleThumb); delay(delTick); }
  delay(del);

  for (; angleForearm >= defForearm; angleForearm--) { myServoForearm.write(angleForearm); delay(delTick); }
  for (; angleForearm <= defForearm; angleForearm++) { myServoForearm.write(angleForearm); delay(delTick); }
  delay(del);

  // --- COLOR SORTING DETECTION RUNS HERE ---
  
  // sorting for green (Move Base to 0)
  if (color == "Green" || color == "green") {
    for (; angleBase >= 0; angleBase--) { myServoBase.write(angleBase); delay(delTick); }
    delay(del);
    for (; angleShoulder <= 80; angleShoulder++) { myServoShoulder.write(angleShoulder); delay(delTick); } 
    delay(del);
    for (; angleForearm >= 60; angleForearm--) { myServoForearm.write(angleForearm); delay(delTick); } 
    delay(del);
    for (; angleThumb <= 10; angleThumb++) { myServoThumb.write(angleThumb); delay(delTick); } 
    delay(del);
    
    // Release the block
    for (; angleGripper >= defGripper; angleGripper--) { myServoGripper.write(angleGripper); delay(delTick); }
    for (; angleGripper <= defGripper; angleGripper++) { myServoGripper.write(angleGripper); delay(delTick); }
    delay(del);
  }
  // sorting for yellow (Move Base to 180)
  else if (color == "Yellow" || color == "yellow") {
    for (; angleBase <= 180; angleBase++) { myServoBase.write(angleBase); delay(delTick); }
    delay(del);
    for (; angleShoulder <= 80; angleShoulder++) { myServoShoulder.write(angleShoulder); delay(delTick); } 
    delay(del);
    for (; angleForearm >= 60; angleForearm--) { myServoForearm.write(angleForearm); delay(delTick); } 
    delay(del);
    for (; angleThumb <= 10; angleThumb++) { myServoThumb.write(angleThumb); delay(delTick); } 
    delay(del);
    
    // FIXED: Corrected loop constraints to gracefully open gripper to 50
    for (; angleGripper >= defGripper; angleGripper--) { myServoGripper.write(angleGripper); delay(delTick); }
    for (; angleGripper <= defGripper; angleGripper++) { myServoGripper.write(angleGripper); delay(delTick); }
    delay(del);
  }

  // --- RESET SEGMENTS BACK TO SAFETY WINDOWS ---
  // The arm resets its profile links, leaving 'angleBase' exactly where it dropped the box off!
  for (; angleShoulder >= defShoulder; angleShoulder--) { myServoShoulder.write(angleShoulder); delay(delTick); }
  for (; angleShoulder <= defShoulder; angleShoulder++) { myServoShoulder.write(angleShoulder); delay(delTick); }
  delay(del);

  for (; angleThumb >= defThumb; angleThumb--) { myServoThumb.write(angleThumb); delay(delTick); }
  delay(del);

  for (; angleForearm >= defForearm; angleForearm--) { myServoForearm.write(angleForearm); delay(delTick); }
  for (; angleForearm <= defForearm; angleForearm++) { myServoForearm.write(angleForearm); delay(delTick); }
  delay(del);
}
