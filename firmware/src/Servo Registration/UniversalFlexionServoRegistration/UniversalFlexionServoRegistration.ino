#include <SCServo.h>
#include "BluetoothSerial.h"

#define S_RXD 18
#define S_TXD 19

SMS_STS st;
int ID_ChangeFrom = 1;  // Change the original servo ID, and the factory default is 1
int ID_ChangeTo = -1;   // IndexFlexionServo: ID 1
int ans = -100;
BluetoothSerial SerialBT;

void pressEnterWhenReady(bool continuous_print = true) {
  SerialBT.println("Press ENTER when ready!"); 
  auto ready = false;
  while (!ready) {
    if (continuous_print) { SerialBT.println("Press ENTER when ready!"); }
    delay(1000);
    if (SerialBT.available()) {
      SerialBT.readStringUntil('\n');
      ready = true;
    }
  }
}

void setup() {
  SerialBT.begin("YeahHand");
  SerialBT.setTimeout(50);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while (!Serial1) { delay(100); }

  delay(1000);

  pressEnterWhenReady();

  SerialBT.println("\n\nWARNING: This program will modify the ID of the connected servo and will configure its center of range to the current position!");
  SerialBT.println("WARNING: Connect only one servomotor to the logic board!");

  while (ID_ChangeTo < 1) {
    SerialBT.println("Which servo ID do you want to register?");

    while (!SerialBT.available())
      ;

    String cmd = SerialBT.readStringUntil('\n');
    if (cmd.length() > 0) {
      ID_ChangeTo = cmd.toInt();
      if (ID_ChangeTo < 1) {
        SerialBT.println("Please enter an ID > 0..");
      }
    }
  }

  // Identify ID..
  SerialBT.printf("Identifying Servo ID \n");
  SerialBT.printf("Trying with ID %d..", ID_ChangeFrom);
  int retrievedID = st.Ping(ID_ChangeFrom);
  SerialBT.printf("..retrieved: %d\n", ID_ChangeFrom);
  while (retrievedID != ID_ChangeFrom) {
    delay(1000);
    ID_ChangeFrom += 1;
    SerialBT.printf("Trying with ID %d..", ID_ChangeFrom);
    retrievedID = st.Ping(ID_ChangeFrom);
    SerialBT.printf("..retrieved: %d\n", retrievedID);
  }
  // ID identified
  SerialBT.printf("Found ID %d\n", ID_ChangeFrom);

  SerialBT.printf("Programming new ID %d...\n", ID_ChangeTo);
  ans = st.unLockEprom(ID_ChangeFrom);  //Unlock EPROM-SSAFE
  SerialBT.printf("Unlock ID %d, answer: %d\n", ID_ChangeFrom, ans);

  ans = st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_ChangeTo);  //Change ID
  SerialBT.printf("WriteByte ID %d, answer: %d\n", ID_ChangeFrom, ans);

  ans = st.LockEprom(ID_ChangeTo);  // EPROM-SAFE is locked
  SerialBT.printf("Lock ID %d, answer: %d\n", ID_ChangeTo, ans);

  delay(1000);

  // Check programmed ID
  SerialBT.printf("Trying with ID %d..", ID_ChangeTo);
  retrievedID = st.Ping(ID_ChangeTo);
  SerialBT.printf("..retrieved: %d\n", retrievedID);
  delay(1000);
  if (retrievedID == ID_ChangeTo) {
    SerialBT.printf("Retrieved ID after programming: %d\n", retrievedID);
    SerialBT.printf("REGISTRATION CONFIRMED, NEW ID : %d\n", retrievedID);
  } else {
    SerialBT.printf("ID progamming failed!\n");
    return;
  }

  // Set center of range
  SerialBT.println("\nPlease MANUALLY MOVE the servomotor's FLANGE PLATE to the CENTER OF RANGE position (please check on the instructions manual!)");

  pressEnterWhenReady(false);

  SerialBT.printf("Setting center of range..\n");

  int result = st.CalibrationOfs(ID_ChangeTo);
  delay(1000);
  if (result == 1) {  //Ack 1:Successful 0:Error
    SerialBT.println("Successfully calibrated center of range! ");
  } else {
    SerialBT.printf("Failed center of range calibration! Error: %d\n", result);
  }

  // oscillate servo to confirm its all done
  for (int i = 0; i < 5; i++) {
    st.WritePosEx(ID_ChangeTo, 1948, 1500, 50);  //To control the servo with ID 1, rotate it to position 1000 at a speed of 1500, with a start-stop acceleration of 50.
    delay(400);                                  //[(P1-P0)/V]*1000+100

    st.WritePosEx(ID_ChangeTo, 2148, 1500, 50);  // To control the servo with ID 1, rotate it to position 20 at a speed of 1500, with a start-stop acceleration of 50.
    delay(400);                                  //[(P1-P0)/V]*1000+100
  }
  // Finally move at center of range
  st.WritePosEx(ID_ChangeTo, 2048, 1500, 50);  // To control the servo with ID 1, rotate it to position 20 at a speed of 1500, with a start-stop acceleration of 50.

  SerialBT.printf("Servo ID %d registered! \n Center of range set!\n", ID_ChangeTo);
  SerialBT.println("Please disconnect any servo from the logic board!");
}

void loop() {
  delay(1000);
}
