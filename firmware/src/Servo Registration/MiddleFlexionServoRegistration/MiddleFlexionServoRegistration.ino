#include <SCServo.h>

#define S_RXD 18
#define S_TXD 19

SMS_STS st;
int ID_ChangeFrom = 1;  // Change the original servo ID, and the factory default is 1
int ID_ChangeTo = 2;    // MiddleFlexionServo: ID 2
int ans = -100;
void setup() {
  Serial.begin(115200, SERIAL_8N1);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while (!Serial1) {delay(100);}

  delay(1000);

  // Identify ID..
  Serial.printf("Identifying Servo ID \n");
  Serial.printf("Trying with ID %d..", ID_ChangeFrom);
  int retrievedID = st.Ping(ID_ChangeFrom);
  Serial.printf("..retrieved: %d\n", ID_ChangeFrom);  
  while (retrievedID != ID_ChangeFrom) {
    delay(1000);
    ID_ChangeFrom += 1;
    Serial.printf("Trying with ID %d..", ID_ChangeFrom);
    retrievedID = st.Ping(ID_ChangeFrom);
    Serial.printf("..retrieved: %d\n", retrievedID);
    
  }
  // ID identified
  Serial.printf("Found ID %d\n", ID_ChangeFrom);

  Serial.printf("Programming new ID %d...\n", ID_ChangeTo);
  ans = st.unLockEprom(ID_ChangeFrom);  //Unlock EPROM-SSAFE
  Serial.printf("Unlock ID %d, answer: %d\n", ID_ChangeFrom, ans);

  ans = st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_ChangeTo);  //Change ID
  Serial.printf("WriteByte ID %d, answer: %d\n", ID_ChangeFrom, ans);

  ans = st.LockEprom(ID_ChangeTo);  // EPROM-SAFE is locked
  Serial.printf("Lock ID %d, answer: %d\n", ID_ChangeTo, ans);

  delay(1000);

  // Check programmed ID
  Serial.printf("Trying with ID %d..", ID_ChangeTo);
  retrievedID = st.Ping(ID_ChangeTo); 
  Serial.printf("..retrieved: %d\n", retrievedID);
  delay(1000);
  if (retrievedID == ID_ChangeTo) {
    Serial.printf("Retrieved ID after programming: %d\n", retrievedID);
  } else {
    Serial.printf("ID progamming failed!\n");
    return;
  }

  // Set center of range
  Serial.printf("Setting center of range..\n");

  int result = st.CalibrationOfs(ID_ChangeTo);
  delay(1000);
  if (result == 1) {//Ack 1:Successful 0:Error
    Serial.println("Successfully calibrated center of range! ");
  } else {
    Serial.printf("Failed center of range calibration! Error: %d\n", result);
  }

  // oscillate servo to confirm its all done
  for (int i = 0; i < 5; i++) {
    st.WritePosEx(ID_ChangeTo, 1948, 1500, 50);  //To control the servo with ID 1, rotate it to position 1000 at a speed of 1500, with a start-stop acceleration of 50.
    delay(400);                      //[(P1-P0)/V]*1000+100

    st.WritePosEx(ID_ChangeTo, 2148, 1500, 50);  // To control the servo with ID 1, rotate it to position 20 at a speed of 1500, with a start-stop acceleration of 50.
    delay(400);                    //[(P1-P0)/V]*1000+100
  }
  // Finally move at center of range
  st.WritePosEx(ID_ChangeTo, 2048, 1500, 50);  // To control the servo with ID 1, rotate it to position 20 at a speed of 1500, with a start-stop acceleration of 50.
}

void loop() {
  delay(1000);
}
