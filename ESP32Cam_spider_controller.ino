//created by ATLIN ANDERSON
//BOARD: AI THINKER ESP32-CAM
//ESP32 CAM board used

#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <WiFiUdp.h>


/////////////////////////////////////////////////////////////
//SPIDERBOT SECTION/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////




//    //spider leg assignment diagram
//    //    Front
//    //
//    //leg 2     leg 1
//    //     \   /
//    // leg4- 0 -leg 3
//    //     /  \
//    // leg 6   leg 5
//
//    //group1: 1, 4, 5
//    //group2: 2, 3, 6

//global variableS
//half of 270 for servo
int max_servo_angle = 135;

//leg servo binding
int servo_coxa_1 = 29;
int servo_femur_1 = 30;
int servo_tibia_1 = 31;
//4
int servo_coxa_2 = 1;
int servo_femur_2 = 2;
int servo_tibia_2 = 3;
//8
int servo_coxa_3 = 25;
int servo_femur_3 = 26;
int servo_tibia_3 = 27;
//12
int servo_coxa_4 = 5;
int servo_femur_4 = 6;
int servo_tibia_4 = 7;
//16
int servo_coxa_5 = 21;
int servo_femur_5 = 22;
int servo_tibia_5 = 23;
//20
int servo_coxa_6 = 9;
int servo_femur_6 = 10;
int servo_tibia_6 = 11;



////leg servo binding
//int servo_coxa_1 = 1;
//int servo_femur_1 = 2;
//int servo_tibia_1 = 3;
////4
//int servo_coxa_2 = 5;
//int servo_femur_2 = 6;
//int servo_tibia_2 = 7;
////8
//int servo_coxa_3 = 9;
//int servo_femur_3 = 10;
//int servo_tibia_3 = 11;
////12
//int servo_coxa_4 = 13;
//int servo_femur_4 = 14;
//int servo_tibia_4 = 15;
////16
//int servo_coxa_5 = 17;
//int servo_femur_5 = 18;
//int servo_tibia_5 = 19;
////20
//int servo_coxa_6 = 21;
//int servo_femur_6 = 22;
//int servo_tibia_6 = 23;
//
//spider control variables
int delay_speed = 900;
int servo_speed = 400;
int servo_speed_old = 400;

//trackers of current leg position goal
//int leg_group_1_coxa_current_angle = 0;
//int leg_group_1_femur_current_angle = 0;
//int leg_group_1_tibia_current_angle = 0;
//
//int leg_group_2_coxa_current_angle = 0;
//int leg_group_2_femur_current_angle = 0;
//int leg_group_2_tibia_current_angle = 0;

//default standing angles
int default_standing_coxa_angle = 0;
int default_standing_femur_angle = 5;
int default_standing_tibia_angle = 85;

//used for timing of movement command
unsigned long milli_time_reference = 0;
unsigned long time_interval_old = 0;

//stores all commands to be sent to servo controller
String servo_serial_command_buffer = "";
String servo_serial_command_buffer_old = "";

bool calibration_tracker = false;


/////////////////////////////////////////////////////////////
//Network setup SECTION/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

const char* WIFI_SSID = "ESP32 Spiderbot";
const char* WIFI_PASS = "0123456789";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[1];  // buffer for incoming packets
char old_Packet[1];  // buffer for old packets
char  replyPacket[] = "";  // a reply string to send back

WebServer server(80);

TaskHandle_t Task1;

void
send_udp_reply(char reply_buffer[])
{
  Udp.beginPacket(WiFi.localIP(), localUdpPort);

  Udp.print(reply_buffer);

  Udp.endPacket();
}

///////////////////////////////////////////////////////////////
////ESP32cam SECTION/////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////



//
static auto loRes = esp32cam::Resolution::find(320, 240);
static auto hiRes = esp32cam::Resolution::find(800, 600);

void
handleBmp()
{
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }

  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
                static_cast<int>(frame->size()));

  if (!frame->toBmp()) {
    Serial.println("CONVERT FAIL");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CONVERT OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
                static_cast<int>(frame->size()));

  server.setContentLength(frame->size());
  server.send(200, "image/bmp");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void
serveJpg()
{
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
                static_cast<int>(frame->size()));

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void
handleJpgLo()
{
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }
  serveJpg();
}

void
handleJpgHi()
{
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("SET-HI-RES FAIL");
  }
  serveJpg();
}

void
handleJpg()
{
  server.sendHeader("Location", "/cam-hi.jpg");
  server.send(302, "", "");
}

void
handleMjpeg()
{
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("SET-HI-RES FAIL");
  }

  Serial.println("STREAM BEGIN");
  WiFiClient client = server.client();
  auto startTime = millis();
  int res = esp32cam::Camera.streamMjpeg(client);
  if (res <= 0) {
    Serial.printf("STREAM ERROR %d\n", res);
    return;
  }
  auto duration = millis() - startTime;
  Serial.printf("STREAM END %dfrm %0.2ffps\n", res, 1000.0 * res / duration);
}

//Task1code: blinks an LED every 1000 ms
void
Task1code( void * pvParameters )
{
  for (;;) {
    server.handleClient();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//SPIDERBOT FUNCTIONS////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void
servo_position(int servo_num, int servo_angle, int servo_speed)
{
  //speed 100-9999
  servo_angle = map(servo_angle, -max_servo_angle, max_servo_angle, -90, 90);
  servo_angle = map(servo_angle, -90, 90, 500, 2500);
  servo_angle = constrain(servo_angle, 500, 2500);
  Serial.print("#" + String(servo_num) + "P" + String(servo_angle) + "T" + String(servo_speed) + "\r\n");
  return;
}

void
multi_servo_position(int servo_num1, int servo_angle1, int servo_num2, int servo_angle2, int servo_num3, int servo_angle3)
{
  //speed 100-9999
  servo_angle1 = map(servo_angle1, -max_servo_angle, max_servo_angle, -90, 90);
  servo_angle1 = map(servo_angle1, -90, 90, 500, 2500);
  servo_angle1 = constrain(servo_angle1, 500, 2500);

  servo_angle2 = map(servo_angle2, -max_servo_angle, max_servo_angle, -90, 90);
  servo_angle2 = map(servo_angle2, -90, 90, 500, 2500);
  servo_angle2 = constrain(servo_angle2, 500, 2500);

  servo_angle3 = map(servo_angle3, -max_servo_angle, max_servo_angle, -90, 90);
  servo_angle3 = map(-servo_angle3, -90, 90, 500, 2500);
  servo_angle3 = constrain(servo_angle3, 500, 2500);

  //Serial.print("#" + String(servo_num1) + "P" + String(servo_angle1) + "#" + String(servo_num2) + "P" + String(servo_angle2) + "#" + String(servo_num3) + "P" + String(servo_angle3) + "T" + String(servo_speed) + "\r\n");
  servo_serial_command_buffer = servo_serial_command_buffer + "#" + String(servo_num1) + "P" + String(servo_angle1) + "#" + String(servo_num2) + "P" + String(servo_angle2) + "#" + String(servo_num3) + "P" + String(servo_angle3);

  return;
}

void
servo_test()
{
  for (int i = 0; i <= 23; i++)
  {
    servo_position(i, -90, 100);
    delay(1000);
    servo_position(i, 90, 100);
    delay(1000);
    servo_position(i, 0, 100);
    delay(1000);
  }
}

void
specific_leg_relative_command(int leg_ID, int coxa_servo_angle, int femur_servo_angle, int tibia_servo_angle)
{
  //spider leg assignment diagram
  //    Front
  //
  //leg 2     leg 1
  //     \   /
  // leg4- 0 -leg 3
  //     /   \
  // leg 6   leg 5

  //group1: 1, 4, 5
  //group2: 2, 3, 6

  switch (leg_ID) {

    //command group1
    case 1:
      multi_servo_position(servo_coxa_1 , -coxa_servo_angle, servo_femur_1 , -femur_servo_angle , servo_tibia_1 , -tibia_servo_angle);
      break;
    case 2:
      multi_servo_position(servo_coxa_2 , coxa_servo_angle, servo_femur_2 , femur_servo_angle , servo_tibia_2 , tibia_servo_angle);
      break;
    case 3:
      multi_servo_position(servo_coxa_3 , -coxa_servo_angle, servo_femur_3 , -femur_servo_angle , servo_tibia_3 , -tibia_servo_angle);
      break;
    //command group 2
    case 4:
      multi_servo_position(servo_coxa_4 , coxa_servo_angle, servo_femur_4 , femur_servo_angle , servo_tibia_4 , tibia_servo_angle);
      break;
    case 5:
      multi_servo_position(servo_coxa_5 , coxa_servo_angle, servo_femur_5 , femur_servo_angle , servo_tibia_5 , tibia_servo_angle);
      break;
    case 6:
      multi_servo_position(servo_coxa_6 , -coxa_servo_angle, servo_femur_6 , -femur_servo_angle , servo_tibia_6 , -tibia_servo_angle);
      break;
    default:
      break;
  }
}

void
leg_group_1_command (int coxa_servo_angle, int femur_servo_angle, int tibia_servo_angle)
{
  //legs 1, 4, 5

  //  leg_group_1_coxa_current_angle = coxa_servo_angle;
  //  leg_group_1_femur_current_angle = femur_servo_angle;
  //  leg_group_1_tibia_current_angle = tibia_servo_angle;

  //  //coxa

  multi_servo_position(servo_coxa_1 , -coxa_servo_angle, servo_coxa_4 , coxa_servo_angle, servo_coxa_5 , coxa_servo_angle);

  //  //femur

  multi_servo_position(servo_femur_1 , -femur_servo_angle, servo_femur_4 , femur_servo_angle, servo_femur_5 , femur_servo_angle);
  //
  //  //tibia

  multi_servo_position(servo_tibia_1 , -tibia_servo_angle, servo_tibia_4 , tibia_servo_angle, servo_tibia_5 , tibia_servo_angle);

}

void
leg_group_2_command (int coxa_servo_angle, int femur_servo_angle, int tibia_servo_angle)
{
  //legs 2, 3, 6

  //  leg_group_2_coxa_current_angle = coxa_servo_angle;
  //  leg_group_2_femur_current_angle = femur_servo_angle;
  //  leg_group_2_tibia_current_angle = tibia_servo_angle;

  //  //coxa

  multi_servo_position(servo_coxa_2 , coxa_servo_angle, servo_coxa_3 , -coxa_servo_angle, servo_coxa_6 , -coxa_servo_angle);

  //  //femur

  multi_servo_position(servo_femur_2 , femur_servo_angle, servo_femur_3 , -femur_servo_angle, servo_femur_6 , -femur_servo_angle);

  //  //tibia

  multi_servo_position(servo_tibia_2 , tibia_servo_angle, servo_tibia_3 , -tibia_servo_angle, servo_tibia_6 , -tibia_servo_angle);

}

void
brake()
{
  //stand still
  leg_group_1_command (default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);
  leg_group_2_command (default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);
}

void
walk_forward(int reference_time_millis, int servo_speed)
{

  //total seperate actions preformed
  int action_group_count = 4;

  //how long it take to repeat the action (ms)
  //int movement_cycle_period = 3000;
  int movement_cycle_period = servo_speed * action_group_count + servo_speed * 2;

  //how many ms have elapsed during the current movemnt cycle
  int current_movement_cycle_elapsed = (millis() - reference_time_millis) % movement_cycle_period;

  //determine which action group to do based on how much time has elapsed in the period of the movement cycle
  if (current_movement_cycle_elapsed < movement_cycle_period / action_group_count)
  {
    //ACTION GROUP 1
    leg_group_2_command (-45, -45, -85); //leg group 2 forward and up
    leg_group_1_command (0, -5, -85);  //leg group 1 back to default pos
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 1 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 2)
  {
    //ACTION GROUP 2
    leg_group_2_command (-45, -5, -85); //leg group 2 down
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 2 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 3)
  {
    //ACTION GROUP 3
    leg_group_2_command (0, -5, -85);  //leg group 2 back to default pos
    leg_group_1_command (-45, -45, -85); // leg group 1 fowrd and up
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 3 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 4)
  {
    //ACTION GROUP 4
    leg_group_1_command (-45, -5, -85); //leg group 1 down
  }
}

void
walk_backward(int reference_time_millis, int servo_speed)
{

  //total seperate actions preformed
  int action_group_count = 4;

  //how long it take to repeat the action (ms)
  //int movement_cycle_period = 3000;
  int movement_cycle_period = servo_speed * action_group_count + servo_speed * 2;

  //how many ms have elapsed during the current movemnt cycle
  int current_movement_cycle_elapsed = (millis() - reference_time_millis) % movement_cycle_period;

  //determine which action group to do based on how much time has elapsed in the period of the movement cycle
  if (current_movement_cycle_elapsed < movement_cycle_period / action_group_count)
  {
    //ACTION GROUP 1
    leg_group_2_command (-45, 45, 85); //leg group 2 forward and up
    leg_group_1_command (0, 5, 85);  //leg group 1 back to default pos
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 1 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 2)
  {
    //ACTION GROUP 2
    leg_group_2_command (-45, 5, 85); //leg group 2 down
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 2 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 3)
  {
    //ACTION GROUP 3
    leg_group_2_command (0, 5, 85);  //leg group 2 back to default pos
    leg_group_1_command (-45, 45, 85); // leg group 1 fowrd and up
  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 3 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 4)
  {
    //ACTION GROUP 4
    leg_group_1_command (-45, 5, 85); //leg group 1 down
  }
}

void
rotate_left(int reference_time_millis, int servo_speed)
{

  //  int default_standing_coxa_angle = 0;
  //int default_standing_femur_angle = 5;
  //int default_standing_tibia_angle = 85;

  //total seperate actions preformed
  int action_group_count = 4;

  //how long it take to repeat the action (ms)
  //int movement_cycle_period = 3000;
  int movement_cycle_period = servo_speed * action_group_count + servo_speed * 2;

  //how many ms have elapsed during the current movemnt cycle
  int current_movement_cycle_elapsed = (millis() - reference_time_millis) % movement_cycle_period;

  //determine which action group to do based on how much time has elapsed in the period of the movement cycle
  if (current_movement_cycle_elapsed < movement_cycle_period / action_group_count)
  {
    //ACTION GROUP 1
    //leg_group_2_command (-45, -45, 85); //leg group 2 forward and up
    //leg_group_1_command (0, -5, 85);  //leg group 1 back to default pos

    specific_leg_relative_command(1, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(2, -60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(3, 60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(4, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(5, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(6, -60, 45, default_standing_tibia_angle);


  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 1 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 2)
  {
    //ACTION GROUP 2
    //leg_group_2_command (-45, -5, 85); //leg group 2 down

    specific_leg_relative_command(2, -60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(3, 60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(6, -60, default_standing_femur_angle, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 2 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 3)
  {
    //ACTION GROUP 3
    //leg_group_1_command (-45, -45, 85); // leg group 1 fowrd and up

    specific_leg_relative_command(1, 60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(2, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(3, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(4, -60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(5, 60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(6, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 3 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 4)
  {
    //ACTION GROUP 4
    //leg_group_1_command (-45, -5, 85); //leg group 1 down

    specific_leg_relative_command(1, 60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(4, -60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(5, 60, default_standing_femur_angle, default_standing_tibia_angle);

  }

}

void
rotate_right(int reference_time_millis, int servo_speed)
{
  //  int default_standing_coxa_angle = 0;
  //int default_standing_femur_angle = 5;
  //int default_standing_tibia_angle = 85;

  //total seperate actions preformed
  int action_group_count = 4;

  //how long it take to repeat the action (ms)
  //int movement_cycle_period = 3000;
  int movement_cycle_period = servo_speed * action_group_count + servo_speed * 2;

  //how many ms have elapsed during the current movemnt cycle
  int current_movement_cycle_elapsed = (millis() - reference_time_millis) % movement_cycle_period;

  //determine which action group to do based on how much time has elapsed in the period of the movement cycle
  if (current_movement_cycle_elapsed < movement_cycle_period / action_group_count)
  {
    //ACTION GROUP 1
    //leg_group_2_command (-45, -45, 85); //leg group 2 forward and up
    //leg_group_1_command (0, -5, 85);  //leg group 1 back to default pos

    specific_leg_relative_command(1, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(2, 60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(3, -60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(4, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(5, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(6, 60, 45, default_standing_tibia_angle);


  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 1 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 2)
  {
    //ACTION GROUP 2
    //leg_group_2_command (-45, -5, 85); //leg group 2 down

    specific_leg_relative_command(2, 60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(3, -60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(6, 60, default_standing_femur_angle, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 2 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 3)
  {
    //ACTION GROUP 3
    //leg_group_1_command (-45, -45, 85); // leg group 1 fowrd and up

    specific_leg_relative_command(1, -60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(2, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(3, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(4, 60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(5, -60, 45, default_standing_tibia_angle);

    specific_leg_relative_command(6, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 3 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 4)
  {
    //ACTION GROUP 4
    //leg_group_1_command (-45, -5, 85); //leg group 1 down

    specific_leg_relative_command(1, -60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(4, 60, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(5, -60, default_standing_femur_angle, default_standing_tibia_angle);

  }
}

void
strafe_right(int reference_time_millis, int servo_speed)
{

  //  int default_standing_coxa_angle = 0;
  //int default_standing_femur_angle = 5;
  //int default_standing_tibia_angle = 85;

  //total seperate actions preformed
  int action_group_count = 5;

  //how long it take to repeat the action (ms)
  //int movement_cycle_period = 3000;
  int movement_cycle_period = servo_speed * action_group_count + servo_speed * 2;

  //how many ms have elapsed during the current movemnt cycle
  int current_movement_cycle_elapsed = (millis() - reference_time_millis) % movement_cycle_period;

  //determine which action group to do based on how much time has elapsed in the period of the movement cycle
  if (current_movement_cycle_elapsed < movement_cycle_period / action_group_count)
  {
    //ACTION GROUP 1

    specific_leg_relative_command(1, -45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(2, -45, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(3, default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);

    specific_leg_relative_command(4, default_standing_coxa_angle, 45, default_standing_tibia_angle);

    specific_leg_relative_command(5, 45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(6, 45, default_standing_femur_angle, default_standing_tibia_angle);


  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 1 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 2)
  {
    //ACTION GROUP 2

    specific_leg_relative_command(1, -45, -45, 0);

    specific_leg_relative_command(2, -45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(3, default_standing_coxa_angle, 45, default_standing_tibia_angle);

    specific_leg_relative_command(4, default_standing_coxa_angle, -25, 89);

    specific_leg_relative_command(5, 45, -45, 0);

    specific_leg_relative_command(6, 45, 45, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 2 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 3)
  {
    //ACTION GROUP 3

    specific_leg_relative_command(1, -45, -45, default_standing_tibia_angle);

    specific_leg_relative_command(4, default_standing_coxa_angle, -45, default_standing_tibia_angle);

    specific_leg_relative_command(5, 45, -45, default_standing_tibia_angle);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 3 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 4)
  {
    //ACTION GROUP 4

    specific_leg_relative_command(1, -45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(2, -45, -25, 89);

    specific_leg_relative_command(3, default_standing_coxa_angle, -45, 0);

    specific_leg_relative_command(4, default_standing_coxa_angle, 45, default_standing_tibia_angle);

    specific_leg_relative_command(5, 45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(6, 45, -25, 89);

  }
  else if (current_movement_cycle_elapsed > (movement_cycle_period / action_group_count) * 4 && current_movement_cycle_elapsed < (movement_cycle_period / action_group_count) * 5)
  {
    //ACTION GROUP 5

    specific_leg_relative_command(2, -45, 45, default_standing_tibia_angle);

    specific_leg_relative_command(3, default_standing_coxa_angle, 45, default_standing_tibia_angle);

    specific_leg_relative_command(6, 45, 45, default_standing_tibia_angle);

  }
}

void
update_servo_speed_int(int speed_int)
{
  int max_servo_speed = 150;
  int min_servo_speed = 900;
  speed_int = constrain(speed_int, 1, 9);
  servo_speed = map(speed_int, 1, 9, min_servo_speed, max_servo_speed);
  servo_speed_old = servo_speed;
}

void
setup()
{
  //enable the GPIO port for LED headlight
  pinMode(4, OUTPUT);


  Serial.begin(9600);


  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }

  //  WiFi.persistent(false);
  //  WiFi.mode(WIFI_STA);
  //  WiFi.begin(WIFI_SSID, WIFI_PASS);

  WiFi.softAP(WIFI_SSID, WIFI_PASS);             // Start the access point
  Serial.print("Access Point \"");
  Serial.print(WIFI_SSID);
  Serial.println("\" started");

  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

  //  while (WiFi.status() != WL_CONNECTED) {
  //    delay(500);
  //  }

  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("  /cam.bmp");
  Serial.println("  /cam-lo.jpg");
  Serial.println("  /cam-hi.jpg");
  Serial.println("  /cam.mjpeg");

  server.on("/cam.bmp", handleBmp);
  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam-hi.jpg", handleJpgHi);
  server.on("/cam.jpg", handleJpg);
  server.on("/cam.mjpeg", handleMjpeg);

  server.begin();

  Udp.begin(localUdpPort);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */


  ///SPIDERBOT SETUP////////////////////////////////////////////////////////////////////////

  //lay flat on ground
  leg_group_1_command (0, 0, 0);
  leg_group_2_command (0, 0, 0);

  delay(2000);

  //default standing position
  leg_group_1_command (default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);
  leg_group_2_command (default_standing_coxa_angle, default_standing_femur_angle, default_standing_tibia_angle);
  delay(2000);

  //set time reference
  milli_time_reference = millis();
}


void
loop() {

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    //Serial.printf("UDP packet contents: %s\n", incomingPacket);
    Serial.print(incomingPacket);
    //    // send back a reply, to the IP address and port we got the packet from
    //    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //    Udp.write(replyPacket);
    //    Udp.endPacket();

    //Functions to be preformed only when a packet is recievied
    ///////////////////////////////////////////////////////////////////
    if (incomingPacket[0] == 'F')
    {
      digitalWrite(4, !digitalRead(4));
      char message[] = "Headlight Toggled";
      send_udp_reply(message);
      //Serial.print("toggle light");
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == 'C')
    {
      if (calibration_tracker == false)
      {
        leg_group_1_command (0, 0, 0);
        leg_group_2_command (0, 0, 0);
        char message[] = "Calibration 0 degrees";
        send_udp_reply(message);
        //Serial.print("0 degrees calibration");
        calibration_tracker = true;
      }
      else
      {
        leg_group_1_command (-45, 45, 45);
        leg_group_2_command (-45, 45, 45);
        char message[] = "Calibration 45 degrees";
        send_udp_reply(message);
        calibration_tracker = false;
        //Serial.print("80 degrees calibration");
      }

    }

    /////////////////////////////////////////////
    //update servo speed
    if (incomingPacket[0] == '1')
    {
      update_servo_speed_int(1);
      char message[] = "Servo Speed 1";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '2')
    {
      update_servo_speed_int(2);
      char message[] = "Servo Speed 2";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '3')
    {
      update_servo_speed_int(3);
      char message[] = "Servo Speed 3";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '4')
    {
      update_servo_speed_int(4);
      char message[] = "Servo Speed 4";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '5')
    {
      update_servo_speed_int(5);
      char message[] = "Servo Speed 5";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '6')
    {
      update_servo_speed_int(6);
      char message[] = "Servo Speed 6";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '7')
    {
      update_servo_speed_int(7);
      char message[] = "Servo Speed 7";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '8')
    {
      update_servo_speed_int(8);
      char message[] = "Servo Speed 8";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '9')
    {
      update_servo_speed_int(9);
      char message[] = "Servo Speed 9";
      send_udp_reply(message);
      incomingPacket[0] = old_Packet[0];
    }


  }
  //Functions to be preformed repeatedly
  //////////////////////////////////////////////////////////////////////////////
  if (incomingPacket[0] == 'S')
  {
    brake();
    char message[] = "Stop Spiderbot";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'W')
  {
    walk_forward(milli_time_reference, servo_speed);
    char message[] = "Forward";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'X')
  {
    walk_backward(milli_time_reference, servo_speed);
    char message[] = "Backward";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'A')
  {
    //strafe_left(milli_time_reference, servo_speed);
    char message[] = "Strafe Left";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'D')
  {
    strafe_right(milli_time_reference, servo_speed);
    char message[] = "Strafe Right";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'Q')
  {
    rotate_left(milli_time_reference, servo_speed);
    char message[] = "Rotate Left";
    send_udp_reply(message);
  }

  if (incomingPacket[0] == 'E')
  {
    rotate_right(milli_time_reference, servo_speed);
    char message[] = "Rotate Right";
    send_udp_reply(message);
  }


  //send the final serial command string appended with servo speed
  if (servo_serial_command_buffer.length() > 0 && servo_serial_command_buffer != servo_serial_command_buffer_old || servo_speed_old != servo_speed)
  {
    Serial.println(servo_serial_command_buffer + "T" + String(servo_speed));
  }
  //then reset the string for new command
  servo_serial_command_buffer_old = servo_serial_command_buffer;
  servo_serial_command_buffer = "";

  old_Packet[0] = incomingPacket[0];

  delay(5);

}
