//created by ATLIN ANDERSON
//BOARD: AI THINKER ESP32-CAM
//ESP32 CAM board used

#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <WiFiUdp.h>


/////////////////////////////////////////////////////////////
//SPIDERBOT Global Variables/////////////////////////////////////////////////////////////
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

//spider control variables
int delay_speed = 900;
int servo_speed = 400;
int servo_speed_old = 400;

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

int added_time_to_movement_cycle_min = 500;

bool calibration_tracker = false;


/////////////////////////////////////////////////////////////
//Network setup global variables/////////////////////////////////////////////////////////////
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


///////////////////////////////////////////////////////////////
////ESP32cam Variables/////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

static auto loRes = esp32cam::Resolution::find(320, 240);
static auto hiRes = esp32cam::Resolution::find(800, 600);



/////////////////////////////////////////////////////////////////////////////////////////////
//SPIDERBOT FUNCTIONS////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


void
setup()
{
  //enable the GPIO port for LED headlight
  pinMode(4, OUTPUT);

  Serial.begin(9600);

  //configure ESP32CAM
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

  //set up wifi access point
  WiFi.softAP(WIFI_SSID, WIFI_PASS);             // Start the access point
  Serial.print("Access Point \"");
  Serial.print(WIFI_SSID);
  Serial.println("\" started");
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

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

//set up UDP packets
  Udp.begin(localUdpPort);


  //create a task that to operate ESP32cam on core 1
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
      ////send_udp_reply(message);
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
        ////send_udp_reply(message);
        //Serial.print("0 degrees calibration");
        calibration_tracker = true;
      }
      else
      {
        leg_group_1_command (-45, 45, 45);
        leg_group_2_command (-45, 45, 45);
        char message[] = "Calibration 45 degrees";
        ////send_udp_reply(message);
        calibration_tracker = false;
        //Serial.print("80 degrees calibration");
      }

    }

    /////////////////////////////////////////////
    //update servo speed
    if (incomingPacket[0] == '1')
    {
      update_servo_speed_int(1);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '2')
    {
      update_servo_speed_int(2);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '3')
    {
      update_servo_speed_int(3);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '4')
    {
      update_servo_speed_int(4);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '5')
    {
      update_servo_speed_int(5);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '6')
    {
      update_servo_speed_int(6);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '7')
    {
      update_servo_speed_int(7);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '8')
    {
      update_servo_speed_int(8);
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '9')
    {
      update_servo_speed_int(9);
      incomingPacket[0] = old_Packet[0];
    }


  }
  //Functions to be preformed repeatedly
  //////////////////////////////////////////////////////////////////////////////
  if (incomingPacket[0] == 'S')
  {
    brake();
  }

  if (incomingPacket[0] == 'W')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    walk_forward(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'X')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    walk_backward(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'A')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    strafe_left(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'D')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    strafe_right(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'Q')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    rotate_left(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'E')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    rotate_right(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'H')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    greeting(milli_time_reference, servo_speed);
  }

  if (incomingPacket[0] == 'X')
  {
    movement_state_transition(old_Packet[0], incomingPacket[0]);
    dance(milli_time_reference, servo_speed);
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
