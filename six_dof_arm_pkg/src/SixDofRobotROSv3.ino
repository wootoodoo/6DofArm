/* to subscribe to a /servo_commands topic, receiving a String
    and publish to a /joint_states topic with a int16multiarray
    To interface with hardware_interface node
    Referenced https://github.com/bandasaikrishna/3-DOF_Manipulator/blob/master/arduino_code/arduino_code.ino
    1. Run the ROS master -> roscore
    2. Run the serial connection to Arduino -> rosrun rosserial_python serial_node.py /dev/ttyACM0
    3. Run the servo_commands publisher -> rostopic pub -1 servo_commands std_msgs/String "60,70,80,160,0,50"
    4. Run the servo_angles subscriber -> rostopic list /read_joint_states
*/

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>

#define SHOULDER_PAN 8
#define SHOULDER_PITCH 7
#define ELBOW_PITCH 6
#define ELBOW_ROLL 5
#define WRIST_PITCH 4
#define GRIPPER_ROLL 3
#define GRIPPER 2

#define SERVO_DELAY 1

#define MIN_GRIPPER 35     // Servo6; Open: 35; Close: 115; Gripper 
#define MAX_GRIPPER 115
#define MIN_GRIPPER_ROLL 0      // Servo5; MIN 0; MAX180         Gripper roll
#define MAX_GRIPPER_ROLL 180
#define MIN_WRIST_PITCH 10     // Servo4: MIN 10; MAX 160;      Wrist Pitch
#define MAX_WRIST_PITCH 160
#define MIN_ELBOW_ROLL 0     // Servo3: MIN 0; MAX 180;       Elbow roll
#define MAX_ELBOW_ROLL 180
#define MIN_ELBOW_PITCH 0     // Servo2: MIN 0; MAX 180;       Elbow pitch
#define MAX_ELBOW_PITCH 180
#define MIN_SHOULDER_PITCH 0      // Servo1: MIN 0; MAX 180;       Shoulder pitch
#define MAX_SHOULDER_PITCH 180
#define MIN_SHOULDER_PAN 0      // Servo0: MIN 0; MAX 160;       Shoulder pan
#define MAX_SHOULDER_PAN 160
#define DEFAULT6 50     //default: 60,70,80,160,0,50 zero position: 55,80,180,80,180,115
#define DEFAULT5 0      // step 2: 35,0,160,80,70,115
#define DEFAULT4 160    // step 3: 35,0,160,65,70,115
#define DEFAULT3 85    // 
#define DEFAULT2 90     // step 4: 100,0,160,65,70,115
#define DEFAULT1 80     // step 5: 100,0,140,100,70,115 
#define DEFAULT0 100     // step 6: 100,0,140,100,70,10    step 7:100,0,160,65,70,10

ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024>   nh;

const int NUMBER_OF_FIELDS = 7;
int fieldIndex = 0;
int IN[NUMBER_OF_FIELDS];                    // array of commands for servos

// define the Servo instances
Servo shoulder_pan, shoulder_pitch, elbow_pitch, elbow_roll, wrist_pitch, gripper_roll, gripper;

void moveServos()
{
  // Control Servo 0
  if (((IN[0] <= MAX_SHOULDER_PAN) || (IN[0] >= MIN_SHOULDER_PAN)) && IN[0] != 0) { //Commands cannot be more/less than Min/Max, or zero degrees
    int angle = shoulder_pan.read();
    if (IN[0] > angle) {
      for (int x = angle + 1; x < IN[0] + 1; x++) {
        shoulder_pan.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[0] < angle) {
      for (int x = angle - 1; x > IN[0] - 1; x--) {
        shoulder_pan.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[0] = shoulder_pan.read();
  }

  // Control Servo 1
  if (((IN[1] <= MAX_SHOULDER_PITCH) || (IN[1] >= MIN_SHOULDER_PITCH)) && IN[1] != 0) {
    int angle = shoulder_pitch.read();
    if (IN[1] > angle) {
      for (int x = angle + 1; x < IN[1] + 1; x++) {
        shoulder_pitch.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[1] < angle) {
      for (int x = angle - 1; x > IN[1] - 1; x--) {
        shoulder_pitch.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[1] = shoulder_pitch.read();
  }

  // Control Servo 2
  if (((IN[2] <= MAX_ELBOW_PITCH) || (IN[2] >= MIN_ELBOW_PITCH)) && IN[2] != 0) { // To ensure the robot doesn't go crazy if input is zero
    int angle = elbow_pitch.read();
    if (IN[2] > angle) {
      for (int x = angle + 1; x < IN[2] + 1; x++) {
        elbow_pitch.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[2] < angle) {
      for (int x = angle - 1; x > IN[2] - 1; x--) {
        elbow_pitch.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[2] = elbow_pitch.read();
  }

  // Control Servo 3
  if (((IN[3] <= MAX_ELBOW_ROLL) || (IN[3] >= MIN_ELBOW_ROLL)) && IN[3] != 0) {
    int angle = elbow_roll.read();
    if (IN[3] > angle) {
      for (int x = angle + 1; x < IN[3] + 1; x++) {
        elbow_roll.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[3] < angle) {
      for (int x = angle - 1; x > IN[3] - 1; x--) {
        elbow_roll.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[3] = elbow_roll.read();
  }

  // Control Servo 4
  if (((IN[4] <= MAX_WRIST_PITCH) || (IN[4] >= MIN_WRIST_PITCH)) && IN[4] != 0) {
    int angle = wrist_pitch.read();
    if (IN[4] > angle) {
      for (int x = angle + 1; x < IN[4] + 1; x++) {
        wrist_pitch.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[4] < angle) {
      for (int x = angle - 1; x > IN[4] - 1; x--) {
        wrist_pitch.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[4] = wrist_pitch.read();
  }

  // Control Servo 5
  if (((IN[5] <= MAX_GRIPPER_ROLL) || (IN[5] >= MIN_GRIPPER_ROLL)) && IN[5] != 0) {
    int angle = gripper_roll.read();
    if (IN[5] > angle) {
      for (int x = angle + 1; x < IN[5] + 1; x++) {
        gripper_roll.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[5] < angle) {
      for (int x = angle - 1; x > IN[5] - 1; x--) {
        gripper_roll.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[5] = gripper_roll.read();
  }

  // Control gripper
  if (((IN[6] <= MAX_GRIPPER) || (IN[6] >= MIN_GRIPPER)) && IN[6] != 0) {
    int angle = gripper.read();
    if (IN[6] > angle) {
      for (int x = angle + 1; x < IN[6] + 1; x++) {
        gripper.write(x);
        delay(SERVO_DELAY);
      }
    } else if (IN[6] < angle) {
      for (int x = angle - 1; x > IN[6] - 1; x--) {
        gripper.write(x);
        delay(SERVO_DELAY);
      }
    }
  }
  else {
    IN[6] = gripper.read();
  }
}


void messageCallBack (const std_msgs::String& commands)
{
  //Read the message, convert from std_msgs/String to Arduino String, and store into IN[] array
  String Input = commands.data;
  for (int i = 0; i < 7; i++){
      IN[i] = 0;
    }
  for (int i = 0; i < Input.length(); i++)
  {
    if (Input.charAt(i) >= '0' && Input.charAt(i) <= '9' && fieldIndex < NUMBER_OF_FIELDS)   // accumulate the number inputed
    {
      IN[fieldIndex] = (IN[fieldIndex] * 10) + int(Input.charAt(i) - '0');
    }
    else if (Input.charAt(i) == ',')    // Comma will shift to next field
    {
      if (fieldIndex < NUMBER_OF_FIELDS - 1)
        fieldIndex++;
    }
  }
  fieldIndex = 0;
 
}

// read the joint states and publish them into the js array 
std_msgs::Int16MultiArray   js;
char label[] = "readings";
void readJointStates()
{
    js.data[0] = shoulder_pan.read();
    js.data[1] = shoulder_pitch.read();
    js.data[2] = elbow_pitch.read();
    js.data[3] = elbow_roll.read();
    js.data[4] = wrist_pitch.read();
    js.data[5] = gripper_roll.read();
    js.data[6] = gripper.read();
}


ros::Subscriber<std_msgs::String> sub("/servo_commands", &messageCallBack);
ros::Publisher pub ("/read_joint_state", &js);

void setup() {
  // attach servo to servo pin
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  shoulder_pan.attach(SHOULDER_PAN);
  shoulder_pitch.attach(SHOULDER_PITCH);
  elbow_pitch.attach(ELBOW_PITCH);
  elbow_roll.attach(ELBOW_ROLL);
  wrist_pitch.attach(WRIST_PITCH);
  gripper_roll.attach(GRIPPER_ROLL);
  gripper.attach(GRIPPER);
  shoulder_pan.write(DEFAULT0);
  shoulder_pitch.write(DEFAULT1);
  elbow_pitch.write(DEFAULT2);
  elbow_roll.write(DEFAULT3);
  wrist_pitch.write(DEFAULT4);
  gripper_roll.write(DEFAULT5);
  gripper.write(DEFAULT6);

  // Generate the message to send the position of the servos back to ROS
  js.data_length = 7;
  js.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  js.layout.dim[0].label = label;
  js.layout.dim[0].size = 7;
  js.layout.dim[0].stride = 1*7;
  js.layout.data_offset = 0;
  js.data = (int *)malloc(sizeof(int)*7);
}

void loop()
{
  nh.spinOnce();
  moveServos();
  readJointStates();
  pub.publish( &js );
  delay(10);    // Update the joint states once every 10ms (100Hz);
}
