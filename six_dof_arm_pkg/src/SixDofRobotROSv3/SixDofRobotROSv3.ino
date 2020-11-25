/* to subscribe to a /servo_commands topic, receiving a String
    and publish to a /joint_states topic with a int16multiarray
    To interface with hardware_interface node
    
    1. Run the ROS master -> roscore
    2. Run the serial connection to Arduino -> rosrun rosserial_python serial_node.py /dev/ttyACM0
    3. Run the servo_commands publisher -> rostopic pub -1 servo_commands std_msgs/String "100,90,45,55,130,80,180,35"
    4. Run the servo_angles subscriber -> rostopic echo /read_joint_states
*/

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>

#define SHOULDER_PAN 9
#define SHOULDER_PITCH 8
#define ELBOW_ROLL 7
#define ELBOW_PITCH 6
#define WRIST_ROLL 5
#define WRIST_PITCH 4
#define GRIPPER_ROLL 3
#define GRIPPER 2

#define SERVO_DELAY 5

//#define MIN_GRIPPER 35     // Servo7; Open: 35; Close: 115; Gripper 
//#define MAX_GRIPPER 115
//#define MIN_GRIPPER_ROLL 0      // Servo6; MIN 0; MAX180         Gripper roll
//#define MAX_GRIPPER_ROLL 180
//#define MIN_WRIST_PITCH 10     // Servo5: MIN 10; MAX 160;      Wrist Pitch
//#define MAX_WRIST_PITCH 180
//#define MIN_WRIST_ROLL 0     // Servo4: MIN 10; MAX 180;      Wrist roll
//#define MAX_WRIST_ROLL 180
//#define MIN_ELBOW_PITCH 0     // Servo3: MIN 0; MAX 180;       Elbow pitch
//#define MAX_ELBOW_PITCH 180
//#define MIN_ELBOW_ROLL 0     // Servo2: MIN 0; MAX 180;       Elbow roll
//#define MAX_ELBOW_ROLL 180
//#define MIN_SHOULDER_PITCH 0      // Servo1: MIN 0; MAX 180;       Shoulder pitch
//#define MAX_SHOULDER_PITCH 180
//#define MIN_SHOULDER_PAN 0      // Servo0: MIN 0; MAX 160;       Shoulder pan
//#define MAX_SHOULDER_PAN 180

#define DEFAULT7 35      // gripper "100,90,45,55,130,80,180,35"
#define DEFAULT6 180      // gripper roll
#define DEFAULT5 80      // wrist pitch
#define DEFAULT4 130      // wrist roll
#define DEFAULT3 55      // elbow pitch
#define DEFAULT2 45      // elbow roll
#define DEFAULT1 90      // shoulder pitch
#define DEFAULT0 100     // shoulder roll

// new ROS nodehandle object
ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> nh;

// array of commands for servos
const int NUMBER_OF_FIELDS = 8;
int fieldIndex = 0;
int IN[NUMBER_OF_FIELDS] = {DEFAULT0, DEFAULT1, DEFAULT2, DEFAULT3, DEFAULT4, DEFAULT5, DEFAULT6, DEFAULT7};
int max_angles[NUMBER_OF_FIELDS] = {180, 180, 180, 180, 180, 180, 180, 115}; // From shoulder pan to gripper
int min_angles[NUMBER_OF_FIELDS] = {0, 0, 0, 0, 10, 10, 0, 35};                    

// define the Servo instances
Servo shoulder_pan, shoulder_pitch, elbow_pitch, elbow_roll, wrist_roll, wrist_pitch, gripper_roll, gripper;

void messageCallBack (const std_msgs::String& commands)
{
  //Read the message, convert from std_msgs/String to Arduino String, and store into IN[] array
  String Input = commands.data;
  for (int i = 0; i < 8; i++){
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
    js.data[2] = elbow_roll.read();
    js.data[3] = elbow_pitch.read();
    js.data[4] = wrist_roll.read();
    js.data[5] = wrist_pitch.read();
    js.data[6] = gripper_roll.read();
    js.data[7] = gripper.read();
}


void moveServos(Servo servo, int index, int angle) {
  if (angle <= max_angles[index] && angle >= min_angles[index] && angle != 0) { //Commands cannot be more/less than Min/Max, or zero degrees
        servo.write(angle);
        delay(SERVO_DELAY);
  }
  else {
    IN[index] = servo.read();
  }
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
  wrist_roll.attach(WRIST_ROLL);
  gripper_roll.attach(GRIPPER_ROLL);
  gripper.attach(GRIPPER);
  shoulder_pan.write(DEFAULT0);
  shoulder_pitch.write(DEFAULT1);
  elbow_roll.write(DEFAULT2);
  elbow_pitch.write(DEFAULT3);
  wrist_roll.write(DEFAULT4);
  wrist_pitch.write(DEFAULT5);
  gripper_roll.write(DEFAULT6);
  gripper.write(DEFAULT7);

  // Generate the message to send the position of the servos back to ROS
  js.data_length = 8;
  js.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  js.layout.dim[0].label = label;
  js.layout.dim[0].size = 8;
  js.layout.dim[0].stride = 1*8;
  js.layout.data_offset = 0;
  js.data = (int *)malloc(sizeof(int)*8);
}

void loop()
{
  nh.spinOnce();
  readJointStates();
  int movement[NUMBER_OF_FIELDS];
  for (int i = 0; i < NUMBER_OF_FIELDS; i++) {
    movement[i] = IN[i] - js.data[i]; 
  }
  int loops = 10;
  for (int i = 0; i < loops; i++) {
    moveServos(shoulder_pan, 0, js.data[0] + (i + 1) * movement[0]/loops);
    moveServos(shoulder_pitch, 1, js.data[1] + (i + 1) * movement[1]/loops); 
    moveServos(elbow_roll, 2, js.data[2] + (i + 1) * movement[2]/loops);
    moveServos(elbow_pitch, 3, js.data[3] + (i + 1) * movement[3]/loops);
    moveServos(wrist_roll, 4, js.data[4] + (i + 1) * movement[4]/loops);
    moveServos(wrist_pitch, 5, js.data[5] + (i + 1) * movement[5]/loops);
    moveServos(gripper_roll, 6, js.data[6] + (i + 1) * movement[6]/loops);
    moveServos(gripper, 7, js.data[7] + (i + 1) * movement[7]/loops);
  }
  moveServos(shoulder_pan, 0, IN[0]);
  moveServos(shoulder_pitch, 1, IN[1]); 
  moveServos(elbow_roll, 2, IN[2]);
  moveServos(elbow_pitch, 3, IN[3]);
  moveServos(wrist_roll, 4, IN[4]);
  moveServos(wrist_pitch, 5, IN[5]);
  moveServos(gripper_roll, 6, IN[6]);
  moveServos(gripper, 7, IN[7]);
    
  readJointStates();
  pub.publish( &js );
  delay(10);    // Update the joint states once every 10ms (100Hz);
}
