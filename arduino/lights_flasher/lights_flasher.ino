#include <ros.h>
#include <commbot/LightStates.h> // ROS MSG with left and right LED states

// LED pin numbers
const int PIN_LEFT  = 6;
const int PIN_RIGHT = 11;
const int PIN_REVERSE = 3;

// timeout after which to turn off all lights
const float TIMEOUT = 1000; // millisecs
float t_prev_light_states = millis();

// Callback for light states subscriber. Writes states to lights.
void cb_light_states(const commbot::LightStates &light_states){
    
  t_prev_light_states = millis();  

  if(light_states.left) digitalWrite(PIN_LEFT, HIGH);
    else digitalWrite(PIN_LEFT, LOW);
    
  if(light_states.right) digitalWrite(PIN_RIGHT, HIGH);
    else digitalWrite(PIN_RIGHT, LOW);
}

// ROS objects
ros::NodeHandle nh;
ros::Subscriber<commbot::LightStates> sub_light_states("lights_states", cb_light_states);

// Inits node and subscriber
void setup(){
  nh.initNode();
  nh.subscribe(sub_light_states);
    
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  pinMode(PIN_REVERSE, OUTPUT);
  
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);
  digitalWrite(PIN_REVERSE, LOW);
}

// Spins node
void loop()
{

  // watchdog
  if(millis() - t_prev_light_states > TIMEOUT){
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
    digitalWrite(PIN_REVERSE, LOW);
  }

  nh.spinOnce();
  delay(1);

}

