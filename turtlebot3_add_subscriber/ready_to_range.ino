/**
  The Pozyx ready to range tutorial (c) Pozyx Labs
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_range/Arduino

  This demo requires two Pozyx devices and one Arduino. It demonstrates the ranging capabilities and the functionality to
  to remotely control a Pozyx device. Place one of the Pozyx shields on the Arduino and upload this sketch. Move around
  with the other Pozyx device.

  This demo measures the range between the two devices. The closer the devices are to each other, the more LEDs will
  light up on both devices.
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] =  {0x6940, 0x6e2b, 0x6e36, 0x6e50};     // the network id of the anchors: change these to the network ids of your anchors.

uint16_t destination_id = 0x6940;     // the network id of the other pozyx device: fill in the network id of the other device
signed int range_step_mm = 1000;      // every 1000mm in range, one LED less will be giving light.

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.

uint16_t remote_id = 0x6976;          // the network ID of the remote device
bool remote = false;                  // whether to use the given remote device for ranging

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }
  // setting the remote_id back to NULL will use the local Pozyx
  if (!remote){
    remote_id = NULL;
  }
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println("NOTES:");
  Serial.println("- Change the parameters:");
  Serial.println("\tdestination_id (target device)");
  Serial.println("\trange_step (mm)");
  Serial.println();
  Serial.println("- Approach target device to see range and");
  Serial.println("led control");
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println();
  Serial.println("START Ranging:");

  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t led_config = 0x0;
  Pozyx.setLedConfig(led_config, remote_id);
  // do the same with the remote device
  Pozyx.setLedConfig(led_config, destination_id);
  // set the ranging protocol
  Pozyx.setRangingProtocol(ranging_protocol, remote_id);
}

  device_range_t past_range_0;
  device_range_t past_range_1;
  device_range_t past_range_2;
  device_range_t past_range_3;
  device_range_t range_0;         //timestamp, distance, RSS
  device_range_t range_1;         //timestamp, distance, RSS
  device_range_t range_2;         //timestamp, distance, RSS
  device_range_t range_3;         //timestamp, distance, RSS
  int status_0 = 0;
  int status_1 = 0;
  int status_2 = 0;
  int status_3 = 0;

void loop(){
  device_range_t range;
  int status = 0;
  

  // let's perform ranging with the destination
  if(!remote)
  {
//    status = Pozyx.doRanging(destination_id, &range);
    status_0 = Pozyx.doRanging(anchors[0], &range_0);
    status_1 = Pozyx.doRanging(anchors[1], &range_1);
    status_2 = Pozyx.doRanging(anchors[2], &range_2);
    status_3 = Pozyx.doRanging(anchors[3], &range_3);
  }
  else
  {
//    status = Pozyx.doRemoteRanging(remote_id, destination_id, &range);
    status_0 = Pozyx.doRemoteRanging(remote_id, anchors[0], &range_0);
    status_1 = Pozyx.doRemoteRanging(remote_id, anchors[1], &range_1);
    status_2 = Pozyx.doRemoteRanging(remote_id, anchors[2], &range_2);
    status_3 = Pozyx.doRemoteRanging(remote_id, anchors[3], &range_3);
  }


  if ((status_0 == POZYX_SUCCESS)||(status_1 == POZYX_SUCCESS)||(status_2 == POZYX_SUCCESS)||(status_3 == POZYX_SUCCESS))
  {
    Serial.print("a 1 ");
    if(status_0 == POZYX_SUCCESS)
    {
      if(range_0.distance > 10000  || range_0.distance == 0)
      {
        Serial.print(past_range_0.distance);
      }
      else
      {
        Serial.print(range_0.distance);      
        past_range_0 = range_0;
      }
    }
    else
    {
      Serial.print(past_range_0.distance);
    }
    Serial.print(" ");
    if(status_1 == POZYX_SUCCESS)
    {
      if(range_1.distance > 10000 || range_1.distance == 0)
      {
        Serial.print(past_range_1.distance);
      }
      else
      {
        Serial.print(range_1.distance);   
        past_range_1 = range_1;
      }
    }
    else
    {
      Serial.print(past_range_1.distance);
    }
    Serial.print(" ");
    if(status_2 == POZYX_SUCCESS)
    {
      if(range_2.distance > 10000 || range_2.distance == 0)
      {
        Serial.print(past_range_2.distance);
      }
      else
      {
        Serial.print(range_2.distance);   
        past_range_2 = range_2;
      }
    }
    else
    {
      Serial.print(past_range_2.distance);
    }
    Serial.print(" ");
    if(status_3 == POZYX_SUCCESS)
    {
      if(range_3.distance > 10000 || range_3.distance == 0)
      {
        Serial.print(past_range_3.distance);
      }
      else
      {
        Serial.print(range_3.distance);   
        past_range_3 = range_3;
      }
    }
    else
    {
      Serial.print(past_range_3.distance);
    }
    Serial.println(" b");
     
    statusLedShow(status_0, status_1, status_2, status_3);
    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
//    if (ledControl(range_0.distance) == POZYX_FAILURE){
////      Serial.println("ERROR: setting (remote) leds");
//    }
  }
  else{
    //Serial.println("ERROR: ranging");
    //Serial.println("0");
  }
//  if (status == POZYX_SUCCESS){
//    Serial.print(range.timestamp);
//    Serial.print("ms, ");
//    Serial.print(range.distance);
//    Serial.print("mm, ");
//    Serial.print(range.RSS);
//    Serial.println("dBm");
//
//    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
//    if (ledControl(range_0.distance) == POZYX_FAILURE){
//      Serial.println("ERROR: setting (remote) leds");
//    }
//  }
//  else{
////    Serial.println("ERROR: ranging");
//  }
  delay(5);
}


void statusLedShow(int s1, int s2, int s3, int s4)
{
  int st;
  if(s1 == POZYX_SUCCESS)
  {
    st &= Pozyx.setLed(1, true, remote_id);
  }
  else
  {
    st &= Pozyx.setLed(1, false, remote_id);
  }
  if(s2 == POZYX_SUCCESS)
  {
    st &= Pozyx.setLed(2, true, remote_id);
  }
  else
  {
    st &= Pozyx.setLed(2, false, remote_id);
  }
  if(s3 == POZYX_SUCCESS)
  {
    st &= Pozyx.setLed(3, true, remote_id);
  }
  else
  {
    st &= Pozyx.setLed(3, false, remote_id);
  }
  if(s4 == POZYX_SUCCESS)
  {
    st &= Pozyx.setLed(4, true, remote_id);
  }
  else
  {
    st &= Pozyx.setLed(4, false, remote_id);
  }
}

int ledControl(uint32_t range){
  int status = POZYX_SUCCESS;
  // set the LEDs of the pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), remote_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), remote_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), remote_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), remote_id);

  // set the LEDs of the destination pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), anchors[0]);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), anchors[0]);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), anchors[0]);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), anchors[0]);

  // status will be zero if setting the LEDs failed somewhere along the way
  return status;
}
