
#include <iostream>
#include <gazebo/math/gzmath.hh>

int main(int argc, char **argv) {

  gazebo::math::Pose p_ref, p_servo, p_servo2;

  // Initialize ref and servo with arbitrary poses
  p_ref.pos.x = 1;
  p_ref.pos.y = 1;
  p_ref.pos.z = 1;

  p_ref.rot.w = 0.6971828456178173;
  p_ref.rot.x = -0.1894264840231191;
  p_ref.rot.y = -0.2117691903294058;
  p_ref.rot.z = -0.6581850020732086;

  p_servo.pos.x = 0;
  p_servo.pos.y = 0;
  p_servo.pos.z = 0;

  p_servo.rot.w = 0.9141389813398726;
  p_servo.rot.x = 0.1557981963675354;
  p_servo.rot.y = -0.3179224039958124;
  p_servo.rot.z = 0.19748972084907693;

  p_servo2 = p_servo;

  // Integration "gain"
  double k=0.005;
  int count=0;

  while(p_servo.pos.x != 1 && p_servo2.pos.x != 1) {
    // Subtract the whole poses
    gazebo::math::Pose p_rel = p_ref - p_servo;
    p_servo.pos += k * p_rel.pos;  

    // Just subtract the positions
    p_servo2.pos += k * (p_ref.pos - p_servo2.pos);

    // Only output every 100 steps
    if(count % 100 == 0) {
      std::cout<<p_servo.pos.x<<", "<<p_servo2.pos.x<<std::endl;
    }

    // Don't do it too fast
    usleep(1000);
  }



  return 0;
}
