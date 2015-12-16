#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>

int ii = 0;
int imax = 76000;
double sign = 1.0;

namespace gazebo
{
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {


// Store the pointer to the model
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&AnimatePose::OnUpdate, this));

    }
 public: void OnUpdate()
    {
//      printf("- %i\n\r",ii);
    if( ii == 0) {
      //printf("here\n\r");
      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("test", 200.0, true));
      double a=0,b=0,c=0.02,time=30;
      gazebo::common::PoseKeyFrame *key;
//    while(1){
      key = anim->CreateKeyFrame(0);
      key->SetTranslation(math::Vector3(0.5, -0.5, c));
      key->SetRotation(math::Quaternion(0, 0, 1.5707));

      key = anim->CreateKeyFrame(time*1);
      key->SetTranslation(math::Vector3(0.5,0.5, c));
      key->SetRotation(math::Quaternion(0, 0, -1.5707));

      key = anim->CreateKeyFrame(time*1.5);
      key->SetTranslation(math::Vector3(1, 0.5, c));
      key->SetRotation(math::Quaternion(0, 0, -1.5707));

      key = anim->CreateKeyFrame(time*2);
      key->SetTranslation(math::Vector3(1,-0.5, c));
      key->SetRotation(math::Quaternion(0, 0, -1.5707));

      key = anim->CreateKeyFrame(time*2.5);
      key->SetTranslation(math::Vector3(0.5,-0.5, c));
      key->SetRotation(math::Quaternion(0, 0, 1.5707));

      this->model->SetAnimation(anim);
//      sign = -1.0*sign;
     // usleep(1000000);
      ii++;
     }
     else{
        ii++;
        if (ii >= imax){ ii = 0;}
         }
//
    }

// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
   // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

  };




  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
