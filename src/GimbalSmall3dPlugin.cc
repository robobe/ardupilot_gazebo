/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "GimbalSmall3dPlugin.hh"
#include <gazebo/sensors/sensors.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall3dPlugin)

/// \brief Private data class
class gazebo::GimbalSmall3dPluginPrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void OnTiltMsg(ConstGzStringPtr &_msg);
  public: void OnRollMsg(ConstGzStringPtr &_msg);
  public: void OnYawMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr sub_tilt;
  public: transport::SubscriberPtr sub_roll;
  public: transport::SubscriberPtr sub_yaw;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for tilting the gimbal
  public: physics::JointPtr tiltJoint;
  public: physics::JointPtr rollJoint;
  public: physics::JointPtr yawJoint;

  /// \brief Command that updates the gimbal tilt angle
  public: double commandTilt = 0;
  public: double commandRoll = 0;
  public: double commandYaw = 0;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief PID controller for the gimbal
  public: common::PID pidTilt;
  public: common::PID pidRoll;
  public: common::PID pidYaw;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;

  public: sensors::ImuSensorPtr imu_tilt_sensor;
};

/////////////////////////////////////////////////
GimbalSmall3dPlugin::GimbalSmall3dPlugin()
  : dataPtr(new GimbalSmall3dPluginPrivate)
{
  this->dataPtr->pidTilt.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->dataPtr->pidRoll.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->dataPtr->pidYaw.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

/////////////////////////////////////////////////
void GimbalSmall3dPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  std::string tiltName = "tilt_joint";
  std::string rollName = "roll_joint";
  std::string yawName = "yaw_joint";

  // tilt
  if (_sdf->HasElement("tilt_joint"))
  {
    tiltName = _sdf->Get<std::string>("tilt_joint");
  }
  this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(tiltName);

  // roll
  if (_sdf->HasElement("roll_joint"))
  {
    rollName = _sdf->Get<std::string>("roll_joint");
  }
  this->dataPtr->rollJoint = this->dataPtr->model->GetJoint(rollName);
  
  // yaw
  if (_sdf->HasElement("yaw_joint"))
  {
    yawName = _sdf->Get<std::string>("yaw_joint");
  }
  this->dataPtr->yawJoint = this->dataPtr->model->GetJoint(yawName);
  
  sensors::Sensor_V sensorList =
    sensors::SensorManager::Instance()->GetSensors();


    for (auto &sensor : sensorList)
{
    if (!sensor)
        continue;

    std::cout << "Sensor name: " << sensor->ScopedName() << "\n";
    std::cout << "Sensor type: " << sensor->Type() << "\n";
}

      // -------------------
  std::vector<std::string> imuScopedName =
    this->dataPtr->model->SensorScopedName("default::iris_demo::iris_demo::gimbal_small_3d::pitch_link::imu_sensor");
    if (imuScopedName.size() > 1)
  {
    gzwarn << "--------------------------------------"
           << "multiple names match [ XXX ] using first found"
           << " name.\n";
    for (unsigned k = 0; k < imuScopedName.size(); ++k)
    {
      gzwarn << "  sensor " << k << " [" << imuScopedName[k] << "].\n";
    }
  }

  if (imuScopedName.size() > 0)
  {
  this->dataPtr->imu_tilt_sensor = std::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(imuScopedName[0]));
  }
      if (!this->dataPtr->imu_tilt_sensor)
      {
        gzerr << "GimbalSmall3dPlugin::Load ERROR! Can't get imu sensor '"
              << "/gazebo/default/iris_demo/iris_demo/gimbal_small_3d/pitch_link/imu_sensor/imu" << "' " << endl;
      }
      else{
        gzwarn << "GimbalSmall3dPlugin::Load got imu sensor '"
              << "/gazebo/default/iris_demo/iris_demo/gimbal_small_3d/pitch_link/imu_sensor/imu" << "' " << endl;

        
      }
  // if (!this->dataPtr->tiltJoint)
  // {
  //   std::string scopedJointName = _model->GetScopedName() + "::" + tiltName;
  //   gzwarn << "joint [" << jointName
  //          << "] not found, trying again with scoped joint name ["
  //          << scopedJointName << "]\n";
  //   this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(scopedJointName);
  // }
  // if (!this->dataPtr->tiltJoint)
  // {
  //   gzerr << "GimbalSmall2dPlugin::Load ERROR! Can't get joint '"
  //         << jointName << "' " << endl;
  // }
}

/////////////////////////////////////////////////
void GimbalSmall3dPlugin::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  this->dataPtr->lastUpdateTime =
    this->dataPtr->model->GetWorld()->SimTime();

  std::string tilt_topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_tilt_cmd";

  std::string roll_topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_roll_cmd";

  std::string yaw_topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_yaw_cmd";


  this->dataPtr->sub_tilt = this->dataPtr->node->Subscribe(tilt_topic,
      &GimbalSmall3dPluginPrivate::OnTiltMsg, this->dataPtr.get());

  this->dataPtr->sub_roll = this->dataPtr->node->Subscribe(roll_topic,
      &GimbalSmall3dPluginPrivate::OnRollMsg, this->dataPtr.get());

  this->dataPtr->sub_yaw = this->dataPtr->node->Subscribe(yaw_topic,
      &GimbalSmall3dPluginPrivate::OnYawMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalSmall3dPlugin::OnUpdate, this)));

  // topic = std::string("~/") +
  //   this->dataPtr->model->GetName() + "/gimbal_tilt_status";

  // this->dataPtr->pub =
  //   this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);
  gzwarn << "---------------- GimbalSmall3dPlugin loaded for model ["
         << this->dataPtr->model->GetName() << "] ----------------\n";
}

/////////////////////////////////////////////////
void GimbalSmall3dPluginPrivate::OnTiltMsg(ConstGzStringPtr &_msg)
{
  this->commandTilt = atof(_msg->data().c_str());
}

void GimbalSmall3dPluginPrivate::OnRollMsg(ConstGzStringPtr &_msg)
{
  this->commandRoll = atof(_msg->data().c_str());
}

void GimbalSmall3dPluginPrivate::OnYawMsg(ConstGzStringPtr &_msg)
{
  this->commandYaw = atof(_msg->data().c_str());
}
/////////////////////////////////////////////////
void GimbalSmall3dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint)
    return;

  double tilt_angle = this->dataPtr->tiltJoint->Position(0);
  double roll_angle = this->dataPtr->rollJoint->Position(0);
  double yaw_angle = this->dataPtr->yawJoint->Position(0);

  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  else if (time > this->dataPtr->lastUpdateTime)
  {
    double dt = (this->dataPtr->lastUpdateTime - time).Double();
    double error = tilt_angle - this->dataPtr->commandTilt;
    double force = this->dataPtr->pidTilt.Update(error, dt);
    this->dataPtr->tiltJoint->SetForce(0, force);


    double error_roll = roll_angle - this->dataPtr->commandRoll;
    double force_roll = this->dataPtr->pidRoll.Update(error_roll, dt);
    this->dataPtr->rollJoint->SetForce(0, force_roll);

    double error_yaw = yaw_angle - this->dataPtr->commandYaw;
    double force_yaw = this->dataPtr->pidYaw.Update(error_yaw, dt);
    this->dataPtr->yawJoint->SetForce(0, force_yaw);

    this->dataPtr->lastUpdateTime = time;

    // gzwarn << "imu orientation: "
    //            << this->dataPtr->imu_tilt_sensor->Orientation().Euler().Y() << "\n";
    gzwarn << "imu orientation: "
               << this->dataPtr->imu_tilt_sensor->Orientation().Euler().X() << "\n";
  }

  // static int i = 1000;
  // if (++i > 100)
  // {
  //   i = 0;
  //   std::stringstream ss;
  //   ss << angle;
  //   gazebo::msgs::GzString m;
  //   m.set_data(ss.str());
  //   this->dataPtr->pub->Publish(m);
  // }
}
