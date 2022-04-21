/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/pointcloud_packed.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/GpuRadarSensor.hh"
#include "ignition/sensors/SensorFactory.hh"

using namespace ignition::sensors;

/// \brief Private data for the GpuRadar class
class ignition::sensors::GpuRadarSensorPrivate
{
  /// \brief Fill the point cloud packed message
  /// \param[in] _laserBuffer Radar data buffer.
  public: void FillPointCloudMsg(const float *_laserBuffer);

  /// \brief Rendering camera
  public: ignition::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Publisher for the publish point cloud message.
  public: transport::Node::Publisher pointPub;

  public: int h_measures, v_measures;
};

//////////////////////////////////////////////////
GpuRadarSensor::GpuRadarSensor()
  : dataPtr(new GpuRadarSensorPrivate())
{
}

//////////////////////////////////////////////////
GpuRadarSensor::~GpuRadarSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuRadarSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->radarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());

    // ignition::rendering::ScenePtr new_scene(_scene->Clone());   TODO: Discarded: no clone method found for scene
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateRadar();
  }
}

//////////////////////////////////////////////////
void GpuRadarSensor::RemoveGpuRays(
    ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
bool GpuRadarSensor::Load(const sdf::Sensor &_sdf)
{
  // Check if this is being loaded via "builtin" or via another sensor
  if (!Radar::Load(_sdf))
  {
    return false;
  }

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured. This same problem is in the
  // RgbdCameraSensor.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->Name(), true,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", msgs::PointCloudPacked::Field::FLOAT32},
      {"ring", msgs::PointCloudPacked::Field::UINT16}});

  if (this->Scene())
    this->CreateRadar();

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&GpuRadarSensor::SetScene, this, std::placeholders::_1));

  // Create the point cloud publisher
  this->SetTopic(this->Topic() + "/points");

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic());

  if (!this->dataPtr->pointPub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  igndbg << "Radar points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool GpuRadarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpuRadarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuRadarSensor::CreateRadar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "Unable to create gpu radar sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->RangeCount());
  this->dataPtr->pointMsg.set_height(
      this->dataPtr->gpuRays->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

//////////////////////////////////////////////////
bool GpuRadarSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("GpuRadarSensor::Update");
  if (!this->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "GpuRays doesn't exist.\n";
    return false;
  }

  int len = this->dataPtr->gpuRays->RayCount() *
    this->dataPtr->gpuRays->VerticalRayCount() * 3;

  if (this->laserBuffer == nullptr)
  {
    this->laserBuffer = new float[len];
  }

  // Before rendering --> move the particle emitters very very far away --> do not appear in the rendering
  auto _scene = this->Scene();
  std::vector<math::Vector3d> pos_vec;
  std::vector<int> index_vec;
  math::Vector3d p(1e10,1e10,1e10);
  // std::cout << "Scene: Node Count " << _scene->NodeCount() << std::endl;
    

    for (unsigned int i = 0; i < _scene->NodeCount(); i++) {
      auto n = _scene->NodeByIndex(i);
      if (n != NULL)  {
        auto name = _scene->NodeByIndex(i)->Name();
        
        if (name.find("emitter") != std::string::npos) {
          pos_vec.push_back(n->LocalPosition());
          index_vec.push_back(i);
          n->SetLocalPosition(p);
        }
      }
    }

  this->Render();

  // Put the emitters in their place again!
  for (unsigned int i = 0; i < index_vec.size(); i++) {
    _scene->NodeByIndex(index_vec[i])->SetLocalPosition(pos_vec[i]);
  }

  /// \todo(anyone) It would be nice to remove this copy.
  this->dataPtr->gpuRays->Copy(this->laserBuffer);

  // Apply noise before publishing the data.
  this->ApplyNoise();

  this->PublishRadarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      msgs::Convert(_now);

    this->dataPtr->FillPointCloudMsg(this->laserBuffer);

    {
      this->AddSequence(this->dataPtr->pointMsg.mutable_header());
      IGN_PROFILE("GpuRadarSensor::Update Publish point cloud");
      this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr GpuRadarSensor::ConnectNewRadarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(_subscriber);
}

/////////////////////////////////////////////////
ignition::rendering::GpuRaysPtr GpuRadarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuRadarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRadarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRadarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////

// The point cloud of the RADAR is not the full point cloud.
// Rather, we keep the closest point of each region of the original LiDAR pointcloud
// Therefore, we 
void GpuRadarSensorPrivate::FillPointCloudMsg(const float *_laserBuffer)
{
  IGN_PROFILE("GpuRadarSensorPrivate::FillPointCloudMsg");
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  float angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  float verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->gpuRays->VerticalAngleMin().Radian();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step() *
      this->pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();
  // Set Pointcloud as dense. Change if invalid points are found.
  bool isDense { true };
  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = this->gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point, and the depth value at that point
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];
      // Validate Depth/Radius and update pointcloud density flag
      if (isDense)
        isDense = !(ignition::math::isnan(depth) || std::isinf(depth));

      float intensity = _laserBuffer[index + 1];
      uint16_t ring = j;

      int fieldIndex = 0;

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::cos(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::sin(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::sin(inclination);

      // Intensity
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = intensity;

      // Ring
      *reinterpret_cast<uint16_t *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = ring;

      // Move the index to the next point.
      msgBufferIndex += this->pointMsg.point_step();

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
  this->pointMsg.set_is_dense(isDense);
}

