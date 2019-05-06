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
#ifndef IGNITION_SENSORS_DEPTHCAMERASENSOR_HH_
#define IGNITION_SENSORS_DEPTHCAMERASENSOR_HH_

#include <memory>
#include <cstdint>
#include <mutex>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common/Event.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Time.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs.hh>

#include <ignition/rendering/DepthCamera.hh>

#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/Events.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/Sensor.hh>

#include <ignition/transport.hh>

#ifndef _WIN32
#  define DepthCameraSensor_EXPORTS_API
#else
#  if (defined(DepthCameraSensor_EXPORTS))
#    define DepthCameraSensor_EXPORTS_API __declspec(dllexport)
#  else
#    define DepthCameraSensor_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // forward declarations
    class DepthCameraSensorPrivate;

    /// \brief Depth camera sensor class.
    ///
    /// This class creates depth image from an ignition rendering scene.
    /// The scene  must be created in advance and given to Manager::Init().
    /// It offers both an ignition-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class DepthCameraSensor_EXPORTS_API DepthCameraSensor : public CameraSensor
    {
      /// \brief constructor
      public: DepthCameraSensor();

      /// \brief destructor
      public: virtual ~DepthCameraSensor();

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual rendering::DepthCameraPtr DepthCamera();

      /// \brief Depth data callback used to get the data from the sensor
      /// \param[in] _scan pointer to the data from the sensor
      /// \param[in] _width width of the depth image
      /// \param[in] _height height of the depth image
      /// \param[in] _channel bytes used for the depth data
      /// \param[in] _format string with the format
      public: void OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/);

      /// \brief Set a callback to be called when image frame data is
      /// generated.
      /// \param[in] _callback This callback will be called every time the
      /// camera produces image data. The Update function will be blocked
      /// while the callbacks are executed.
      /// \remark Do not block inside of the callback.
      /// \return A connection pointer that must remain in scope. When the
      /// connection pointer falls out of scope, the connection is broken.
      public: ignition::common::ConnectionPtr ConnectImageCallback(
                  std::function<
                  void(const ignition::msgs::Image &)> _callback);

      /// \brief Set the rendering scene.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(
                  ignition::rendering::ScenePtr _scene) override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual unsigned int ImageWidth() const override;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual unsigned int ImageHeight() const override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual double FarClip() const;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual double NearClip() const;

      /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      /// \brief Callback that is triggered when the scene changes on
      /// the Manager.
      /// \param[in] _scene Pointer to the new scene.
      private: void OnSceneChange(ignition::rendering::ScenePtr /*_scene*/)
              { }

      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<DepthCameraSensorPrivate> dataPtr;
    };
    }
  }
}

#endif