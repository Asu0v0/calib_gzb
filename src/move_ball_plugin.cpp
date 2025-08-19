#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <random>

namespace gazebo {
class MoveBallPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;

    if (_sdf->HasElement("speed"))
      this->speed = _sdf->Get<double>("speed"); // 速度
    if (_sdf->HasElement("z_fixed"))
      this->z = _sdf->Get<double>("z_fixed");

    this->GenerateRandomPath();
    this->updateConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MoveBallPlugin::OnUpdate, this));
  }

  void GenerateRandomPath() {
    std::mt19937 gen(42); // 固定种子确保轨迹可复现
    std::uniform_real_distribution<> dist_x(6.0, 12.0);
    std::uniform_real_distribution<> dist_y(-5.0, 5.0);

    for (int i = 0; i < 15; ++i) {
      waypoints.emplace_back(dist_x(gen), dist_y(gen), z);
    }
    currentIdx = 0;
  }

  void OnUpdate() {
    if (waypoints.empty())
      return;

    ignition::math::Vector3d pos = model->WorldPose().Pos();
    ignition::math::Vector3d target = waypoints[currentIdx];
    ignition::math::Vector3d diff = target - pos;

    // 到达目标点就切换下一个
    if (diff.Length() < 0.2) {
      currentIdx = (currentIdx + 1) % waypoints.size();
      return;
    }

    ignition::math::Vector3d dir = diff.Normalized();
    model->SetLinearVel(dir * speed);
  }

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConn;
  std::vector<ignition::math::Vector3d> waypoints;
  int currentIdx = 0;
  double speed = 1.5;
  double z = 0.5;
};

GZ_REGISTER_MODEL_PLUGIN(MoveBallPlugin)
} // namespace gazebo
