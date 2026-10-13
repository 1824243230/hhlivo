#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>
namespace rig {
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  Mat3 skew(const Vec3& v);
  Mat3 expSO3(const Vec3& v);
  Vec3 logSO3(const Mat3& R);
  struct Pose {
    Mat3 R = Mat3::Identity();
    Vec3 p = Vec3::Zero();
    Vec3 apply(const Vec3& x) const {
      return R * x + p;
    }
    Pose inverse() const {
      return {
        R.transpose(), -R.transpose() * p
      };
    }
    Pose operator*(const Pose& b) const {
      return {
        R * b.R, R * b.p + p
      };
    }
  };
  struct SensorConfig {
    int id = 0;
    std::string topic;
    bool enabled = true;
    Pose extrinsic;
    // T_B_sensor, never T_sensor_B
    double time_offset = 0;
    Mat6 extrinsic_cov = Mat6::Zero();
  };
  struct ImuConfig : SensorConfig {
    double gyro_noise = 0.02, accel_noise = 0.2;
    double gyro_walk = 0.0001, accel_walk = 0.001;
    double gyro_limit = 20, accel_limit = 100;
  };
  struct CameraConfig : SensorConfig {
    int width = 1920, height = 1080;
    double scale = 0.5, fx = 960, fy = 960, cx = 960, cy = 540;
    std::string model = "pinhole";
    std::vector<double> distortion{
      0, 0, 0, 0
    };
  };
  struct LidarConfig : SensorConfig {
    int max_points = 5000;
    double blind = 0.5, max_range = 150, range_noise = 0.03, beam_noise = 0.001;
  };
  struct SensorRegistry {
    std::vector<ImuConfig> imus;
    std::vector<CameraConfig> cameras;
    std::vector<LidarConfig> lidars;
    std::string calibration_mode = "placeholder";
    int threads = 4, max_match_points = 20000, visual_points = 800;
    int camera_min = 50, camera_max = 150, patch_size = 8;
    int iterations = 3, max_voxels = 40000, max_visual_points = 3000;
    double update_period = 0.1, wait_budget = 0.15, sensor_timeout = 0.3;
    double init_duration = 0.2, voxel_size = 1.0, map_radius = 50;
    double photo_variance = 100, photo_gate = 40;
    double rigid_rotation_noise = 0.01, rigid_position_noise = 0.02;
    bool geometry = true, visual = true, uncertainty = true, cross_camera = true;
    static SensorRegistry load(const std::string& path);
    void validate() const;
  };
  struct StateLayout {
    std::vector<int> imu_ids, camera_ids;
    int imu(int id) const;
    int gravity() const {
      return 15 * int(imu_ids.size());
    }
    int exposure(int id) const;
    int size() const {
      return gravity() + 3 + int(camera_ids.size());
    }
  };
  struct ImuState {
    Pose pose;
    Vec3 velocity = Vec3::Zero(), bg = Vec3::Zero(), ba = Vec3::Zero();
  };
  struct JointState {
    StateLayout layout;
    std::map<int, ImuState> imus;
    Vec3 gravity{
      0, 0, -9.81
    };
    std::map<int, double> exposure;
    Eigen::MatrixXd covariance;
    double time = 0;
    bool initialized = false;
    Eigen::VectorXd difference(const JointState& prior) const;
    void inject(const Eigen::VectorXd& delta);
    bool finite() const;
  };
  struct ImuSample {
    int id = 0;
    double time = 0;
    Vec3 accel = Vec3::Zero(), gyro = Vec3::Zero();
  };
  struct NormalEquations {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    size_t rows = 0;
    double error = 0;
    explicit NormalEquations(int n) : A(Eigen::MatrixXd::Zero(n, n)), b(Eigen::VectorXd::Zero(n)) {
    }
    void add(const Eigen::RowVectorXd& H, double residual, double variance, double huber = 3);
    void merge(const NormalEquations& other);
  };
  class JointFilter {
    public:
    explicit JointFilter(const SensorRegistry& config) : config_(config) {
    }
    JointState state;
    int base_id = -1;
    std::map<int, uint64_t> propagated;
    uint64_t rigid_updates = 0, rejected_updates = 0;
    void initialize(double time, const std::map<int, ImuSample>& averages);
    void propagate(double end, const std::map<int, ImuSample>& inputs);
    void constrainRig();
    bool iteration(const JointState& prior, const NormalEquations& equations,
    Eigen::MatrixXd& posterior, Eigen::VectorXd& delta);
    void commit(const Eigen::MatrixXd& posterior, const Eigen::VectorXd& last_delta);
    void removeImu(int id);
    void restoreImu(int id);
    Pose bodyPose() const;
    Vec3 bodyVelocity() const;
    Vec3 bodyAngularVelocity() const;
    Mat6 bodyTwistCovariance() const;
    Eigen::MatrixXd bodyJacobian() const;
    Mat6 bodyCovariance() const;
    NormalEquations rigidEquations() const;
    static Eigen::Matrix<double, 1, 6> planeJacobian(const Pose& pose, const Vec3& point, const Vec3& normal);
    private:
    const SensorRegistry& config_;
    std::map<int, Vec3> last_gyro_;
    const ImuConfig& imuConfig(int id) const;
  };
  struct TrajectoryKnot {
    double time = 0;
    Pose pose;
    Vec3 velocity = Vec3::Zero();
    Mat6 covariance = Mat6::Zero();
  };
  class TrajectoryBuffer {
    public:
    std::deque<TrajectoryKnot> knots;
    uint64_t spline_queries = 0, fallback_queries = 0, rejected_queries = 0;
    void append(const TrajectoryKnot& knot);
    bool query(double time, Pose& pose);
    void correctFrom(double time, const Pose& old_pose, const Pose& new_pose);
    Mat3 pointCovariance(const Vec3& point, double dt, const LidarConfig& lidar) const;
    void clear() {
      knots.clear();
    }
  };
  struct Health {
    uint64_t received = 0, rejected = 0, duplicates = 0, stale = 0, recoveries = 0;
    double last_time = -1, last_arrival = -1;
    int consecutive_good = 0;
    bool isolated = false;
  };
  class SensorHealthMonitor {
    public:
    std::map<std::string, Health> sensors;
    bool observe(const std::string& key, double time, double arrival, bool valid);
    static std::string key(const std::string& kind, int id) {
      return kind + "/" + std::to_string(id);
    }
  };
} // namespace rig
