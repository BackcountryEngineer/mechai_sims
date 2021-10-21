#include <gazebo/common/Plugin.hh>

#include <memory>


namespace gazebo_plugin_tutorials {
  class GazeboRosMechanumControlPrivate;
  class GazeboRosMechanumControl : public gazebo::ModelPlugin {
  public:
    /// Constructor
    GazeboRosMechanumControl();

    /// Destructor
    ~GazeboRosMechanumControl();

  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;

  private:
    /// Private data pointer
    std::unique_ptr<GazeboRosMechanumControlPrivate> impl_;
  };
}  // namespace gazebo_plugins