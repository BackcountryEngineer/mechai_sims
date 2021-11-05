#include <gazebo/common/Plugin.hh>

#include <memory>


namespace mechai_sims {
  class GazeboRosMecanumControlPrivate;
  class GazeboRosMecanumControl : public gazebo::ModelPlugin {
  public:
    /// Constructor
    GazeboRosMecanumControl();

    /// Destructor
    ~GazeboRosMecanumControl();

  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;

  private:
    /// Private data pointer
    std::unique_ptr<GazeboRosMecanumControlPrivate> impl_;
  };
}  // namespace gazebo_plugins