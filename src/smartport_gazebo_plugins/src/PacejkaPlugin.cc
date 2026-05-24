#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

namespace smartport_gazebo_plugins
{
  struct WheelParams {
    std::string name;
    gz::sim::Entity entity;
    bool is_steerable;
    
    // Base Pacejka & Suspension
    double ks, cs, mu, B, C;
    double last_dz = 0.0;

    // --- NEW: Combined Slip & Transient State ---
    double B_xa = 1.0, C_xa = 1.0; // Friction ellipse coefficients for Fx
    double B_yk = 1.0, C_yk = 1.0; // Friction ellipse coefficients for Fy
    double sigma = 0.3;            // Relaxation length (meters)
    
    // Memory of the actual forces from the previous millisecond
    double Fx_actual = 0.0;
    double Fy_actual = 0.0;
  };

  class PacejkaPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private:
      gz::sim::Entity model_entity;
      std::vector<WheelParams> wheels;
      
      // Global parameters
      double vehicle_mass = 4750.0;
      double unsprung_mass = 85.0;
      double wheel_radius = 0.5;
      double wheel_width = 0.3;
      double track_width = 2.4;
      double cg_height = 0.98;

      std::chrono::steady_clock::duration last_update_time{0};

    public:
      void Configure(const gz::sim::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     gz::sim::EntityComponentManager &_ecm,
                     gz::sim::EventManager &/*_eventMgr*/) override
      {
        this->model_entity = _entity;
        gz::sim::Model model(this->model_entity);

        if (!model.Valid(_ecm)) {
          gzerr << "PacejkaPlugin should be attached to a model entity." << std::endl;
          return;
        }

        // Parse global parameters
        if (_sdf->HasElement("vehicle_mass"))
          this->vehicle_mass = _sdf->Get<double>("vehicle_mass");
        if (_sdf->HasElement("unsprung_mass"))
          this->unsprung_mass = _sdf->Get<double>("unsprung_mass");
        if (_sdf->HasElement("wheel_radius"))
          this->wheel_radius = _sdf->Get<double>("wheel_radius");
        if (_sdf->HasElement("wheel_width"))
          this->wheel_width = _sdf->Get<double>("wheel_width");
        if (_sdf->HasElement("track_width"))
          this->track_width = _sdf->Get<double>("track_width");
        if (_sdf->HasElement("cg_height"))
          this->cg_height = _sdf->Get<double>("cg_height");

        // Parse wheels
        auto wheel_elem = _sdf->FindElement("wheel");
        while (wheel_elem)
        {
          WheelParams w;
          w.name = wheel_elem->Get<std::string>("name");
          w.is_steerable = wheel_elem->Get<bool>("is_steerable", false).first;
          w.ks = wheel_elem->Get<double>("ks", 75000.0).first;
          w.cs = wheel_elem->Get<double>("cs", 5000.0).first;
          w.mu = wheel_elem->Get<double>("mu", 0.85).first;
          w.B = wheel_elem->Get<double>("pacejka_B", 10.0).first;
          w.C = wheel_elem->Get<double>("pacejka_C", 1.65).first;

          w.entity = model.LinkByName(_ecm, w.name);
          if (w.entity != gz::sim::kNullEntity) {
            // Enable components
            _ecm.CreateComponent(w.entity, gz::sim::components::WorldPose());
            _ecm.CreateComponent(w.entity, gz::sim::components::WorldLinearVelocity());
            _ecm.CreateComponent(w.entity, gz::sim::components::WorldAngularVelocity());
            
            // We use ExternalWorldWrenchCmd to apply the forces
            if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(w.entity)) {
              _ecm.CreateComponent(w.entity, gz::sim::components::ExternalWorldWrenchCmd());
            }
            this->wheels.push_back(w);
            gzmsg << "PacejkaPlugin initialized wheel: " << w.name << std::endl;
          } else {
            gzerr << "Link not found: " << w.name << std::endl;
          }

          wheel_elem = wheel_elem->GetNextElement("wheel");
        }
        gzmsg << "PacejkaPlugin configured with " << this->wheels.size() << " wheels." << std::endl;
      }

      void PreUpdate(const gz::sim::UpdateInfo &_info,
                     gz::sim::EntityComponentManager &_ecm) override
      {
        if (_info.paused) return;

        double dt = std::chrono::duration<double>(_info.dt).count();
        if (dt <= 0.0) return;

        for (auto &w : this->wheels)
        {
          auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(w.entity);
          auto linVelComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(w.entity);
          auto angVelComp = _ecm.Component<gz::sim::components::WorldAngularVelocity>(w.entity);

          if (!poseComp || !linVelComp || !angVelComp) continue;

          gz::math::Pose3d pose = poseComp->Data();
          gz::math::Vector3d world_lin_vel = linVelComp->Data();
          gz::math::Vector3d world_ang_vel = angVelComp->Data();

          // 1. Suspension (Fz)
          // Simplified flat-ground assumption: distance to ground is just the Z coordinate
          double min_dist = pose.Pos().Z(); 
          double Fz = 0.0;

          if (min_dist < this->wheel_radius) {
            double dz = this->wheel_radius - min_dist;
            double dz_dot = (dz - w.last_dz) / dt;
            w.last_dz = dz;

            double F_susp = (w.ks * dz) + (w.cs * dz_dot);
            Fz = std::max(0.0, F_susp + (this->unsprung_mass * 9.81));
          } else {
            w.last_dz = 0.0;
          }

          if (Fz <= 0.0) continue;

          // Transform velocities to local wheel frame
          gz::math::Vector3d local_lin_vel = pose.Rot().Inverse() * world_lin_vel;
          gz::math::Vector3d local_ang_vel = pose.Rot().Inverse() * world_ang_vel;

          // 2. Slip Calculation
          double Vx = local_lin_vel.X();
          double Vy = local_lin_vel.Y();
          double omega = local_ang_vel.Y(); // Rolling is around local Y

          double Vx_safe = (std::abs(Vx) < 0.05) ? 0.05 : Vx;
          double kappa = ((this->wheel_radius * omega) - Vx) / std::abs(Vx_safe);
          double alpha = std::atan2(Vy, std::abs(Vx_safe));

          // 3. PURE PACEJKA FORMULA (Step 1)
          double Dx = w.mu * Fz;
          double Dy = w.mu * Fz;

          double Fx0 = Dx * std::sin(w.C * std::atan(w.B * kappa));
          double Fy0 = Dy * std::sin(w.C * std::atan(w.B * alpha));

          // 4. COMBINED SLIP FRICTION ELLIPSE (Step 2)
          // Degrading the pure forces based on simultaneous braking/cornering
          double Fx_target = Fx0 * std::cos(w.C_xa * std::atan(w.B_xa * alpha));
          double Fy_target = Fy0 * std::cos(w.C_yk * std::atan(w.B_yk * kappa));

          // 5. TRANSIENT TIRE MODEL / RELAXATION LENGTH (Step 3)
          // Prevent division by zero if someone sets sigma to 0 in the SDF
          double sigma_safe = std::max(0.01, w.sigma); 
          
          // Calculate the rate of change of the force
          double Fx_dot = (std::abs(Vx_safe) / sigma_safe) * (Fx_target - w.Fx_actual);
          double Fy_dot = (std::abs(Vx_safe) / sigma_safe) * (Fy_target - w.Fy_actual);

          // 6. EULER INTEGRATION (Step 4)
          // Accumulate the force over the timestep
          w.Fx_actual += (Fx_dot * dt);
          w.Fy_actual += (Fy_dot * dt);

          // 7. FORCE APPLICATION
          // Use the integrated actual forces, not the instantaneous target forces
          gz::math::Vector3d local_force(w.Fx_actual, -w.Fy_actual, Fz);
          gz::math::Vector3d local_contact_pt(0, 0, -this->wheel_radius);
          gz::math::Vector3d local_torque = local_contact_pt.Cross(local_force);

          // Convert to world frame
          gz::math::Vector3d global_force = pose.Rot() * local_force;
          gz::math::Vector3d global_torque = pose.Rot() * local_torque;

          // Apply Wrench
          gz::msgs::Wrench wrench_msg;
          gz::msgs::Set(wrench_msg.mutable_force(), global_force);
          gz::msgs::Set(wrench_msg.mutable_torque(), global_torque);

          auto wrenchComp = _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(w.entity);
          if (wrenchComp) {
            gz::msgs::Wrench &existing_wrench = wrenchComp->Data();
            existing_wrench.mutable_force()->set_x(existing_wrench.force().x() + global_force.X());
            existing_wrench.mutable_force()->set_y(existing_wrench.force().y() + global_force.Y());
            existing_wrench.mutable_force()->set_z(existing_wrench.force().z() + global_force.Z());
            existing_wrench.mutable_torque()->set_x(existing_wrench.torque().x() + global_torque.X());
            existing_wrench.mutable_torque()->set_y(existing_wrench.torque().y() + global_torque.Y());
            existing_wrench.mutable_torque()->set_z(existing_wrench.torque().z() + global_torque.Z());
          } else {
            _ecm.CreateComponent(w.entity, gz::sim::components::ExternalWorldWrenchCmd(wrench_msg));
          }
        }
      }
  };
}

GZ_ADD_PLUGIN(smartport_gazebo_plugins::PacejkaPlugin,
              gz::sim::System,
              smartport_gazebo_plugins::PacejkaPlugin::ISystemConfigure,
              smartport_gazebo_plugins::PacejkaPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(smartport_gazebo_plugins::PacejkaPlugin, "pacejka_plugin")
