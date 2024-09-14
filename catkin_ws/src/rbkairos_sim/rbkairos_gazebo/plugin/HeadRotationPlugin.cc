
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>

namespace gazebo
{
  class HeadRotationPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;

      // Trova il link della testa
      this->headLink = this->model->GetLink("Head");
      if (!this->headLink)
      {
        gzerr << "Il link della testa non esiste nel modello!" << std::endl;
        return;
      }

      // Connette l'evento di aggiornamento del mondo alla funzione di rotazione
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HeadRotationPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Ottieni il tempo corrente della simulazione
      common::Time currentTime = this->model->GetWorld()->SimTime();

      // Definisci un'angolazione di rotazione in base al tempo (es. sinusoidale)
      double angle = sin(currentTime.Double()) * 0.5; // rotazione sinusoidale

      // Crea la rotazione lungo l'asse Z (puoi cambiare X, Y o Z a seconda dell'asse)
      ignition::math::Quaterniond rotation(ignition::math::Vector3d(0, 0, angle));

      // Applica la rotazione al link della testa
      this->headLink->SetWorldPose(ignition::math::Pose3d(
        this->headLink->WorldPose().Pos(), rotation));
    }

  private:
    physics::ModelPtr model;
    physics::LinkPtr headLink;
    event::ConnectionPtr updateConnection;
  };

  // Registra il plugin
  GZ_REGISTER_MODEL_PLUGIN(HeadRotationPlugin)
}
