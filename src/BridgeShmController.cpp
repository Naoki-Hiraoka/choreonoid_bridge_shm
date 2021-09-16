#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/ForceSensor>
#include <vector>
#include <iostream>

#include "../controller/system_shm.c"
#include "../controller/myshm.h"
#include "../controller/servo_shm.h"

using namespace cnoid;

class BridgeShmController : public SimpleController
{
  BodyPtr robot;
  double dt;

  std::vector<double> hardware_pgain;
  std::vector<double> hardware_dgain;

  std::vector<double> qrefprev;
  std::vector<double> qactprev;

  struct servo_shm *s_shm;
  unsigned long long frame_counter=0;

  void initialize_shm()
  {
    std::cerr << "[BridgeShmController] set_shared_memory " << 5555 << std::endl;
    s_shm = (struct servo_shm *)set_shared_memory(5555, sizeof(struct servo_shm));
    if (s_shm == NULL) {
      std::cerr << "[BridgeShmController] set_shared_memory failed" << std::endl;
      exit(1);
    }

    // stateやflagを初期化する. ここではbridge_shmが毎周期書き込まない変数をセット
    s_shm->set_ref_vel = 0;
    s_shm->calib_mode = 0;
    s_shm->disable_alert_on_servo_on = 0;

    for (int i = 0; i < robot->numJoints(); i++) {
      s_shm->ref_angle[i] = robot->joint(i)->q();
      s_shm->ref_vel[i] = 0.0;
      s_shm->ref_torque[i] = 0.0;
      s_shm->pgain[i] = 1.0;
      s_shm->dgain[i] = 1.0;
      s_shm->torque_pgain[i] = 0.0;
      s_shm->torque_dgain[i] = 0.0;
      for (int j = 0; j < (int)(sizeof(s_shm->subgain[0]) / sizeof(s_shm->subgain[0][0])); j++){
        s_shm->subgain[i][j] = 0.0;
      }
      s_shm->motor_num[i] = 1;
      s_shm->controlmode[i] = SERVOMODE_POSITION;
      s_shm->is_servo_on[i] = 1; // 1: servoon start. 0: servooff start.
      s_shm->servo_on[i] = 0;
      s_shm->servo_off[i] = 0;
      s_shm->torque0[i] = 0;
      s_shm->servo_state[0][i] = 0x0000; //0x0000: servoon start. 0x0800: servooff start
      s_shm->loopback[i] = 0; // 0: servoon start. 1: servooff start.
      s_shm->joint_enable[i] = 1;
      s_shm->joint_offset[i] = 0;
    }
  }

  void write_shm() {
    s_shm->frame = (int)frame_counter; frame_counter++;
    s_shm->received_packet_cnt = 0;
    s_shm->jitter = 0.0;

    for (int i = 0; i < robot->numJoints(); i++){
      s_shm->cur_angle[i] = robot->joint(i)->q();
      s_shm->abs_angle[i] = robot->joint(i)->q();
      s_shm->cur_vel[i] = 0.0;
      s_shm->cur_torque[i] = robot->joint(i)->u();
      s_shm->motor_temp[0][i] = 0.0;
      s_shm->motor_outer_temp[0][i] = 0.0;
      s_shm->motor_current[0][i] = 0.0;
      s_shm->motor_output[0][i] = 0.0;
      s_shm->board_vin[0][i] = 0.0;
      s_shm->board_vdd[0][i] = 0.0;
      s_shm->comm_normal[0][i] = 1;
      s_shm->h817_rx_error0[0][i] = 0.0;
      s_shm->h817_rx_error1[0][i] = 0.0;

      s_shm->hole_status[0][i] = 0x01;// &0x07==0, &0x07 == 7, &0x10==0だとOFF扱いらしい

      s_shm->torque_coef_current[i] = 0.0;
      s_shm->torque_coef_inertia[i] = 0.0;
      s_shm->torque_coef_coulombfric[i] = 0.0;
      s_shm->torque_coef_viscousfric[i] = 0.0;
    }
    {
      const DeviceList<RateGyroSensor>& rateGyroSensors = robot->devices();
      for(int i=0; i < rateGyroSensors.size(); i++){
        for (int j=0; j<3; j++) s_shm->body_omega[i][j] = rateGyroSensors[i]->w()[j];
      }
    }
    {
      const DeviceList<AccelerationSensor>& accelerationSensors = robot->devices();
      for(int i=0; i < accelerationSensors.size(); i++){
        for (int j=0; j<3; j++) s_shm->body_acc[i][j] = accelerationSensors[i]->dv()[j];
      }
    }
    {
      const DeviceList<ForceSensor>& forceSensors = robot->devices();
      for(int i=0; i < forceSensors.size(); i++){
        for (int j=0; j<6; j++) s_shm->reaction_force[i][j] = forceSensors[i]->F()[j];
      }
    }
  }

  void read_shm_and_control() {
    for (int i = 0; i < robot->numJoints(); i++){
      Link* joint = robot->joint(i);

      if (s_shm->servo_off[i]){
        //do_power_off
        s_shm->servo_state[0][i] = 0x0800; //0x00だとON. それ以外だとOFF. 0x0800以外のflagがONだとEMERGENCY
        s_shm->is_servo_on[i] = 0;
        s_shm->servo_off[i] = 0;
      }
      else if (s_shm->servo_on[i]) {
        if (s_shm->joint_enable[i]) {
          //do_power_on
          s_shm->servo_state[0][i] = 0x0000; //0x00だとON. それ以外だとOFF. 0x0800以外のflagがONだとEMERGENCY
          s_shm->is_servo_on[i] = 1;
        }
        s_shm->servo_on[i] = 0;
      }

      double qref = (s_shm->loopback[i] == 1) ? s_shm->cur_angle[i] : s_shm->ref_angle[i];
      double qact = joint->q();
      if ( s_shm->is_servo_on[i] == 1 && s_shm->loopback[i] == 0 ) {
        double dqact = (qact - qactprev[i]) / dt;
        double dqref = (qref - qrefprev[i]) / dt;
        float limited_pgain = (s_shm->pgain[i] < 0.0) ? 0.0 : s_shm->pgain[i];
        float limited_dgain = (s_shm->dgain[i] < 0.0) ? 0.0 : s_shm->dgain[i];
        float limited_torque_pgain = (s_shm->torque_pgain[i] < 0.0) ? 0.0 : s_shm->torque_pgain[i];
        float limited_torque_dgain = (s_shm->torque_dgain[i] < 0.0) ? 0.0 : s_shm->torque_dgain[i];
        double u=0;
        switch(s_shm->controlmode[i]){
        case SERVOMODE_POSITION_TORQUE:
        case SERVOMODE_POSITION_FFTORQUE:
          u = (qref - qact) * limited_pgain * hardware_pgain[i] + (dqref - dqact) * limited_dgain * hardware_dgain[i] + s_shm->ref_torque[i] * limited_torque_pgain;
          break;
        case SERVOMODE_POSITION:
          u = (qref - qact) * limited_pgain * hardware_pgain[i] + (dqref - dqact) * limited_dgain * hardware_dgain[i];
          break;
        default:
          break;
        }
        joint->u() = u;
      } else {
        joint->u() = 0;
      }

      qrefprev[i] = qref;
      qactprev[i] = qact;
    }
  }

public:

  virtual bool initialize(SimpleControllerIO* io) override
  {
    robot = io->body();
    dt = io->timeStep();

    for(int i=0; i < robot->numJoints(); ++i){
      Link* joint = robot->joint(i);
      joint->setActuationMode(Link::JOINT_TORQUE);
      io->enableIO(joint);
      qrefprev.push_back(joint->q());
      qactprev.push_back(joint->q());
      hardware_pgain.push_back(5000);
      hardware_dgain.push_back(100);
    }

    this->initialize_shm();
    return true;
  }

  virtual bool control() override
  {
    read_shm_and_control();
    write_shm();
    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(BridgeShmController)
