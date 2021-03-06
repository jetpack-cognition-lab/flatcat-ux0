#ifndef GMES_JOINT_GROUP_HPP
#define GMES_JOINT_GROUP_HPP

#include <common/save_load.h>

#include <control/spaces.h>
#include <control/sensorspace.h>
#include <control/jointcontrol.h>

#include <learning/gmes.h>
#include <learning/payload.h>

namespace learning {

class FlatcatJointSpace : public sensor_vector
{
public:
    FlatcatJointSpace(const robots::Joint_Model& joint)
    : sensor_vector(7)
    {
        sts_msg("Creating flatcat joint space.");
        sensors.emplace_back("[1] angle sin"  , [&joint](){ return +sin(M_PI*joint.s_ang); });
        sensors.emplace_back("[2] angle cos"  , [&joint](){ return -cos(M_PI*joint.s_ang); });
        sensors.emplace_back("[3] torque"     , [&joint](){ return 4*joint.motor.get();    });
        sensors.emplace_back("[4] velocity"   , [&joint](){ return joint.s_vel; });
        sensors.emplace_back("[5] voltage"    , [&joint](){ return joint.s_vol; });
        sensors.emplace_back("[6] current"    , [&joint](){ return joint.s_cur; });
        sensors.emplace_back("[7] temperature", [&joint](){ return joint.s_tmp; });
    }
};


class GMES_Joint : public common::Save_Load {
    GMES_Joint(const GMES_Joint& other) = delete;
    GMES_Joint& operator=(const GMES_Joint&) = delete; // non copyable

public:
    GMES_Joint(GMES_Joint&& other) = default;
    GMES_Joint& operator=(GMES_Joint&& other) = default;

    GMES_Joint( const robots::Joint_Model& joint
              , const std::size_t number_of_experts
              , const double global_learning_rate
              , const double local_learning_rate
              , const std::size_t experience_size )
    : sensors(joint)
    , payload(number_of_experts)
    , expert(number_of_experts, payload, sensors, local_learning_rate, gmes_constants::random_weight_range, experience_size)
    , gmes(expert, global_learning_rate, true, gmes_constants::number_of_initial_experts, joint.name)
    {
        assert(sensors.size() >= 3);
        sts_msg("Created GMES joint.");
    }

    void execute_cycle(void)
    {
        assert(sensors.size() >= 3);
        sensors.execute_cycle();
        gmes.execute_cycle();
    }

    FlatcatJointSpace            sensors;
    static_vector<Empty_Payload> payload;
    Expert_Vector                expert;
    GMES                         gmes;

    void save(std::string f) { expert.save(f); }
    void load(std::string f) { expert.load(f); }
};


class GMES_Joint_Group : public learning::Learning_Machine_Interface {
public:
    GMES_Joint_Group( const robots::Jointvector_t& joints
                    , const std::size_t number_of_experts
                    , const double global_learning_rate
                    , const double local_learning_rate
                    , const std::size_t experience_size )
    : number_of_gmes_joints(joints.size())
    , group()
    , group_activations()
    {
        assert(number_of_gmes_joints > 0);
        group.reserve(number_of_gmes_joints);

        std::size_t number_of_group_activations = 0;
        for (unsigned int i = 0; i < number_of_gmes_joints; ++i) {
            group.emplace_back(joints[i], number_of_experts, global_learning_rate, local_learning_rate, experience_size);
            number_of_group_activations += group[i].gmes.get_activations().size();
        }

        group_activations.assign(number_of_group_activations, .0);
        sts_msg("Created GMES Group of size: %u", number_of_gmes_joints);
        sts_msg("Activation vector has length: %u", group_activations.size());
    }


    ~GMES_Joint_Group() {}

    void execute_cycle(void)
    {
        learning_progress = 0.0;
        std::size_t n = 0;
        for (unsigned int i = 0; i < number_of_gmes_joints; ++i)
        {
            group[i].execute_cycle();
            const VectorN& activations = group[i].gmes.get_activations();

            // copy all activation to a single large vector
            for (std::size_t k = 0; k < activations.size(); ++k)
                group_activations[n++] = activations[k];

            learning_progress += group[i].gmes.get_learning_progress();

        }
        assert(n == group_activations.size());
    }

    double get_learning_progress(void) const { return learning_progress; }
    void enable_learning(bool /*b*/) { assert(false); /*not implemented*/ };

    const VectorN& get_activations(void) const { return group_activations; }

    void save(std::string f) { for (std::size_t i = 0; i < group.size(); ++i) group[i].save(f+"joint"+std::to_string(i)+"_"); }
    void load(std::string f) { for (std::size_t i = 0; i < group.size(); ++i) group[i].load(f+"joint"+std::to_string(i)+"_"); }

private:
    const unsigned int      number_of_gmes_joints;
    std::vector<GMES_Joint> group;
    VectorN                 group_activations;
    double                  learning_progress = 0.0;

};


class GMES_Layer : public learning::Learning_Machine_Interface {
public:

    sensor_vector                       activation;
    static_vector<State_Payload>        payload;
    Expert_Vector                       experts;
    GMES                                gmes;

    std::string                         prefix = "super_";

    GMES_Layer( std::size_t num_experts
              , const VectorN& inputs
              , const Action_Module_Interface& actions
              , std::size_t num_policies
              , float initial_Q
              , float gmes_learning_rate
              , float local_learning_rate
              , std::size_t experience_size
              )
    : activation(inputs)
    , payload(num_experts, actions, num_policies, initial_Q)
    , experts(num_experts, payload, activation, local_learning_rate, gmes_constants::random_weight_range, experience_size)
    , gmes(experts, gmes_learning_rate, true, gmes_constants::number_of_initial_experts, "Layer")
    {}

    void execute_cycle() {
        activation.execute_cycle();
        gmes      .execute_cycle();
    }

    double get_learning_progress(void) const { return gmes.get_learning_progress(); }
    void enable_learning(bool /*b*/) { assert(false); /*not implemented*/ };

    void save(std::string f) {
        experts.save(f+prefix);
        save_payload(f+prefix);
    }

    void load(std::string f) {
        experts.load(f+prefix);
        load_payload(f+prefix);
    }

private:

    void save_payload(std::string f) {
        auto const cols = payload[0].policies[0].qvalues.size();
        auto const rows = payload.size() * payload[0].policies.size();
        file_io::CSV_File<double> csv(f+"payload.dat", rows, cols);
        unsigned line = 0;
        for (std::size_t i = 0; i < payload.size(); ++i)
            for (std::size_t j = 0; j < payload[i].policies.size(); ++j)
                csv.set_line(line++, payload[i].policies[j].qvalues.get_content());
        csv.write();
        assert(line == rows);
    }

    void load_payload(std::string f) {
        auto const cols = payload[0].policies[0].qvalues.size();
        auto const rows = payload.size() * payload[0].policies.size();
        file_io::CSV_File<double> csv(f+"payload.dat", rows, cols);
        unsigned line = 0;
        csv.read();
        for (std::size_t i = 0; i < payload.size(); ++i)
            for (std::size_t j = 0; j < payload[i].policies.size(); ++j)
                csv.get_line(line++, payload[i].policies[j].qvalues);
        assert(line == rows);
    }
};


} /* namespace learning */

#endif /* GMES_JOINT_GROUP_HPP */
