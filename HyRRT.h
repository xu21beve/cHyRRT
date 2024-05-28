// Adapted from OMPL Planner template: https://ompl.kavrakilab.org/newPlanner.html

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_HYRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_HYRRT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <any>
#include <ompl/base/Planner.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/GoalTypes.h>
#include <ompl/control/ODESolver.h>

using namespace std;

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor HyRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           [fill]. DOI: [fill]
        */

        /** \brief Hybrid Rapidly-exploring Random Trees */
        class HyRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            HyRRT(const base::SpaceInformationPtr &si);
            ~HyRRT() override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            enum inputSamplingMethods_ {
                UNIFORM_01,
                UNIFORM_REAL,
                UNIFORM_INT,
                GAUSSIAN_01,
                GAUSSIAN_REAL,
                HALF_NORMAL_REAL,
                HALF_NORMAL_INT,
                QUATERNION,
                EURLER_RPY
            };

            /// \brief Representation of a motion in the search tree
            class Motion
            {
            public:
                /// \brief Default constructor
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()) {}

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Pointer to the root of the tree this motion is
                /// contained in.
                const base::State *root{nullptr};

                /// \brief The integration steps defining the edge of the motion
                std::vector<base::State *> *edge{nullptr};
            };

            /** \brief Get trajectory matrix from trajectoryMatrix_*/
            std::vector<base::State *> getTrajectoryMatrix() {
                return trajectoryMatrix_;
            }

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Set the input range the planner is supposed to use. */
            void setFlowInputRange(std::vector<double> min, std::vector<double> max)
            {
                int size = max.size() > min.size() ? max.size() : min.size();
                if (min.size() != max.size())
                {
                    throw ompl::Exception("Max input value (maxFlowInputValue) and min input value (minFlowInputValue) must be of the same size");
                }
                for (int i = 0; i < size; i++)
                {
                    if (min.at(0) > max.at(0))
                    {
                        throw ompl::Exception("Max input value must be greater than or equal to min input value");
                    }
                }
                minFlowInputValue_ = min;
                maxFlowInputValue_ = max;
            }

             /** \brief Set the input range the planner is supposed to use. */
            void setJumpInputRange(std::vector<double> min, std::vector<double> max)
            {
                int size = max.size() > min.size() ? max.size() : min.size();
                if (min.size() != max.size())
                {
                    throw ompl::Exception("Max input value (maxJumpInputValue_) and min input value (minJumpInputValue) must be of the same size");
                }
                for (int i = 0; i < size; i++)
                {
                    if (min.at(i) > max.at(i))
                    {
                        throw ompl::Exception("Max input value must be greater than or equal to min input value");
                    }
                }
                minJumpInputValue_ = min;
                maxJumpInputValue_ = max;
            }

            /** \brief Set the maximum flow time for a given propagation step. */
            void setTm(double Tm)
            {
                if (Tm <= 0)
                {
                    throw ompl::Exception("Maximum flow time per propagation step must be greater than 0");
                }
                if (!flowStepLength_)
                {
                    if (Tm < flowStepLength_)
                    {
                        throw ompl::Exception("Maximum flow time per propagation step must be greater than or equal to the length of time for each flow integration step (flowStepLength)");
                    }
                }
                Tm_ = Tm;
            }

            /** \brief Set the flow time for a given integration step, within a flow propagation step. */
            void setFlowStepLength(double stepLength)
            {
                if (stepLength <= 0)
                {
                    throw ompl::Exception("Flow step length must be greater than 0");
                }
                if (!Tm_)
                {
                    if (Tm_ < stepLength)
                    {
                        throw ompl::Exception("Flow step length must be less than or equal to the maximum flow time per propagation step (Tm)");
                    }
                }
                flowStepLength_ = stepLength;
            }

            /** \brief Set distance tolerance from goal state. */
            void setGoalTolerance(double tolerance)
            {
                if (tolerance < 0)
                {
                    throw ompl::Exception("Goal tolerance must be greater than or equal to 0");
                }
                tolerance_ = tolerance;
            }

            void setJumpSet(std::function<bool(base::State *)> jumpSet)
            {
                jumpSet_ = jumpSet;
            }

            void setFlowSet(std::function<bool(base::State *)> flowSet)
            {
                flowSet_ = flowSet;
            }

            void setUnsafeSet(std::function<bool(base::State *)> unsafeSet)
            {
                unsafeSet_ = unsafeSet;
            }

            void setDistanceFunction(std::function<double(base::State *, base::State *)> function)
            {
                distanceFunc_ = function;
            }

            void setDiscreteSimulator(std::function<base::State *(base::State *x_cur, std::vector<double> u, base::State *x_new)> function)
            {
                discreteSimulator_ = function;
            }

            void setContinuousSimulator(std::function<base::State *(std::vector<double> inputs, base::State *x_cur, double tFlowMax, base::State *x_new)> function)
            {
                continuousSimulator_ = function;
            }

            void setCollisionChecker(std::function<bool(std::vector<std::vector<double>> *propStepStates, std::function<bool(base::State *state)> obstacleSet, double ts, double tf, base::State *new_state, int tFIndex)> function)
            {
                collisionChecker_ = function;
            }

            /** \brief Set the input sampling mode. See
             * https://github.com/ompl/ompl/blob/main/src/ompl/util/RandomNumbers.h for details on each available mode.
             */
            void setFlowInputSamplingMode(inputSamplingMethods_ mode, std::vector<double> inputs)  // TODO: Add this into input sampling call
            {
                inputSamplingMethod_ = mode;
                    // OMPL_WARN("Input sampling mode not recognized. Defaulting to uniformReal.");
                
                int targetParameterCount = 0;
                
                switch(mode) {
                    case UNIFORM_INT:
                        targetParameterCount = 2;
                        getRandFlowInput_ = [this](int i) {
                            return randomSampler_->uniformInt(minFlowInputValue_[i], maxFlowInputValue_[i]);
                        };
                        break;
                    case GAUSSIAN_REAL:
                        targetParameterCount = 2;
                        getRandFlowInput_ = [&](int i) {
                            return randomSampler_->gaussian(inputs[0], inputs [1]);
                        };
                        break;
                    case HALF_NORMAL_REAL:
                        targetParameterCount = 2;   // Can also be three, if want to specify focus, which defaults to 3.0
                        getRandFlowInput_ = [&](int i) {
                            return randomSampler_->halfNormalReal(inputs[0], inputs[1], inputs[2]);
                        };
                        break;
                    default:
                        targetParameterCount = 2;
                        getRandFlowInput_ = [this](int i) {
                            return randomSampler_->uniformReal(minFlowInputValue_[i], maxFlowInputValue_[i]);
                        };
                        break;
                    // case QUATERNION:
                    //     targetParameterCount = 4;
                    //     getDistributions_ = [&](int i, double) {
                    //         double value[4] = {inputs[0], inputs[1], inputs[2], inputs[3]};
                    //         randomSampler_->quaternion(value);
                    //         return 1.0; //
                    //     };
                    //     break;
                    // case EURLER_RPY:
                    //     targetParameterCount = 3;
                    //     getDistributions_ = [&](int i) {
                    //         double value[3] = {inputs[0], inputs[1], inputs[2]};
                    //         return randomSampler_->eulerRPY(value);
                    //     };
                    //     break;
                }

                if (inputs.size() == targetParameterCount || (mode == HALF_NORMAL_INT && targetParameterCount == 3)) 
                    inputSamplingParameters_ = inputs;
                else {
                    throw ompl::Exception("Invalid number of input parameters for input sampling mode.");
                }

                // sampleFlowInputs_ = [this]() {
                // std::vector<double> u;
                // for (int i = 0; i < maxFlowInputValue_.size(); i++)
                //     {
                //         u.push_back(getRandFlowInput_(i));
                //     }
                //     return u;
                // };
            }

            void setJumpInputSamplingMode(inputSamplingMethods_ mode, std::vector<double> inputs)  // TODO: Add this into input sampling call
            {
                inputSamplingMethod_ = mode;
                    // OMPL_WARN("Input sampling mode not recognized. Defaulting to uniformReal.");
                
                int targetParameterCount = 0;
                
                switch(mode) {
                    case UNIFORM_INT:
                        targetParameterCount = 2;
                        getRandJumpInput_ = [this](int i) {
                            return randomSampler_->uniformInt(minJumpInputValue_[i], maxJumpInputValue_[i]);
                        };
                        break;
                    case GAUSSIAN_REAL:
                        targetParameterCount = 2;
                        getRandJumpInput_ = [&](int i) {
                            return randomSampler_->gaussian(inputs[0], inputs [1]);
                        };
                        break;
                    case HALF_NORMAL_REAL:
                        targetParameterCount = 2;   // Can also be three, if want to specify focus, which defaults to 3.0
                        getRandJumpInput_ = [&](int i) {
                            return randomSampler_->halfNormalReal(inputs[0], inputs[1], inputs[2]);
                        };
                        break;
                    default:
                        targetParameterCount = 2;
                        getRandJumpInput_ = [this](int i) {
                            return randomSampler_->uniformReal(minJumpInputValue_[i], maxJumpInputValue_[i]);
                        };
                        break;
                    // case QUATERNION:
                    //     targetParameterCount = 4;
                    //     getDistributions_ = [&](int i, double) {
                    //         double value[4] = {inputs[0], inputs[1], inputs[2], inputs[3]};
                    //         randomSampler_->quaternion(value);
                    //         return 1.0; //
                    //     };
                    //     break;
                    // case EURLER_RPY:
                    //     targetParameterCount = 3;
                    //     getDistributions_ = [&](int i) {
                    //         double value[3] = {inputs[0], inputs[1], inputs[2]};
                    //         return randomSampler_->eulerRPY(value);
                    //     };
                    //     break;
                }

                if (inputs.size() == targetParameterCount || (mode == HALF_NORMAL_INT && targetParameterCount == 3)) 
                    inputSamplingParameters_ = inputs;
                else {
                    throw ompl::Exception("Invalid number of input parameters for input sampling mode.");
                }

                // sampleJumpInputs_ = [this]() {
                // std::vector<double> u;
                // for (int i = 0; i < maxJumpInputValue_.size(); i++)
                //     {
                //         u.push_back(getRandJumpInput_(i));
                //     }
                //     return u;
                // };
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            std::vector<double> motion_to_vector(base::State *state)
            {
                std::vector<double> motion_vector;
                for (int i = 0; i < 9; i++)
                {
                    motion_vector.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
                }
                return motion_vector;
            }

            void checkAllParametersSet()
            {
                if (!discreteSimulator_)
                {
                    throw ompl::Exception("Jump map not set");
                }
                if (!continuousSimulator_)
                {
                    throw ompl::Exception("Flow map not set");
                }
                if (!flowSet_)
                {
                    throw ompl::Exception("Flow set not set");
                }
                if (!jumpSet_)
                {
                    throw ompl::Exception("Jump set not set");
                }
                if (!unsafeSet_)
                {
                    throw ompl::Exception("Unsafe set not set");
                }
                if (!Tm_)
                {
                    throw ompl::Exception("Max flow propagation time (Tm) no set");
                }
                if (maxJumpInputValue_.size() == 0)
                {
                    throw ompl::Exception("Max input value (maxJumpInputValue) not set");
                }
                if (minJumpInputValue_.size() == 0)
                {
                    throw ompl::Exception("Min input value (minJumpInputValue) not set");
                }
                if (maxFlowInputValue_.size() == 0)
                {
                    throw ompl::Exception("Max input value (maxFlowInputValue) not set");
                }
                if (minFlowInputValue_.size() == 0)
                {
                    throw ompl::Exception("Min input value (minFlowInputValue) not set");
                }
                if (!flowStepLength_)
                {
                    throw ompl::Exception("Flow step length (flowStepLength) not set");
                }
            }

        protected:

            void printMotion(Motion *motion)
            {
                cout << "x1: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << "   v " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << "    a: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << "    tf: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] << "    tj: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] << endl;
            }

            void printState(base::State *motion)
            {
                cout << "x1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << "   x2 " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << "    v1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << "    v2: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] << "    a1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] << "      a2: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] << "      tFlow: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] << "      tJump: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[7] << "      u1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[8] << endl;
            }

            void printTree()
            {
                std::vector<Motion *> traj;
                nn_->list(traj);
                for (Motion *motion : traj)
                {
                    printMotion(motion);
                }
            }

            base::StateSpacePtr setup_;

            void init_tree();

            void random_sample(Motion *randomMotion, std::mt19937 gen);

            // The following are all customizeable parameters, and affects how the cHyRRT generates trajectories

            bool checkPriority(base::State *state);

            std::vector<base::State *> trajectoryMatrix_{nullptr};

            /** \brief Compute distance between states, default is Euclidean distance */
            std::function<double(base::State *state1, base::State *state2)> distanceFunc_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The maximum flow time for a given flow propagation step. Must be set by the user */
            double Tm_;

            /** \brief The distance tolerance from the goal state for a state to be regarded as a valid final state. Default is .1 */
            double tolerance_{.1};

            double minStepLength = 1e-06;

            /** \brief The flow time for a given integration step, within a flow propagation step. Must be set by user */
            double flowStepLength_;

            /** \brief Minimum input value */
            std::vector<double> minFlowInputValue_;

            /** \brief Maximum input value */
            std::vector<double> maxFlowInputValue_;

            /** \brief Minimum input value */
            std::vector<double> minJumpInputValue_;

            /** \brief Maximum input value */
            std::vector<double> maxJumpInputValue_;

            /** \brief Max distance, need to remove */
            double maxDistance_{0.};

            std::function<base::State *(base::State *x_cur, std::vector<double> u, base::State *x_new)> discreteSimulator_;

            /** \brief Function that returns true if a state is in the jump set, and false if not. */
            std::function<bool(base::State *state)> jumpSet_;

            /** \brief Function that returns true if a state is in the flow set, and false if not. */
            std::function<bool(base::State *state)> flowSet_;

            /** \brief Function that returns true if a state is in the flow set, and false if not. */
            std::function<bool(base::State *state)> unsafeSet_;

            std::function<base::State *(std::vector<double> input, base::State *x_cur, double tFlowMax, base::State *x_new)> continuousSimulator_;

            ompl::RNG *randomSampler_ = new ompl::RNG(); // Default constructor always seeds a different value



           /** \brief Function that returns a random input vector. Default is uniform distribution. */
           RNG rng_;
           std::function<std::vector<double> ()> sampleFlowInputs_ = [this]() {
               std::vector<double> u;
               for (int i = 0; i < maxFlowInputValue_.size(); i++)
               {
                   u.push_back(getRandFlowInput_(i));
               }
               return u;
           };

           std::function<std::vector<double> ()> sampleJumpInputs_ = [this]() {
               std::vector<double> u;
               for (int i = 0; i < maxJumpInputValue_.size(); i++)
               {
                   u.push_back(getRandJumpInput_(i));
               }
               return u;
           };



           std::function<double (int i)> getRandFlowInput_ = [this](int i) {
               return randomSampler_->uniformReal(minFlowInputValue_[i], maxFlowInputValue_[i]);
           };

           std::function<double (int i)> getRandJumpInput_ = [this](int i) {
               return randomSampler_->uniformReal(minJumpInputValue_[i], maxJumpInputValue_[i]);
           };


            void push_back_state(std::vector<std::vector<double>> *states_list, base::State *state)
            {
                ompl::base::RealVectorStateSpace::StateType *state_vec = state->as<ompl::base::RealVectorStateSpace::StateType>();
                std::vector<double> new_state;
                for (int i = 0; i < si_->getStateDimension(); i++)
                    new_state.push_back(state_vec->values[i]);
                states_list->push_back(new_state);
            }

            /** \brief Collision checker. Optional is point-by-point collision checking using the jump set. */
            std::function<bool(std::vector<std::vector<double>> *propStepStates, std::function<bool(base::State *state)> obstacleSet, double ts, double tf, base::State *new_state, int tFIndex)> collisionChecker_ = 
                [this](std::vector<std::vector<double>> *propStepStates, std::function<bool(base::State *state)> obstacleSet, double ts, double tf, base::State *new_state, int tFIndex) -> bool {
                    base::State *previous_temp = si_->allocState();
                    base::State *temp = si_->allocState();
                    for (int i = 0; i < propStepStates->size(); i++){
                        for(int j = 0; j < propStepStates->at(i).size(); j++){
                            temp->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = propStepStates->at(i).at(j);
                        }
                        if (obstacleSet(temp)){
                            if (i == 0)
                                si_->copyState(new_state, temp);
                            else 
                                si_->copyState(new_state, previous_temp);
                            return true;
                        }
                        si_->copyState(previous_temp, temp);
                    }
                    return false;
            };

            /** \brief Name of input sampling method, default is "uniform" */

            inputSamplingMethods_ inputSamplingMethod_{UNIFORM_01};

            /** \brief Name of input sampling method, default is "uniform" */
           std::vector<double> inputSamplingParameters_{};

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif

