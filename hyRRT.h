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

using namespace std;

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor hyRRT
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
        class hyRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            hyRRT(const base::SpaceInformationPtr &si);
            ~hyRRT() override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            using State = pair<base::State, vector<vector<double>>>;
            using kinematics = function<void(const vector<double> *state, vector<double> control, State *result)>;

            // // Function to set functions as parameters
            // template<typename ReturnType, typename... Args>
            // void declareParam(const std::string &name, std::function<ReturnType(Args...)> defaultFunc)
            // {
            //     std::function<void(const std::function<ReturnType(Args...)>&)> setter = [this, name](const std::function<ReturnType(Args...)> &value)
            //     {
            //         this->setParam<ReturnType, Args...>(name, value);
            //     };

            //     std::function<std::function<ReturnType(Args...)>()> getter = [this, name, defaultFunc]() -> std::function<ReturnType(Args...)>
            //     {
            //         return this->getParam<ReturnType, Args...>(name, defaultFunc);
            //     };

            //     Planner::declareParam(name, this, setter, getter, defaultFunc);
            // }

            // template<typename ReturnType, typename... Args>
            // void setParam(const std::string &name, const std::function<ReturnType(Args...)> &value)
            // {
            //     std::any_cast<std::function<ReturnType(Args...)>&>(params_[name]) = value;
            // }

            // template<typename ReturnType, typename... Args>
            // std::function<ReturnType(Args...)> getParam(const std::string &name, std::function<ReturnType(Args...)> defaultFunc) const
            // {
            //     auto it = params_.find(name);
            //     if (it != params_.end())
            //     {
            //         return std::any_cast<std::function<ReturnType(Args...)>>(it->second);
            //     }
            //     else
            //     {
            //         return defaultFunc;
            //     }
            // }

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Set the input range the planner is supposed to use. */
            void setInputRange(std::vector<double> min, std::vector<double> max)
            {
                minInputValue_ = min;
                maxInputValue_ = max;
            }

            /** \brief Set the maximum flow time for a given propagation step. */
            void setTm(double Tm)
            {
                Tm_ = Tm;
            }

            /** \brief Set the flow time for a given integration step, within a flow propagation step. */
            void setFlowStepLength(double stepLength)
            {
                flowStepLength_ = stepLength;
            }

            /** \brief Set distance tolerance from goal state. */
            void setGoalTolerance(double tolerance) 
            {
                tolerance_ = tolerance;
            }

            bool setJumpSet(std::function<bool(base::State *)> jumpSet)
            {
                jumpSet_ = jumpSet;
            }

            bool setFlowSet(std::function<bool(base::State *)> flowSet)
            {
                flowSet_ = flowSet;
            }

            void setDistanceFunction(std::function<double(base::State *, base::State *)> function)
            {
                distanceFunc_ = function;
            }

            void setJumpPropagationFunction(std::function<base::State*(base::State *x_cur, double u, base::State *x_new)> function)
            {
                jumpPropagation_ = function;
            }

            void setFlowPropagationFunction(std::function<base::State*(double input_accel, double input, base::State *x_cur, double tFlowMax, base::State *x_new)> function) 
            {
                flowPropagation_ = function;
            }

            /** \brief Set the input sampling mode. See https://github.com/ompl/ompl/blob/main/src/ompl/util/RandomNumbers.h for details on each available mode. */
            void setInputSamplingMode(std::string mode) // TODO: Add this into input sampling call
            { 
               if(mode != "uniform01" && mode != "uniformReal" && mode != "uniformInt" && mode != "gaussian01" && mode != "gaussian" && mode != "halfNormalReal" && mode != "halfNormalInt" && mode!= "halfNormalInt" && mode != "quarternion" && mode != "eulerRPY") 
                {
                    inputSamplingMethod_ = "uniformReal";
                    OMPL_WARN("Input sampling mode not recognized. Defaulting to uniformReal.");
                }
                else
                {
                    inputSamplingMethod_ = mode; 
                }
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


            // void integrateStates(const ompl::control::ODESolver::ODE &kinematics, vector<double> state, vector<double> control, vector<double> *result, double t);

            // void defineModel(std::function<base::State*(base::State *, double, base::State *)> jumpFunc,
            //                                             std::function<base::State*(double, double, base::State *, double, base::State *)> flowFunc,
            //                                             std::function<double(base::State *, base::State *)> distanceFunc);


            // const ompl::control::ODESolver::ODE& ODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control* u, ompl::control::ODESolver::StateType& qdot);

            std::vector<double> motion_to_vector(base::State *state){
                std::vector<double> motion_vector;
                for(int i = 0; i < 9; i++) {
                    motion_vector.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
                }
                return motion_vector;
            }
            
        protected:
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
            };

            bool Xu(double x1, double x2)
            {
                if (x1 <= 4.5 && x1 >= 0 && x2 >= 1.0 && x2 <= 1.5)
                {
                    return true;
                }
                if (x1 <= 0.5 && x1 >= 0 && x2 >= 1.0 && x2 <= 3.0)
                {
                    return true;
                }
                if (x1 <= 4.5 && x1 >= 0 && x2 >= 2.5 && x2 <= 3.0)
                {
                    return true;
                }
                return false;
            }

            void printMotion(Motion *motion)
            {
                cout << "x1: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << "   x2 " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << "    v1: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << "    v2: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] << "    a1: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] << "      a2: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] << "      tFlow: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] << "      tJump: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[7] << "      u: " << motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[8] << endl;
            }

            void printState(base::State *motion)
            {
                cout << "x1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << "   x2 " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << "    v1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << "    v2: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] << "    a1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] << "      a2: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] << "      tFlow: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] << "      tJump: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[7] << "      u1: " << motion->as<ompl::base::RealVectorStateSpace::StateType>()->values[8] << endl;
            }

            void printTree(){
                std::vector<Motion *> traj;
                nn_->list(traj);
                for(Motion *motion : traj){
                    printMotion(motion);
                }
            }

            base::StateSpacePtr setup_;

            // /// \brief Pointer to the root of the tree this motion is
            // /// contained in.
            // std::map<std::string, std::any> params_;  // Parameter map

            const base::State *root{nullptr};

            void init_tree();

            void random_sample(Motion *randomMotion, std::mt19937 gen);

            // The following are all customizeable parameters, and affects how the cHyRRT generates trajectories

            bool checkPriority(base::State *state);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                double x_a = a->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double x_b = b->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                // cout << "Distance to goal: " << fabs(x_a - x_b) << endl;
                return fabs(x_a - x_b);  // Set to default Pythagorean distance on Euclidean plane
            }

            void checkAllParametersSet(){
                if(!jumpPropagation_){
                    throw ompl::Exception("Jump map not set");
                }
                if(!flowPropagation_){
                    throw ompl::Exception("Flow map not set");
                }
                if(!flowSet_){
                    throw ompl::Exception("Flow set not set");
                }
                if(!jumpSet_){
                    throw ompl::Exception("Jump set not set");
                }
                if(!Tm_){
                    throw ompl::Exception("Max flow propagation time (Tm) no set");
                }
                if(maxInputValue_.size() == 0){
                    throw ompl::Exception("Max input value (maxInputValue) not set");
                }
                if(minInputValue_.size() == 0){
                    throw ompl::Exception("Min input value (minInputValue) not set");
                }
                if(maxInputValue_.size()!= minInputValue_.size()){
                    throw ompl::Exception("Max input value (maxInputValue) and min input value (minInputValue) must be of the same size");
                }
                if(!flowStepLength_){
                    throw ompl::Exception("Flow step length (flowStepLength) not set");
                }
            }

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
            std::vector<double> minInputValue_;


            /** \brief Maximum input value */
            std::vector<double> maxInputValue_;

            std::function<base::State*(base::State *x_cur, double u, base::State *x_new)> jumpPropagation_;

            /** \brief Function that returns true if a state is in the jump set, and false if not. */
            std::function<bool(base::State *state)> jumpSet_;
            
            /** \brief Function that returns true if a state is in the flow set, and false if not. */
            std::function<bool(base::State *state)> flowSet_;

            std::function<base::State*(double input_accel, double input, base::State *x_cur, double tFlowMax, base::State *x_new)> flowPropagation_;

            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief Name of input sampling method, default is "uniform" */
            std::string inputSamplingMethod_{"uniform"};
        };
    }
}

#endif