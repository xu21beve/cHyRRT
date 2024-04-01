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

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
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

            // bool solutionChecker(bool isExtended, vector<vector<double>> state, vector<vector<double>> goalState);

            std::function<base::State*(double input_accel, double input, base::State *x_cur, double tFlowMax, base::State *x_new)> flowPropagation;

            void defineModel(std::function<base::State*(base::State *, double, base::State *)> jumpFunc,
                                                        std::function<base::State*(double, double, base::State *, double, base::State *)> flowFunc,
                                                        std::function<double(base::State *, base::State *)> distanceFunc);


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
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Cost of the state
                base::Cost cost;

                /// \brief Pointer to the root of the tree this motion is
                /// contained in.
                const base::State *root{nullptr};

                bool valid{false};

                std::vector<Motion *> children;
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
            base::State *state_{nullptr};
            Motion *parent{nullptr};
            /// \brief Pointer to the root of the tree this motion is
            /// contained in.
            const base::State *root{nullptr};

            std::function<base::State*(base::State *x_cur, double u, base::State *x_new)> jumpPropagation;

            std::function<bool(base::State *state)> jumpSet;
            
            std::function<bool(base::State *state)> flowSet;

            void init_tree();

            void random_sample(Motion *randomMotion, std::mt19937 gen);

            bool checkPriority(base::State *state);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                double x_a = a->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double x_b = b->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                // cout << "Distance to goal: " << fabs(x_a - x_b) << endl;
                return fabs(x_a - x_b);  // Set to default Pythagorean distance on Euclidean plane
            }

            std::function<double(base::State *state1, base::State *state2)> distanceFunc;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            double maxSteps = 2000;
            double maxJumps = 80;
            double maxFlows = 100;
            double Tm = 0.5;
            double minStepLength = 1e-06;
            double flowStepLength = 0.01;
        };
    }
}

#endif