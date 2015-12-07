/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#pragma once

#include <string>
#include <fstream>

#include "blackboard/Adapter.hpp"
#include "types/Odometry.hpp"
#include "types/ActionCommand.hpp"
#include "VisionUpdateBundle.hpp"
#include "robotfilter/RobotFilter.hpp"

/* Forward declarations */
class Localiser;
class RobotFilter;

/* Adapter that allows Localisation to communicate with the Blackboard */
class LocalisationAdapter : Adapter {
   public:
      /* Constructor */
      LocalisationAdapter(Blackboard *bb);
      /* Destructor */
      ~LocalisationAdapter();
      /* One cycle of this thread */
      void tick();

   private:
      /* Filter module instances */
      Localiser *L;
      //commented by:James
      //filter for multi variable Kalman filter
      RobotFilter *robotFilter;
      bool firstCycle;
      int playerNumber;
      //odometry-> measurement for dead reckon
      Odometry prevOdometry;
      int64_t prevTimestamp;
      uint8_t prevGameState;
      
      bool isInPenaltyShootout;
      
      std::ofstream logStream;
      
      //commented by;James
      //WTH are the following methods
      //not even a little bit explanation???
      //let me guess
      //according to the passed in state, can localization function be activated
      bool canLocaliseInState(uint8_t state, std::string skill);

      //Commented by:James
      //if the robot is not picked up by referees, is not in the process of diving, is not dead,
      //and is not in the process of getting up
      //the robot can do observation
      bool canDoObservations(void);
      
      //Commented by:James
      //State transition for the robot from Init, Ready, Set, Playing, Finished, Invalid, and Penalized
      //The following two methods are checking for the transition from Initi to Set, Set to Playing.
      bool haveTransitionedSetToPlay(void);
      bool haveTransitionedIntoSet(void);
      std::vector<AbsCoord> getTeammatePoses(void);
      
      //Commented by:James
      //move forward or turn or towards left? the last condition is weird
      bool amWalking(const ActionCommand::All &commands);
      //turning head to search for ball?
      bool amTurningHead(const ActionCommand::All &commands);
      
      void writeResultToBlackboard(void);
      void handleMySharedDistribution(void);
      void handleIncomingSharedUpdate(void);
      
      // The below methods/variables are used for calibration purposes.
      std::vector<double> distanceObservations;
      std::vector<double> headingObservations;
      int numFramesSinceSeen;
      
      void handleObservationMeasurements(const VisionUpdateBundle &visionUpdateBundle);
      void outputVariances(void);
      
      double getMean(const std::vector<double> &vals, double outliersRatio);
      double getVariance(const std::vector<double> &vals, double mean, double outliersRatio);
};
