class_<BehaviourRequest>("BehaviourRequest")
   .def_readwrite("actions", &BehaviourRequest::actions)
   .def_readwrite("goalieAttacking", &BehaviourRequest::goalieAttacking)
   .def_readwrite("doingBallLineUp", &BehaviourRequest::doingBallLineUp)
   .def_readwrite("isInReadyMode", &BehaviourRequest::isInReadyMode)
   .def_readwrite("timeToReachBall", &BehaviourRequest::timeToReachBall)
   .def_readwrite("timeToReachDefender", &BehaviourRequest::timeToReachDefender)
   .def_readwrite("timeToReachMidfielder", &BehaviourRequest::timeToReachMidfielder)
   .def_readwrite("timeToReachUpfielder", &BehaviourRequest::timeToReachUpfielder)
   .def_readwrite("currentRole", &BehaviourRequest::currentRole)
   .def_readwrite("walkingToX", &BehaviourRequest::walkingToX)
   .def_readwrite("walkingToY", &BehaviourRequest::walkingToY)
   .def_readwrite("shootingToX", &BehaviourRequest::shootingToX)
   .def_readwrite("shootingToY", &BehaviourRequest::shootingToY)
   .def_readwrite("readyPositionAllocation0", &BehaviourRequest::readyPositionAllocation0)
   .def_readwrite("readyPositionAllocation1", &BehaviourRequest::readyPositionAllocation1)
   .def_readwrite("readyPositionAllocation2", &BehaviourRequest::readyPositionAllocation2)
   .def_readwrite("readyPositionAllocation3", &BehaviourRequest::readyPositionAllocation3)
   .def_readwrite("readyPositionAllocation4", &BehaviourRequest::readyPositionAllocation4);
