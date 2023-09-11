#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <conio.h>
#include <future>
#include <iostream>
#include <tuple>
#include <functional>
#include <optional>
#include <array>
#include <algorithm>

#include "AimsunWSClientHandler.h"
#include "WebSocketClient.h"

#include "lib/jsoncpp-amalgamated-2016-09-05/json/json.h"

#include <stdio.h>
// Here you can include any additional headers or add useful functions

class optimizationSettings {
public:

	const std::string endpointUrl = "ws://localhost:3333";

	//process settings
	const bool preprocessing = false; //true if recording vehicles free flow; false if preprocessed vehicles are available

	//Rolling horizon settings
	const int controlIntLngth = 6; // smallest unity control can switch phases //Note: cannot be smaller than losstime //check usage in combi with semaphore!
	const int nrofDecisionInt = 2; // decision horizon; after these nrof intervals a new control decision is made
	const int nrofPredictionInt = 10; //prediction horizon; the nrof intervals that is looked ahead in making the decision
	const int nrofSimInt = 450; //total number of intervals in entire simulation period 
	int controlIntIdx = 0;// current control interval system is in; =0 complete simulation from start; >0: warm start (not supported)
	
	//Prediction model settings
	const int lossTime = 3; //losstime <= controlIntLngth; only whole seconds... //check usage in combi with semaphore!
	const bool blockSignals = false; //if true signals switch to red if prediction indicates stream is blocked by downstream traffic; 
	const bool headtail = true; //Prediction model: if true modeling of available space by head and queue tail 
	const bool individualqueue = false; //Prediction model: if true modeling of individual queued vehicles and their destinations; 
	const bool individualdemand = true; //Prediction model: if true modeling of individual arrivals at network entrances 
	const bool headtailSTAT = true; //Simulation model (idem); should be true if value for prediction model is true; (should be true for sim) 
	const bool individualqueueSTAT = false; //Simulation model (idem); should be true if value for prediction model is true; (should be true for sim, doesnot work with RANDOM vehicles!) 
	const bool individualdemandSTAT = true; //Simulation model (idem); should be true if value for prediction model is true; (should be true for sim) 

	//optimization settings
	const bool objdelay = true; //true: objective is delay; false: objective is TTS; 
	const bool suboptimal = false; //true: if solution is not ready in nr of decision time intervals, then a suboptimal solution is used; false: process waits till solution is ready;
	const int activateMethod = 1; //method 1 = BB; method 2 = GA; method 3 = ... [add new methods] 	
	const bool useTotalState = false; //(BB only) true: state is compared without distinguishing active phases; false: state is compared distinguishing active phases
	const int boundpercentage = 6; //(BB only) relative boundcriteria a [additional delay/TTS percentage] in a*min+b (999999 to disable);
	const int boundabsolute = 60; //(BB only) absolute boundcriteria b [additional delay/TTS value] in a*min+b (999999 to disable);
	const int nrofLevelSearchNodes = 5000; //(BB only) max number of nodes in search list (per level in tree);

	//Robust settings 
	//Note: for sensitivity analysis with disturbed average (time independent) parameters, disable robust settings and do not use scenarios, just specify the true and disturbed parameter value in signals.txt and splitfractions.txt (see ReadSignals and readSplitFractions for format)
	//(to disable robust settings: set objrobust=1; nrofScenarios=0; scenarioWeight = {0.0}; random...=false; nrofScenarioPeriods = 0; scenarioPeriods = {{{0,0},{450,450}}, robustHeuristic=false; nrofScenariosSubset = 0; scenarioActive ={false}; scenarioLog = 0;)
	const int objrobust = 3; //robust method; 1=non-robust (based on the average model parameters); 2=single scenario (based on a single parameter scenario specified by scenarioLog); 3=robust minmax (over all uncertainty scenarios, for subset if robustHeuristic = true); 4=expectedmean (over all scenarios averaged by scenarioWeight); 5=average (over all scenarios averaged without weights)
	const int nrofScenarios = 3;// nrof uncertainty scenarios 
	std::array<double, 11> scenarioWeight = { 0.0,0.2,0.7,0.1 }; // (if objrobust == 4 (expectedmean)) probability of scenarios; per scenario idx (0-nrofscenarios) a probability; sum==1;
	const bool randomSaturationRates = false; //true: saturation rates are uncertain based on scenarios; false:average (disturbed) value is used
	const bool randomTravelTimes = false; //true: travel times are uncertain based on scenarios; false:average (disturbed) value is used
	const bool randomState = false; //true: initial states are uncertain based on scenarios; false:average (disturbed) value is used
	const bool randomDemand = false; //true: demand arrivals are uncertain based on scenarios; false:average (disturbed) value is used
	const bool randomFractions = true; //true: turn fractions are uncertain based on scenarios; false:average (disturbed) value is used
	const int nrofScenarioPeriods = 3; //nrof periods in simulation period [0,450] that uncertainty scenarios are active (0 if active for entire period)
	std::array<std::array<int, 2>, 5> scenarioPeriods = { {{0,0},{60,70},{110,120},{160,170},{450,450}} }; //per active period the start interval and end interval	
	const bool robustHeuristic = false; //(if objrobust == 3 (minmax)) true to use heuristic with subset of scenarios; false to use full set; only possible for minmax (hardcoded)
	const int nrofScenariosSubset = 3; //(if objrobust == 3 (minmax)) size of the scenario subset (<= nrofScenarios) in robust heuristic; 
	std::array<bool,11> scenarioActive = {false,true,true,true }; //(if objrobust == 3 (minmax)) initialization active scenarios; per scenario idx (0-nrofscenarios) a bool true if active false otherwise
	std::vector<int> scenarioContainer; //(if objrobust == 3 (minmax)) active scenarios over time;
	const int scenarioLog = 3; //leading scenario number for state statistics (logging), 0 for average parameter statistics for non-robust control (objrobust==1) (and advised for objrobust==4,5); >0 for scenario statistics for one of the scenarios specified (advised for objrobust==2,3)

	//Robust EXAMPLE settings (for simulated traffic case in NetworkCorridor_robust.ang): UNCERTAIN TURN FRACTION SCENARIOS (TEMPORARILY DURING A PREDEFINED TIME PERIOD): 
	/*Note: for now the turn fraction scenarios for the prediction model are not randomly drawn but read from splitfractions.txt (see ReadSplitFractions) for format
	Note: in the example a set of 3 turn fraction uncertainty scenarios is used for the prediction model, around the average(mean) turn fraction value,  with scenario 1 an expected BEST case, scenario 2 a close to average case, scenario 3 an expected WORST case (note in reality more scenarios can be distinguished) 
	Note: the simulated traffic case in the example (NetworkCorridor_robust.ang) contains realized fractions between an average and worst-case situation, and performs (27.5%) better using robust MINMAX control instead of non-robust control based on average turn fractions.  
	*/
	//Robust EXAMPLE settings (for NetworkCorridor_robust.ang): NON-ROBUST CONTROL REFERENCE: Optimizing the control decision based on the average turn fraction values during the uncertainty period;
	/*const int objrobust = 1; // 1=non-robust (based on the average model parameters)
	const int nrofScenarios = 3;// nrof uncertainty scenarios 
	std::array<double, 11> scenarioWeight = { 0.0,0.2,0.7,0.1 }; // probability of scenarios; per scenario idx (0-nrofscenarios) a probability; sum==1;
	const bool randomFractions = true; //true: turn fractions are uncertain based on scenarios; 
	const int nrofScenarioPeriods = 3; //nrof periods in simulation period [0,450] that uncertainty scenarios are active (0 if active for entire period)
	std::array<std::array<int, 2>, 5> scenarioPeriods = { {{0,0},{60,70},{110,120},{160,170},{450,450}} }; //per active period the start interval and end interval		
	const int scenarioLog = 0; //leading scenario number for state statistics (logging), 0 for average parameter statistics for non-robust control (objrobust==1)
	*/
	//Robust EXAMPLE settings (for NetworkCorridor_robust.ang): ROBUST MINMAX CONTROL: Optimizing the control decision based on MINMAX over all turn fraction scenarios during the uncertainty period; 
	/*const int objrobust = 3; // 3=robust minmax (over all uncertainty scenarios);
	const int nrofScenarios = 3;// nrof uncertainty scenarios 
	std::array<double, 11> scenarioWeight = { 0.0,0.2,0.7,0.1 }; // probability of scenarios; per scenario idx (0-nrofscenarios) a probability; sum==1;
	const bool randomFractions = true; //true: turn fractions are uncertain based on scenarios; 
	const int nrofScenarioPeriods = 3; //nrof periods in simulation period [0,450] that uncertainty scenarios are active (0 if active for entire period)
	std::array<std::array<int, 2>, 5> scenarioPeriods = { {{0,0},{60,70},{110,120},{160,170},{450,450}} }; //per active period the start interval and end interval		
	const bool robustHeuristic = false; //full MINMAX approach over all 3 scenarios (heuristic not necessary for 3 scenarios)
	const int scenarioLog = 3; //leading scenario number for state statistics (logging only) 
	*/
};

class signalGroup
{
public:
	signalGroup(int oid, std::string nam, std::string jncnam) :
		oid_{ oid }, name_{ nam }, jncname_{ jncnam } {}

	int oid() { return oid_; }
	std::string name() { return name_; }
	std::string jncname() { return jncname_; }
	int getSgnr() { return sgnr_; }
	void setSgnr(int nr) { sgnr_ = nr; }
	int getInternalSgnr() { return internalsgnr_; }
	void setInternalSgnr(int nr) { internalsgnr_ = nr; }
	int getMinGreen() { return minGRN_; }
	void setMinGreen(int maxGRN) { minGRN_ = maxGRN; }
	int getMaxGreen() { return maxGRN_; }
	void setMaxGreen(int maxGRN) { maxGRN_ = maxGRN; }
	int getNrofLanes() { return nrofLanes_; }
	void setNrofLanes(int lanes) { nrofLanes_ = lanes; }

	bool getInternalArr() { return internalArr_; }
	void setInternalArr(bool flg) { internalArr_ = flg; }
	int getExternalBoundary() { return  externalBoundary_; }
	void setExternalBoundary(int nr) { externalBoundary_ = nr; }

	double getQueuerate() { return queuerate_; }
	void setQueuerate(double qrate) { queuerate_ = qrate; }
	double getSatrate() { return satrate_; }
	void setSatrate(double rate) { satrate_ = rate; }
	double getDistance() { return distance_; }
	void setDistance(double dist) { distance_ = dist; }
	double getVehicleDistance() { return vehicleDistance_; }
	void setVehicleDistance(double dist) { vehicleDistance_ = dist; }
	double getSpeed() { return speed_; }
	void setSpeed(double spd) { speed_ = spd; }
	int getNrofTravIntervals() { return nrofTravIntervals_; }
	void setNrofTravIntervals(int nrof) { nrofTravIntervals_ = nrof; }

	double getDisturbedQueuerate() { return disturbedQueuerate_; }
	void setDisturbedQueuerate(double qrate) { disturbedQueuerate_ = qrate; }
	double getDisturbedSatrate() { return disturbedSatrate_; }
	void setDisturbedSatrate(double rate) { disturbedSatrate_ = rate; }
	double getDisturbedDistance() { return disturbedDistance_; }
	void setDisturbedDistance(double dist) { disturbedDistance_ = dist; }
	double getDisturbedVehicleDistance() { return disturbedVehicleDistance_; }
	void setDisturbedVehicleDistance(double dist) { disturbedVehicleDistance_ = dist; }
	double getDisturbedSpeed() { return disturbedSpeed_; }
	void setDisturbedSpeed(double spd) { disturbedSpeed_ = spd; }
	int getDisturbedNrofTravIntervals() { return disturbedNrofTravIntervals_; }
	void setDisturbedNrofTravIntervals(int nrof) { disturbedNrofTravIntervals_ = nrof; }
	
	double getMeanQueuerate() { return meanQueuerate_; }
	void setMeanQueuerate(double qrate) { meanQueuerate_ = qrate; }
	double getMeanSatrate() { return meanSatrate_; }
	void setMeanSatrate(double rate) { meanSatrate_ = rate; }
	double getMeanDistance() { return meanDistance_; }
	void setMeanDistance(double dist) { meanDistance_ = dist; }
	double getMeanVehicleDistance() { return meanVehicleDistance_; }
	void setMeanVehicleDistance(double dist) { meanVehicleDistance_ = dist; }
	double getMeanSpeed() { return meanSpeed_; }
	void setMeanSpeed(double spd) { meanSpeed_ = spd; }
	int getMeanNrofTravIntervals() { return meanNrofTravIntervals_; }
	void setMeanNrofTravIntervals(int nrof) { meanNrofTravIntervals_ = nrof; }

	double getStddevQueuerate() { return stddevQueuerate_; }
	void setStddevQueuerate(double qrate) { stddevQueuerate_ = qrate; }
	double getStddevSatrate() { return stddevSatrate_; }
	void setStddevSatrate(double rate) { stddevSatrate_ = rate; }
	double getStddevDistance() { return stddevDistance_; }
	void setStddevDistance(double dist) { stddevDistance_ = dist; }
	double getStddevVehicleDistance() { return stddevVehicleDistance_; }
	void setStddevVehicleDistance(double dist) { stddevVehicleDistance_ = dist; }
	double getStddevSpeed() { return stddevSpeed_; }
	void setStddevSpeed(double spd) { stddevSpeed_ = spd; }
	int getStddevNrofTravIntervals() { return stddevNrofTravIntervals_; }
	void setStddevNrofTravIntervals(int nrof) { stddevNrofTravIntervals_ = nrof; }

	int getStateError() { return stateerror_; }
	void setStateError(int err) { stateerror_ = err; }
	int getDemandError() { return demanderror_; }
	void setDemandError(int err) { demanderror_ = err; }
	int getMeanStateError() { return meanstateerror_; }
	void setMeanStateError(int err) { meanstateerror_ = err; }
	int getMeanDemandError() { return meandemanderror_; }
	void setMeanDemandError(int err) { meandemanderror_ = err; }
	int getStddevStateError() { return stddevstateerror_; }
	void setStddevStateError(int err) { stddevstateerror_ = err; }
	int getStddevDemandError() { return stddevdemanderror_; }
	void setStddevDemandError(int err) { stddevdemanderror_ = err; }

	std::unordered_map<int, int> nextSignalGroups() { return nextSignalGroups_; }
	void nextSignalGroup(int sgoid, int frac) { nextSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> prevSignalGroups() { return prevSignalGroups_; }
	void prevSignalGroup(int sgoid, int frac) { prevSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> disturbedNextSignalGroups() { return disturbedNextSignalGroups_; }
	void disturbedNextSignalGroup(int sgoid, int frac) { disturbedNextSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> disturbedPrevSignalGroups() { return disturbedPrevSignalGroups_; }
	void disturbedPrevSignalGroup(int sgoid, int frac) { disturbedPrevSignalGroups_[sgoid] = frac; }

	std::unordered_map<int, int> meanNextSignalGroups() { return meanNextSignalGroups_; }
	void meanNextSignalGroup(int sgoid, int frac) { meanNextSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> meanPrevSignalGroups() { return meanPrevSignalGroups_; }
	void meanPrevSignalGroup(int sgoid, int frac) { meanPrevSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> stddevNextSignalGroups() { return stddevNextSignalGroups_; }
	void stddevNextSignalGroup(int sgoid, int frac) { stddevNextSignalGroups_[sgoid] = frac; }
	std::unordered_map<int, int> stddevPrevSignalGroups() { return stddevPrevSignalGroups_; }
	void stddevPrevSignalGroup(int sgoid, int frac) { stddevPrevSignalGroups_[sgoid] = frac; }

	std::unordered_map<int, int> scenarioNextSignalGroups(int scnr) { return scenarioNextSignalGroups_[scnr]; }
	void scenarioNextSignalGroup(int scnr, int sgoid, int frac) { scenarioNextSignalGroups_[scnr][sgoid] = frac; }
	std::unordered_map<int, int> scenarioPrevSignalGroups(int scnr) { return scenarioPrevSignalGroups_[scnr]; }
	void scenarioPrevSignalGroup(int scnr, int sgoid, int frac) { scenarioPrevSignalGroups_[scnr][sgoid] = frac; }

private:
	int oid_{};
	std::string name_{};
	std::string jncname_{};
	int sgnr_{0};
	int internalsgnr_{0}; //renumbering of internal signalgroups; 0 if external;
	int minGRN_{0};
	int maxGRN_{0};
	int nrofLanes_{1};
	
	//NETWORK FUNCTIONALITY
	bool internalArr_{ false }; //internalArr==true if arrivals are comming from upstream junction;
	int externalBoundary_{ 0 }; //if internalArr == false; boundary number the external signal group belongs to
	double queuerate_{ 0.0 }; //propagation rate [veh/s] of the queue head
	double satrate_{ 0.0 }; //saturation rate [veh/s]
	double distance_{ 0.0 }; //distance/length [m] == storage capacity of the signal group
	double vehicleDistance_{ 0.0 }; //vehicle distance/length [m] == space of one vehicle in the queue 
	double speed_{ 0.0 }; //vehicle speed [km/h] on the link/lane approaching the signal group
	int nrofTravIntervals_{0}; // used to store the number of travelled intervals // constant during sim; // depends on controlintlngth;
	// 30 km/h = 30000 m / 3600 sec ; travel time = 100 m * 3600 sec / 30000 m = 12 sec => 2xtimeIntLngth; nrofTravIntervals = 2;
	double disturbedQueuerate_{ 0.0 };
	double disturbedSatrate_{ 0.0 };
	double disturbedDistance_{ 0.0 }; 
	double disturbedVehicleDistance_{ 0.0 };  
	double disturbedSpeed_{ 0.0 }; 
	int disturbedNrofTravIntervals_{ 0 }; 
	// mean for random disturbances
	double meanQueuerate_{ 0.0 };
	double meanSatrate_{ 0.0 };
	double meanDistance_{ 0.0 }; 
	double meanVehicleDistance_{ 0.0 }; 
	double meanSpeed_{ 0.0 }; 
	int meanNrofTravIntervals_{ 0 };
	// stddev for random disturbances
	double stddevQueuerate_{ 0.0 };
	double stddevSatrate_{ 0.0 };
	double stddevDistance_{ 0.0 };
	double stddevVehicleDistance_{ 0.0 };
	double stddevSpeed_{ 0.0 };
	int stddevNrofTravIntervals_{ 0 };

	int stateerror_{ 0 }; //real state + stateerror = disturbed state //stateerror will be reused & overwritten by the disturbed states (errors!) drawn from distribution;
	int demanderror_{ 0 }; //idem for demand disturbance
	int meanstateerror_{ 0 }; //real state + meanstateerror = mean state for the random distribution
	int meandemanderror_{ 0 }; //idem for demand disturbance
	int stddevstateerror_{ 0 }; //stddev state for the random distribution
	int stddevdemanderror_{ 0 }; //idem for demand disturbance

	std::unordered_map<int, int> nextSignalGroups_{}; //Signalgr numbers of next signal group <oid, frac> SUM==100; //store all networkconnections also if frac == 0!
	std::unordered_map<int, int> prevSignalGroups_{}; //Signalgr numbers of prev signal group <oid, frac> SUM<>100; //store all networkconnections also if frac == 0!
	std::unordered_map<int, int> disturbedNextSignalGroups_{}; // disturbed values
	std::unordered_map<int, int> disturbedPrevSignalGroups_{}; // disturbed values
	std::unordered_map<int, int> meanNextSignalGroups_{}; // mean for random disturbances
	std::unordered_map<int, int> meanPrevSignalGroups_{}; // mean for random disturbances
	std::unordered_map<int, int> stddevNextSignalGroups_{}; // stddev for random disturbances
	std::unordered_map<int, int> stddevPrevSignalGroups_{}; // stddev for random disturbances
	std::unordered_map<int, std::unordered_map<int, int>> scenarioNextSignalGroups_{}; // scenario values from file (for now instead of random values); per scenario <oid, frac>
	std::unordered_map<int, std::unordered_map<int, int>> scenarioPrevSignalGroups_{}; // scenario values from file (for now instead of random values); per scenario <oid, frac>

};
typedef std::shared_ptr<signalGroup> signalGroupPtr;

class phase
{
public:
	phase(int oid) :
		oid_{ oid } {}

	int oid() { return oid_; }

	std::unordered_map<int, signalGroupPtr> signalGroups() { return signalGroups_; }
	void signalGroup(int sgoid, signalGroupPtr sg) { signalGroups_[sgoid] = sg; }

	std::unordered_map<int, int> nextPhases() { return nextPhases_; }
	void nextPhase(int idx, int id) { nextPhases_[idx] = id; }

	std::unordered_map<int, int> basicPhases() { return basicPhases_; }
	void basicPhase(int jncidx, int id) { basicPhases_[jncidx] = id; }

private:
	int oid_{};
	std::unordered_map<int, signalGroupPtr> signalGroups_{}; //signal number
	std::unordered_map<int, int> nextPhases_{}; //int in map is the number of the dest phase	
	std::unordered_map<int, int> basicPhases_{}; //per junction number (NOT AIMSUN ID!), the number of the basicphase
};
typedef std::shared_ptr<phase> phasePtr;

class decisionNode
{
public:
	decisionNode(int oid, phasePtr phas, int tim, double cst, double cumcst, int prvNdeID) :
		oid_{ oid }, phase_{ phas }, timeint_{ tim }, cost_{ cst }, cumcost_{ cumcst }, prevNodeID_{ prvNdeID } {}

	int oid() { return oid_; }
	phasePtr phase() { return phase_; }
	int timeint() { return timeint_; }
	double cost() { return cost_; }
	double cumcost() { return cumcost_; }
	int prevNodeID() { return prevNodeID_; }
	
	void setPhase(phasePtr ph) { phase_ = ph; }
	void setTimeInt(int tmint) { timeint_ = tmint; }
	void setCost(double cst) { cost_ = cst; }
	void setCumCost(double cumcst) { cumcost_ = cumcst; }
	void setPrevNodeID(int id) { prevNodeID_ = id; }
	
	double cumjnccosts(int jncnr) { return cumjnccosts_[jncnr]; }
	void cumjnccost(int jncnr, double cumd) { cumjnccosts_[jncnr] = cumd; }

	double arrivalsLNK(int sgnr) { return arrivalsLNK_[sgnr]; }
	void arrivalLNK(int sgnr, double arr) { arrivalsLNK_[sgnr] = arr; }
	double cumarrivalsLNK(int sgnr) { return cumarrivalsLNK_[sgnr]; }
	void cumarrivalLNK(int sgnr, double cumarr) { cumarrivalsLNK_[sgnr] = cumarr; }
	double arrivals(int sgnr) { return arrivals_[sgnr]; }
	void arrival(int sgnr, double arr) { arrivals_[sgnr] = arr; }
	double cumarrivals(int sgnr) { return cumarrivals_[sgnr]; }
	void cumarrival(int sgnr, double cumarr) { cumarrivals_[sgnr] = cumarr; }
	double departures(int sgnr) { return departures_[sgnr]; }
	void departure(int sgnr, double dep) { departures_[sgnr] = dep; }
	double cumdepartures(int sgnr) { return cumdepartures_[sgnr]; }
	void cumdeparture(int sgnr, double cumdep) { cumdepartures_[sgnr] = cumdep; }
	double cumqueues(int sgnr) { return cumqueues_[sgnr]; }
	void cumqueue(int sgnr, double cumq) { cumqueues_[sgnr] = cumq; }
	double cumdelays(int sgnr) { return cumdelays_[sgnr]; }
	void cumdelay(int sgnr, double cumd) { cumdelays_[sgnr] = cumd; }
	double cumvehicles(int sgnr) { return cumvehicles_[sgnr]; }
	void cumvehicle(int sgnr, double cumv) { cumvehicles_[sgnr] = cumv; }
	double cumtimes(int sgnr) { return cumtimes_[sgnr]; }
	void cumtime(int sgnr, double cumt) { cumtimes_[sgnr] = cumt; }

	double heads(int sgnr) { return heads_[sgnr]; }
	void head(int sgnr, double hd) { heads_[sgnr] = hd; }
	double tails(int sgnr) { return tails_[sgnr]; }
	void tail(int sgnr, double tl) { tails_[sgnr] = tl; }
	double tmpheads(int sgnr) { return tmpheads_[sgnr]; }
	void tmphead(int sgnr, double hd) { tmpheads_[sgnr] = hd; }
	double tmptails(int sgnr) { return tmptails_[sgnr]; }
	void tmptail(int sgnr, double tl) { tmptails_[sgnr] = tl; }

	int leadvehids(int sgnr) { return leadvehids_[sgnr]; }
	void leadvehid(int sgnr, int id) { leadvehids_[sgnr] = id; }
	int vehids(int internalsgnr, int pos) { return vehids_[internalsgnr][pos]; }
	void vehid(int internalsgnr, int pos, int id) { vehids_[internalsgnr][pos] = id; }
	int getSizeVehids(int internalsgnr) { return (int)vehids_[internalsgnr].size(); }

	int stateDurations(int fcnr) { return stateDurations_[fcnr]; }
	void stateDuration(int oid, int statedur) { stateDurations_[oid] = statedur; }
	std::string states(int fcnr) { return states_[fcnr]; }
	void state(int oid, std::string st) { states_[oid] = st; }

	int subNodeIDs(int idx) { return subNodeIDs_[idx]; }
	void subNodeID(int idx, int oid) { subNodeIDs_[idx] = oid; }

	double getSuperCumCost() { return supercumcost_; }
	void setSuperCumCost(double cst) { supercumcost_ = cst; }

private:
	int oid_; //index of the decision node

	phasePtr phase_{ }; //decision=phase that is set to green
	int timeint_{ 0 }; // level of the node (time interval for which decision is taken)
	double cost_{ 0.0 }; //cost (delay) of turning the phase to green for the next time interval
	double cumcost_{ 0.0 }; //cumulative costs delay of the decision sequence so far
	int prevNodeID_{ 0 }; // index of the previos decision node

	std::array<double, 5> cumjnccosts_{ };//cumcosts subtotals per junction; sum of array equals cumcost_ 

	std::array<double, 49> arrivalsLNK_{ }; // arrivals at link entrance per signal group idx, total is stored at int=0; copy of values at controlint
	std::array<double, 49> cumarrivalsLNK_{ }; // cumulative arrivals at link entrance per signal group idx, total is stored at int=0; copy of values at controlint
	std::array<double, 49> arrivals_{ }; // arrivals at queue per signal group idx, total is stored at int=0; copy of values at controlint
	std::array<double, 49> cumarrivals_{ }; // cumulative arrivals at queue per signal group idx, total is stored at int=0; copy of values at controlint
	std::array<double, 49> departures_{ }; // departures per signal group idx, total is stored at int=0
	std::array<double, 49> cumdepartures_{ }; // cumulative departures per signal group idx, total is stored at int=0
	std::array<double, 49> cumqueues_{ }; // queue states per signal group idx, total is stored at int=0
	std::array<double, 49> cumdelays_{ }; // delays per signal group idx, total is stored at int=0; total is equal to cumcost!
	std::array<double, 49> cumvehicles_{ }; // cumulative delays per signal group idx, total is stored at int=0; all vehicles on link;
	std::array<double, 49> cumtimes_{ }; // cumulative total time spend (TTS) per signal group idx, total is stored at int=0; (alternative objective)

	std::array<double, 49> heads_{ }; //per signal group; leading head of first queue
	std::array<double, 49> tails_{ }; //per signal group; leading tail of first queue
	std::array<double, 49> tmpheads_{ }; //per signal group; tmp head of last queue
	std::array<double, 49> tmptails_{ }; //per signal group; tmp tail of last queue
	std::array<int, 49> leadvehids_{ }; //per signal group; leading vehicle (oid) of queue
	std::array<std::array<int, 51>, 25> vehids_{ }; //per INTERNAL signal group (max 24); array with at each position the id of the vehicle; pos 1 = id of leading vehicle, etc (max 18 positions in link)

	std::array<int, 49> stateDurations_{ }; // per signal group; duration of green state 
	std::array<std::string, 49> states_{ }; // per signal group; "GRN" green / "RED" red state

	//Supernode: collection of decision nodes of the different model uncertainty scenarios (for robust control)
	std::array<int, 51> subNodeIDs_{ }; //indices of the sub decision nodes; -1's no subnodes; first field own number!
	double supercumcost_{ 0.0 }; //objective costs of the decision sequence so far; function of cumcost over the subnodes! 
};
typedef std::shared_ptr<decisionNode> decisionNodePtr;

class detector
{
public:
	detector(int oid,  std::string nam , std::string signam, std::string jncnam, int sgnr) :
		oid_{ oid },  name_{ nam }, signame_{ signam }, jncname_{ jncnam }, sgnr_{ sgnr } {}

	int oid() { return oid_; }
	std::string name() { return name_; }
	std::string signame() { return signame_; }
	std::string jncname() { return jncname_; }
	int sgnr() { return sgnr_; }
private:
	int oid_;
	std::string name_;
	std::string signame_;
	std::string jncname_;
	int sgnr_;
};
typedef std::shared_ptr<detector> detectorPtr;

class junction
{
public:
	junction(int oid, std::string nam, int jnr ) :
		oid_{ oid }, name_{ nam }, jncnr_{ jnr } {}

	int oid() { return oid_; }
	std::string name() { return name_; }
	int jncnnr() { return jncnr_; }
private:
	int oid_;
	std::string name_;
	int jncnr_{0};
};
typedef std::shared_ptr<junction> junctionPtr;

class passage 
{
public:
	passage(int oid, int dtid, std::string dtnam, std::string sgnam, std::string jnam, int sgnr) :
		oid_{ oid }, detid_{ dtid }, detname_{ dtnam }, signame_{ sgnam }, jncname_{ jnam }, sgnr_{ sgnr } {}

	int oid() { return oid_; }
	int detid() { return detid_; }
	std::string detname() { return detname_; }
	std::string signame() { return signame_; }
	std::string jncname() { return jncname_; }
	int sgnr() { return sgnr_; }
	double predtime() { return predtime_; }
	double lagtime() { return lagtime_; }
	double realtime() { return realtime_; }
	double delay() { return delay_; }

	void setPredTime(double tme) { predtime_ = tme; }
	void setLagTime(double tme) { lagtime_ = tme; }
	void setRealTime(double tme) { realtime_ = tme; }
	void setDelay(double tme) { delay_ = tme; }

private:
	int oid_;
	int detid_;
	std::string detname_;
	std::string signame_;
	std::string jncname_;
	int sgnr_;
	double predtime_{ 0. }; //freeflow predicted passage time
	double lagtime_{ 0. };  //lagged predicted passage time under upstream delays
	double realtime_{ 0. }; //real passage time (under upstream delays + delay of own signal group)
	double delay_{ 0. }; // delay at own signal group defined as realtime - lagtime -> sum over all vehicles and signal groups is total system delay;
	// external signal group 1: lagtime1 == predtime1, and delay1 = realtime1 - predtime1 == realtime1 - lagtime1;
	// internal signal group 2: lagtime2 >> predtime2, lagtime2 = realtime1 + (predtime2 - predtime1), 
	// delay2 = realtime2-lagtime2 = realtime2 -(realtime1+predtime2-predtime1) = realtime2-predtime2 - (realtime1-predtime1) = delay - delay1
	// total delay over 1 and 2: delay = realtime2 - predtime2 == delay1+delay2 !!!

};
typedef std::shared_ptr<passage> passagePtr;

class vehicle
{
public:
	vehicle(int oid) :
		oid_{ oid } {}

	int oid() { return oid_; }
	std::unordered_map<int, passagePtr> passages() { return passages_; }
	void passage(int idx, passagePtr ptr) { passages_[idx] = ptr; }
	int getNrofPassages() {return nrofPassages_;}
	void setNrofPassages(int nrof) { nrofPassages_=nrof; }
	int getNextVehID() { return nextvehid_; }
	void setNextVehID(int vehid) { nextvehid_ = vehid; }
	int getSeqNr() { return seqnr_; }
	void setSeqNr(int sqnr) { seqnr_ = sqnr; }
	
private:
	int oid_;
	std::unordered_map<int, passagePtr> passages_{}; // int is the passage number=order of passages in time;
	int nrofPassages_{0};
	int nextvehid_{0}; //next vehicle in entrance order
	int seqnr_{0}; // order/position of entrance; 1=first vehicle entering the (external) signal group; 2=second, etc
};
typedef std::shared_ptr<vehicle> vehiclePtr;

class ControlIntervals {
public:
	decisionNodePtr get(int idx) {
		std::lock_guard<std::mutex> lock(data_mutex_);
		auto it = controlIntervals_.find(idx);
		if (it != controlIntervals_.end())
			return it->second;
		else
			return nullptr;
	}
	void add(int idx, decisionNodePtr controlintpr)
	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		controlIntervals_[idx] = controlintpr;
	}

private:
	std::mutex data_mutex_;
	std::unordered_map<int, decisionNodePtr> controlIntervals_{}; // idx number;
};

class randomParameters {
public:
	double getRandomSatrate(int scnr, int timeidx, int sgnr) { return randomSatrates_[scnr][timeidx][sgnr]; }
	void setRandomSatrate(int scnr, int timeidx, int sgnr, double val) { randomSatrates_[scnr][timeidx][sgnr] = val; }
	int getRandomNrofTravInt(int scnr, int timeidx, int sgnr) { return randomNrofTravInts_[scnr][timeidx][sgnr]; }
	void setRandomNrofTravInt(int scnr, int timeidx, int sgnr, int val) { randomNrofTravInts_[scnr][timeidx][sgnr] = val; }
	int getRandomStateError(int scnr, int timeidx, int sgnr) { return randomStateErrors_[scnr][timeidx][sgnr]; }
	void setRandomStateError(int scnr, int timeidx, int sgnr, int val) { randomStateErrors_[scnr][timeidx][sgnr] = val; }
	int getRandomDemandError(int scnr, int timeidx, int sgnr) { return randomDemandErrors_[scnr][timeidx][sgnr]; }
	void setRandomDemandError(int scnr, int timeidx, int sgnr, int val) { randomDemandErrors_[scnr][timeidx][sgnr] = val; }
	int getRandomNextSignalGroup(int scnr, int timeidx, int sgnr, int nextsgid) { return randomNextSignalGroups_[scnr][timeidx][sgnr][nextsgid]; }
	void setRandomNextSignalGroup(int scnr, int timeidx, int sgnr, int nextsgid, int frac) { randomNextSignalGroups_[scnr][timeidx][sgnr][nextsgid] = frac; }
	int getRandomPrevSignalGroup(int scnr, int timeidx, int sgnr, int prevsgid) { return randomPrevSignalGroups_[scnr][timeidx][sgnr][prevsgid]; }
	void setRandomPrevSignalGroup(int scnr, int timeidx, int sgnr, int prevsgid, int frac) { randomPrevSignalGroups_[scnr][timeidx][sgnr][prevsgid] = frac; }
	double getScenarioWeight(int scnr, int timeidx) { return scenarioWeights_[scnr][timeidx]; }
	void setScenarioWeight(int scnr, int timeidx, double val) { scenarioWeights_[scnr][timeidx] = val; }
	bool getScenarioActive(int scnr, int timeidx) { return scenarioActive_[scnr][timeidx]; }
	void setScenarioActive(int scnr, int timeidx, bool val) { scenarioActive_[scnr][timeidx] = val; }
	
private:
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> randomSatrates_{}; //per scenario, per timeidx, per sgnr, the random parameter value
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> randomNrofTravInts_{}; //per scenario, per timeidx, per sgnr, the random parameter value
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> randomStateErrors_{}; //per scenario, per timeidx (only at 0), per sgnr, the random error value
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> randomDemandErrors_{}; //per scenario, per timeidx, per sgnr, the random error value
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>>> randomNextSignalGroups_{}; //per scenario, per timeidx, per sgnr, the random fractions <nextsigid,frac>
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>>> randomPrevSignalGroups_{}; //per scenario, per timeidx, per sgnr, the random fractions <prevsigid,frac>
	std::unordered_map<int, std::unordered_map<int, double>> scenarioWeights_{}; //per scenario, per timeidx, the probability/weight of the scenario;
	std::unordered_map<int, std::unordered_map<int, bool>> scenarioActive_{}; //per scenario, per timeidx, true if the scenario is active; false if not;
};
//typedef std::shared_ptr<randomParameters> randomParametersPtr;

class threadswitch
{
public:
	bool getDecision() { return newDecision_; }
	void setDecision(bool dc) { newDecision_ = dc; }
	bool getStatistics() { return newStatistics_; }
	void setStatistics(bool stat) { newStatistics_ = stat; }
	bool getControlAvail() { return controlAvail_; }
	void setControlAvail(bool ctr) { controlAvail_ = ctr; }
	bool getAbortDecision() { return abortDecision_; }
	void setAbortDecision(bool adc) { abortDecision_ = adc; }
	bool getStalledProcess() { return stalledProcess_; }
	void setStalledProcess(bool stal) { stalledProcess_ = stal; }

private:
	std::atomic<bool> newDecision_{false}; //true if new decision needs to be calculated
	std::atomic<bool> newStatistics_{false}; //true if new statistics needs to be calculated
	std::atomic<bool> controlAvail_{false}; //true if new control sequence is available (can be suboptimum find sofar)
	std::atomic<bool> abortDecision_{false}; //true if new control sequence is available (can be suboptimum find sofar)
	std::atomic<bool> stalledProcess_{false}; //true if Aimsun is stalled; false if Aimsun is running 
};
typedef std::shared_ptr<threadswitch> threadswitchPtr;

class centroid
{
public:
	centroid(int oid) :
		oid_{ oid } {}

	int oid() { return oid_; }
	double getDemand(int idx) { return demands_[idx]; }
	void setDemand(int idx, double dem) { demands_[idx] = dem; }
private:
	int oid_;
	std::unordered_map<int, double> demands_{}; // idx number;
};
typedef std::shared_ptr<centroid> centroidPtr;

class network
{
public: 
	std::unordered_map<int, signalGroupPtr> signalGroups() { return signalGroups_; }
	void signalGroup(int oid, signalGroupPtr sg) { signalGroups_[oid] = sg; }
	phasePtr phases(int oid) { 
		auto it = phases_.find(oid);
		if (it != phases_.end())
			return it->second;
		else
			return nullptr; 
	}
	void phase(int oid, phasePtr ph) { phases_[oid] = ph; }
	phasePtr basicPhases(int oid) {
		auto it = basicphases_.find(oid);
		if (it != basicphases_.end())
			return it->second;
		else
			return nullptr;
	}
	void basicPhase(int oid, phasePtr ph) { basicphases_[oid] = ph; }
	std::unordered_map<int, detectorPtr> detectors() { return detectors_; }
	void detector(int oid, detectorPtr dt) { detectors_[oid] = dt; }
	std::unordered_map<int, junctionPtr> junctions() { return junctions_; }
	void junction(int oid, junctionPtr jnc) { junctions_[oid] = jnc; }
	std::unordered_map<int, centroidPtr> centroids() { return centroids_; }
	void centroid(int oid, centroidPtr ctr) { centroids_[oid] = ctr; }
	int getNrofPhases() { return nrofPhases_; }
	void setNrofPhases(int nrof) { nrofPhases_ = nrof; }
	int getNrofBasicPhases() { return nrofBasicPhases_; }
	void setNrofBasicPhases(int nrof) { nrofBasicPhases_ = nrof; }
	phasePtr getActivePhase() { return activePhase_; }			//in Aimsun!!!
	void setActivePhase(phasePtr ph) { activePhase_ = ph; }		//from Aimsun!!!

private:
	std::unordered_map<int, signalGroupPtr> signalGroups_{}; //AimsunID 
	std::unordered_map<int, phasePtr> phases_{}; //phase number
	std::unordered_map<int, phasePtr> basicphases_{}; //phase number
	std::unordered_map<int, detectorPtr> detectors_{}; //AimsunID
	std::unordered_map<int, junctionPtr> junctions_{}; //AimsunID
	std::unordered_map<int, centroidPtr> centroids_{}; //centroid number (external AimsunID)
	int nrofPhases_{0};
	int nrofBasicPhases_{ 0 };
	phasePtr activePhase_; //phase that is active = green; Aimsun
};
typedef std::shared_ptr<network> networkPtr;

class simulator
{
	public:
		simulator(double sss) :
			sss_{ sss } {}
		double sss() { return sss_; }
		networkPtr network() { return net_; }
		void network(networkPtr net) { net_ = net; }

	private:
		double sss_{0.};	// store the simulation step size
		networkPtr net_{};
};
typedef std::shared_ptr<simulator> simulatorPtr;

enum class aimsunSGState			// To be used to send required signal group state to simulator (Aimsun)
{
	//	0 means RED, 1 means GREEN, 2 means YELLOW, 3 means FLASHING GREEN, 4 means FLASHING RED and 5 means FLASHING YELLOW).
	RED = 0,
	GRN = 1,
	YLW = 2,
	FLY = 5
};

//Initialise mapping for Aimsun signal group states
std::map<aimsunSGState, std::string> SGStateMap = {
	{ aimsunSGState::RED, "RED" },
	{ aimsunSGState::YLW, "YLW" },
	{ aimsunSGState::GRN, "GRN" },
	{ aimsunSGState::FLY, "FLY" }
};

bool sendSGUpdateToAimsun(std::weak_ptr<WebSocketClient> clientPtr, std::string sgAimsunID, std::string nextState);
