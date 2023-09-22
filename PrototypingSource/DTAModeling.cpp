/*
Copyright(C) 2023 Royal HaskoningDHV / Delft University of Technology
Author: Muriel Verkaik-Poelman (muriel.verkaik-poelman@rhdhv.com)

This program is free software : you can redistribute it and /or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see http ://www.gnu.org/licenses/.
*/

#pragma warning(disable: 4503)
#include "DTAModeling.h"

// Macro to print data in the debug output window. Syntax is DBOUT("string to be printed" << "more strings to be printed");
#define DBOUT( s )            \
{                             \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}

int ReleaseSemaphore()
{
	int releasedSemaphore = 0;
	HANDLE ghSemaphore = OpenSemaphore(
		SEMAPHORE_MODIFY_STATE,           // get modify access
		FALSE,  // inherit handle
		LPCWSTR("SEM_DTAM"));          // named semaphore

	if (ghSemaphore == NULL)
	{
		printf("OpenSemaphore error: %d\n", GetLastError());
	}
	else {
		printf("OpenSemaphore success\n");
		DWORD dwWaitResult;
		dwWaitResult = ReleaseSemaphore(
			ghSemaphore,    // semaphore handle
			1,              // how many to release
			NULL            // previous count
		);

		if (dwWaitResult == NULL)
		{
			printf("ReleaseSemaphore error: %d\n", GetLastError());
		}
		else {
			printf("ReleaseSemaphore success\n");
			releasedSemaphore = 1;
		}
	}
	return releasedSemaphore;
}

//WriteSettings: Print user-specified settings to file for checking and logging purposes
//INPUT: user-specified settings in DTAModeling.h
//OUTPUT: settingsfile.txt with the applied user-specified settings
void writeSettings(std::ofstream& settingsfile, optimizationSettings& settings) {

	//dump settings to file: to know what settings have been run!
	//process settings
	settingsfile << "PROCESS SETTINGS" << std::endl;
	settingsfile << "preprocessing" << "\t" << settings.preprocessing << std::endl;
	settingsfile << std::endl;

	//Rolling horizon settings
	settingsfile << "ROLLING HORIZON SETTINGS" << std::endl;
	settingsfile << "controlIntLngth" << "\t" << settings.controlIntLngth << std::endl;
	settingsfile << "nrofDecisionInt" << "\t" << settings.nrofDecisionInt << std::endl;
	settingsfile << "nrofPredictionInt" << "\t" << settings.nrofPredictionInt << std::endl;
	settingsfile << "nrofSimInt" << "\t" << settings.nrofSimInt << std::endl;
	settingsfile << "controlIntIdx" << "\t" << settings.controlIntIdx << std::endl;
	settingsfile << std::endl;

	//Prediction model settings
	settingsfile << "PREDICTION MODEL SETTINGS" << std::endl;
	settingsfile << "lossTime" << "\t" << settings.lossTime << std::endl;
	settingsfile << "blockSignals" << "\t" << settings.blockSignals << std::endl;
	settingsfile << "headtail" << "\t" << settings.headtail << std::endl;
	settingsfile << "individualqueue" << "\t" << settings.individualqueue << std::endl;
	settingsfile << "individualdemand" << "\t" << settings.individualdemand << std::endl;
	settingsfile << "headtailSTAT" << "\t" << settings.headtailSTAT << std::endl;
	settingsfile << "individualqueueSTAT" << "\t" << settings.individualqueueSTAT << std::endl;
	settingsfile << "individualdemandSTAT" << "\t" << settings.individualdemandSTAT << std::endl;
	settingsfile << std::endl;

	//optimization settings
	settingsfile << "OPTIMIZATION SETTINGS" << std::endl;
	settingsfile << "objdelay (1: delay; 0: TTS)" << "\t" << settings.objdelay << std::endl;
	settingsfile << "suboptimal" << "\t" << settings.suboptimal << std::endl;
	settingsfile << "activateMethod (1 = BB; 2 = GA)" << "\t" << settings.activateMethod << std::endl;
	settingsfile << "BB: useTotalState" << "\t" << settings.useTotalState << std::endl;
	settingsfile << "BB: boundpercentage" << "\t" << settings.boundpercentage << std::endl;
	settingsfile << "BB: boundabsolute" << "\t" << settings.boundabsolute << std::endl;
	settingsfile << "BB: nrofLevelSearchNodes" << "\t" << settings.nrofLevelSearchNodes << std::endl;
	settingsfile << std::endl;

	//robust settings
	settingsfile << "ROBUST SETTINGS" << std::endl;
	settingsfile << "objrobust (1:non-robust; 2:single scenario; 3:minmax; 4:weightedaverage; 5:average)" << "\t" << settings.objrobust << std::endl;
	settingsfile << "nrofScenarios" << "\t" << settings.nrofScenarios << std::endl;
	settingsfile << "scenarioWeight" ;
	for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) { settingsfile << "\t" << settings.scenarioWeight[scnr]; }
	settingsfile << std::endl;
	settingsfile << "randomSaturationRates" << "\t" << settings.randomSaturationRates << std::endl;
	settingsfile << "randomTravelTimes" << "\t" << settings.randomTravelTimes << std::endl;
	settingsfile << "randomState" << "\t" << settings.randomState << std::endl;
	settingsfile << "randomDemand" << "\t" << settings.randomDemand << std::endl;
	settingsfile << "randomFractions" << "\t" << settings.randomFractions << std::endl;
	settingsfile << "nrofScenarioPeriods" << "\t" << settings.nrofScenarioPeriods << std::endl;
	settingsfile << "scenarioPeriods";
	for (int scper = 0; scper <= settings.nrofScenarioPeriods+1; ++scper) { settingsfile << "\t" << "[" << settings.scenarioPeriods[scper][0] << "," << settings.scenarioPeriods[scper][1] << "]"; }
	settingsfile << std::endl;
	settingsfile << "robustHeuristic" << "\t" << settings.robustHeuristic << std::endl;
	settingsfile << "nrofScenariosSubset" << "\t" << settings.nrofScenariosSubset << std::endl;
	settingsfile << "scenarioActive";
	for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) { settingsfile << "\t" << settings.scenarioActive[scnr]; }
	settingsfile << std::endl;
	settingsfile << "scenarioLog" << "\t" << settings.scenarioLog << std::endl;
	settingsfile << std::endl;

	//Extend with usefull settings ...
}

//ReadSignals: Read user-specified prediction model and control parameters for the dedicated links of the different signal groups (one movement per signal group) 
//INPUT: signals.txt file with user-specified model and control parameters per signal group/movement in the format
//[nodeid, signalid(1-12), mingreen(s), maxgreen(s), nroflanes, internal(yes/no), distance(m), vehicledistance(m), speed(km/h), saturationrate(veh/s), queueheadrate(veh/s), disturbed distance(m), disturbed vehicledistance(m), disturbed speed(km/h), disturbed saturationrate(veh/s), disturbed queueheadrate(veh/s), stateerror(%), demanderror(%)]
//Note: first parameter values are the true undisturbed average values as reference for sensitivity analysis (nrofscenarios==0), or as default certain value outside the uncertainty scenario periods for robust control (nrofscenarios>0) 
//Note: second parameter values are the disturbed average values for sensitivity analysis (nrofscenarios==0), or the mean values over all scenarios during the uncertainty scenario periods for robust control (nrofscenarios>0)    
//Note: TODO: specify parameter values in file for the scenarios or randomly draw these scenarios around mean values in GenerateRandomDisturbances
//OUTPUT: a map signalGroups_ with signalGroup objects with the user-specified model and control parameters as attribute values.
void readSignals(std::ifstream& signalfile, networkPtr& net, optimizationSettings& settings)
{
	auto signalGroups = net->signalGroups();
	int sigid, mingreen, maxgreen, nodeid;
	int nroflanes, internalArr, distance, vehdist, speed, distance2, vehdist2, speed2, nrofinternals;
	int stateerror, demanderror;
	double satrate, satrate2, queuerate, queuerate2;
	nrofinternals = 0;
	while (signalfile >> nodeid >> sigid >> mingreen >> maxgreen >> nroflanes >> internalArr >> distance >> vehdist >> speed >> satrate >> queuerate >> distance2 >> vehdist2 >> speed2 >> satrate2 >> queuerate2 >> stateerror >> demanderror)
	{
		int sgnr = 12 * (nodeid - 1) + sigid; //is same as order of appearance in file (Sorted on nodeid and sigid)
		for (auto [sgID, sg] : signalGroups)
		{// each signalgroup
			if (sgnr == sg->getSgnr()) {
				sg->setMinGreen(mingreen);
				sg->setMaxGreen(maxgreen);
				sg->setNrofLanes(nroflanes);
				if (internalArr == 1) {
					nrofinternals = nrofinternals + 1;
					sg->setInternalArr(true);
					sg->setInternalSgnr(nrofinternals); //same as order of appearance in file;
				}
				sg->setDistance(distance);
				sg->setVehicleDistance(vehdist);
				sg->setSpeed(speed);
				sg->setSatrate(satrate);
				sg->setQueuerate(queuerate);
				sg->setDisturbedDistance(distance2);
				sg->setDisturbedVehicleDistance(vehdist2);
				sg->setDisturbedSpeed(speed2);
				sg->setDisturbedSatrate(satrate2);
				sg->setDisturbedQueuerate(queuerate2);

				//Mean for random disturbance equals the disturbed value (if scenarios>0, disturbed values are overwritten by random disturbances)
				sg->setMeanDistance(distance2);
				sg->setMeanVehicleDistance(vehdist2);
				sg->setMeanSpeed(speed2);
				sg->setMeanSatrate(satrate2); 
				sg->setMeanQueuerate(queuerate2);

				//Standard deviation for random disturbance, hardcoded for now
				sg->setStddevDistance(0.0);
				sg->setStddevVehicleDistance(0.0);
				sg->setStddevSpeed(0.0);
				sg->setStddevSatrate(0.0); 
				sg->setStddevQueuerate(0.0);

				sg->setStateError(stateerror); //(percentage!)
				sg->setDemandError(demanderror); //(percentage!)
				//Mean error for random disturbance equals the error value (if scenarios>0, error values are overwritten by random drawn errors)
				sg->setMeanStateError(stateerror); //(percentage!)
				sg->setMeanDemandError(demanderror); //(percentage!)
				//Standard deviation for random disturbance, hardcoded for now
				sg->setStddevStateError(0); //(percentage!) 
				sg->setStddevDemandError(0); //(percentage!) 

				int nrofint = 0;
				double travtime = 3.6 * sg->getDistance() / sg->getSpeed(); // 3.6*100/30 = 12 sec
				nrofint = (int)ceil(travtime / settings.controlIntLngth); // at least 1 interval back; 12/6 = 2 intervals;
				sg->setNrofTravIntervals(nrofint);
				int nrofint2 = 0;
				double travtime2 = 3.6 * sg->getDisturbedDistance() / sg->getDisturbedSpeed(); 
				nrofint2 = (int)ceil(travtime2 / settings.controlIntLngth); 
				sg->setDisturbedNrofTravIntervals(nrofint2); //recalculated if disturbed distance and/or disturbed speed changes
				sg->setMeanNrofTravIntervals(nrofint2); //only used to store average value
				sg->setStddevNrofTravIntervals(0); // not used;
			}
		}
	}
	std::cout << "signal file processed" << std::endl;
}

//ReadSplitFractions: Read user-specified turn fractions for the prediction model defining the traffic flow relation between the different signal groups
//INPUT1: splitfractions.txt file with user-specified turn fractions from each signal group to the other signal groups in the format:
//[fromnodeid(=0 centroid,>0 node), fromsignalid(zoneidx for centroids, 1-12 for nodes), tonodeid, tosignalid(1-12), fraction(%), disturbed(mean)fraction(%), fractionScenario1(%), ..., fractionScenarioN(%)]
//Note: fraction(%) is the true undisturbed average fraction as reference for sensitivity analysis (nrofscenarios==0), or as default certain value outside the uncertainty scenario periods for robust control (nrofscenarios>0) 
//Note: disturbedfraction(%) is the disturbed average fraction for sensitivity analysis (nrofscenarios==0), or the mean turn fraction value over all scenarios during the uncertainty scenario periods for robust control (nrofscenarios>0)    
//Note: fractionScenario1(%), ..., fractionScenarioN(%) are the scenario values for the turnfractions during the uncertainty scenario periods for robust control (nrofscenarios>0)
//Note: TODO: insetad of specifying turn fraction values in file for the scenarios, randomly draw these scenarios around mean values in GenerateRandomDisturbances
//INPUT2: demand.txt file with user-specified average demands at network entrances / at external signalgroups (equal to OD matrix in Aimsun) with format:
//[centroidIDX, demandInterval1, ..., demandIntervalM]
//OUTPUT1: a map signalGroups_ with signalGroup objects with the user-specified turn fractions as attribute values.
//OUTPUT2: a map centroids_ with centroid objects with the average demand values (can be used to predict arrivals based on aggregated demand input and turn fractions)
void readSplitFractions(std::ifstream& demandfile, std::ifstream& splitfile, networkPtr& net, optimizationSettings& settings)
{
	auto signalGroups = net->signalGroups();
	int fromnodeid, tonodeid, fromsigid, tosigid, fraction, fraction2, centroidid;
	double demflow;
	std::unordered_map<int, int> sceneriofrac; //per scenarionumber the fractions
	
	while (splitfile >> fromnodeid >> fromsigid >> tonodeid >> tosigid >> fraction >> fraction2)
	{
		//read scenario values (if any)
		for (int scnr = 1; scnr <= settings.nrofScenarios; ++scnr) {
			splitfile >> sceneriofrac[scnr];
		}

		int fromsgnr = 0;
		int tosgnr = 0;

		if (fromnodeid > 0) { fromsgnr = 12 * (fromnodeid - 1) + fromsigid; }
		if (tonodeid > 0) { tosgnr = 12 * (tonodeid - 1) + tosigid; }
		signalGroupPtr toSignalGroup = nullptr;
		signalGroupPtr fromSignalGroup = nullptr;
		for (auto [sgID, sg] : signalGroups) {
			if (fromsgnr == sg->getSgnr()) { fromSignalGroup = sg; }
			if (tosgnr == sg->getSgnr()) { toSignalGroup = sg; }
		}

		if (fromSignalGroup != nullptr) {
			if (toSignalGroup != nullptr) {			
				//store all relations; also with frac 0; to model biased splitfractions!
				fromSignalGroup->nextSignalGroup(toSignalGroup->oid(), fraction);
				toSignalGroup->prevSignalGroup(fromSignalGroup->oid(), fraction);
				fromSignalGroup->disturbedNextSignalGroup(toSignalGroup->oid(), fraction2);
				toSignalGroup->disturbedPrevSignalGroup(fromSignalGroup->oid(), fraction2);

				fromSignalGroup->meanNextSignalGroup(toSignalGroup->oid(), fraction2);
				toSignalGroup->meanPrevSignalGroup(fromSignalGroup->oid(), fraction2);
				fromSignalGroup->stddevNextSignalGroup(toSignalGroup->oid(), 0);
				toSignalGroup->stddevPrevSignalGroup(fromSignalGroup->oid(), 0);

				for (int scnr = 1; scnr <= settings.nrofScenarios; ++scnr) {
					fromSignalGroup->scenarioNextSignalGroup(scnr, toSignalGroup->oid(), sceneriofrac[scnr]);
					toSignalGroup->scenarioPrevSignalGroup(scnr, fromSignalGroup->oid(), sceneriofrac[scnr]);
				}
			}
		}
		else {//from centroid to external signal group
			if (toSignalGroup != nullptr) {
				toSignalGroup->setExternalBoundary(fromsigid);
				//store all relations; also with frac 0; to model biased splitfractions!
				toSignalGroup->prevSignalGroup(toSignalGroup->oid(), fraction);
				toSignalGroup->disturbedPrevSignalGroup(toSignalGroup->oid(), fraction2);

				toSignalGroup->meanPrevSignalGroup(toSignalGroup->oid(), fraction2);
				toSignalGroup->stddevPrevSignalGroup(toSignalGroup->oid(), 0);

				for (int scnr = 1; scnr <= settings.nrofScenarios; ++scnr) {
					toSignalGroup->scenarioPrevSignalGroup(scnr, toSignalGroup->oid(), sceneriofrac[scnr]);
				}
			}
		}	
	}

	//Couple external signal groups with same boundary
	for (auto [sgID, sg] : signalGroups) {
		if (sg->getExternalBoundary() > 0) {//external signal group
			for (auto [tmpsgID, tmpsg] : signalGroups) {
				if (tmpsg->getExternalBoundary()== sg->getExternalBoundary()) {
					sg->prevSignalGroup(tmpsg->oid(), tmpsg->prevSignalGroups()[tmpsg->oid()] );
					sg->disturbedPrevSignalGroup(tmpsg->oid(), tmpsg->disturbedPrevSignalGroups()[tmpsg->oid()]);

					sg->meanPrevSignalGroup(tmpsg->oid(), tmpsg->meanPrevSignalGroups()[tmpsg->oid()]);
					sg->stddevPrevSignalGroup(tmpsg->oid(), tmpsg->stddevPrevSignalGroups()[tmpsg->oid()]);

					for (int scnr = 1; scnr <= settings.nrofScenarios; ++scnr) {
						sg->scenarioPrevSignalGroup(scnr, tmpsg->oid(), tmpsg->scenarioPrevSignalGroups(scnr)[tmpsg->oid()]);
					}
				}
			}
		}
	}

	//read file with average demand at boundaries; [centroidIDX, demandint1, ..., demandintN]; make centroids and setdemand!
	//used to predict arrivals based on aggregated demand input and fractions, instead of real vehicle arrivals;
	while (demandfile >> centroidid)
	{
		centroidPtr tmpcentroid = std::make_shared<centroid>(centroidid);
		for (int k = 1; k <= settings.nrofSimInt; ++k)
		{
			demandfile >> demflow;
			tmpcentroid->setDemand(k,demflow);
		}
		net->centroid(centroidid, tmpcentroid);
	}

	std::cout << "split file processed" << std::endl;
}

//ReadBasicPhases: Read user-specified basic phases / movement groups (non-conflicting groups of signals) that are identical for each junction
//INPUT: phasesbasic.txt file with user specified phases / groups of non-conflicting movements that are identical for each junction in the format:
//[phaseid, basicphaseid(==phaseid), nodeid(==0,node independent), sigid(1-12)] with multiple lines per phaseid to specify the basic active movements (node independent)
//OUTPUT: a map basicphases_ with phase objects with an attribute list of signalgroup objects belonging to each phase (same movement group is listed for each junction)
void readBasicPhases(std::ifstream& basicphasefile, networkPtr& net, optimizationSettings& settings)
{
	auto signalGroups = net->signalGroups();
	int phaseid, basicphaseid, sigid, nodeid;
	
	//initialize all red phase
	phasePtr basicph = std::make_shared<phase>(0);
	net->basicPhase(0, basicph);
	int nrofBasicPhases = 0;
	while (basicphasefile >> phaseid >> basicphaseid >> nodeid >> sigid)
	{
		phasePtr basicph = net->basicPhases(phaseid);
		if (basicph == nullptr) {
			basicph = std::make_shared<phase>(phaseid);
			net->basicPhase(phaseid, basicph);
			nrofBasicPhases = nrofBasicPhases + 1;
			net->setNrofBasicPhases(nrofBasicPhases);
		}
		//loop over junctions
		for (auto [jncID, jnc] : net->junctions()) { //store same info for all junctions
			int nodeid = jnc->jncnnr();
			int sgnr = 12 * (nodeid - 1) + sigid;
			for (auto [sgID, sg] : signalGroups)
			{// each signalgroup
				if (sgnr == sg->getSgnr()) {
					basicph->signalGroup(sgnr, sg);
				}
			}
			//ADD BASIC PHASE NR!
			basicph->basicPhase(nodeid, phaseid);
		}

	}
	std::cout << "basic phase file processed" << std::endl;
}

//ReadAndBuidPhases: Build all the possible phases over all junctions (out of the possible basicphases) by combining different movement groups for the different junctions.
//Preprocessing Functionality: The phases are written to a file, and can be read instead of build from there on.
//INPUT1: a map basicphases_ with phase objects with an attribute list of signalgroup objects belonging to each phase (same movement group is listed for each junction)
//INPUT2: OR preprocessed phases.txt file (see format OUTPUT2)
//OUTPUT0: a map phases_ with phase objects with an attribute list of the basic phases per junction that are combined to build the phase
//OUTPUT1: a map phases_ with phase objects with an attribute list of the signalgroup objects belonging to each phase (different movement groups for the different junctions)
//OUTPUT2: phasesOUT.txt file with all the possible phases of non-conflicting movements for the junctions in the format:
//[phaseid, basicphaseid, nodeid(1-nrofNodes), signalid(1-12)] with multiple lines per phaseid to specify the active signals/movements for the junctions/nodes 
void readAndBuildPhases(std::ifstream& phasefile, std::ofstream& phasefileOUT, networkPtr& net, optimizationSettings& settings, bool& buildPH)
{
	auto signalGroups = net->signalGroups();
	int phaseid, basicphaseid, sigid, nodeid;

	// initialize all red phase (not in file;)
	phasePtr ph = std::make_shared<phase>(0);
	net->phase(0, ph);
	int nrofPhases = 0;

	nodeid = 0;
	// if phasefile is empty, then file need to be generated based on basic phases
	while (phasefile >> phaseid >> basicphaseid >> nodeid >> sigid)
	{
		phasePtr ph = net->phases(phaseid);
		if (ph == nullptr) {
			ph = std::make_shared<phase>(phaseid);
			net->phase(phaseid, ph);
			nrofPhases = nrofPhases + 1;
			net->setNrofPhases(nrofPhases);
		}
		int sgnr = 12 * (nodeid - 1) + sigid;
		for (auto [sgID, sg] : signalGroups)
		{// each signalgroup
			if (sgnr == sg->getSgnr()) {
				ph->signalGroup(sgnr, sg);
			}
		}
		//ADD BASIC PHASE NR!
		ph->basicPhase(nodeid, basicphaseid);

	}
	if (nodeid > 0) { std::cout << "phase file processed" << std::endl; }
	if (nodeid > 0) { buildPH = false; }
	else { buildPH = true; }

	if (buildPH) {
		//check: if phasefile contains only basic phases, nodeid == 0, then build phases;
		//dump in txt file;
		int nrofBasicPhases = net->getNrofBasicPhases();
		int nrofJunctions = (int)net->junctions().size();
		int sizeindicesarray = nrofJunctions + 1;
		std::vector<int> indices(sizeindicesarray, 1);
		int idx = 1;
		int tmpNrofPhases = (int)pow(nrofBasicPhases, nrofJunctions);
		while (idx <= tmpNrofPhases)
		{
			//idx is phaseid;
			phasePtr ph = std::make_shared<phase>(idx);
			net->phase(idx, ph);
			nrofPhases = nrofPhases + 1;
			net->setNrofPhases(nrofPhases);
			for (int k = 1; k <= nrofJunctions; ++k) {
				for (auto [sgIDbasic, sgbasic] : net->basicPhases(indices[k])->signalGroups()) {
					int jncnam = std::stoi(sgbasic->jncname(), nullptr);
					if (jncnam == k) {//filter signal groups for correct junction!
						//print to phasesout.txt:
						int signam = std::stoi(sgbasic->name(), nullptr);
						phasefileOUT << idx << "\t" << indices[k] << "\t" << k << "\t" << signam << std::endl;
						//add to phase:
						ph->signalGroup(sgbasic->getSgnr(), sgbasic);
						ph->basicPhase(k, indices[k]);
					}
				}
			}

			//shift indices to next combi of basicphases
			bool tmpstop = false;
			int k = nrofJunctions;
			while ((k > 0) && (!tmpstop))
			{
				if (indices[k] < nrofBasicPhases) { indices[k] = indices[k] + 1; tmpstop = true; }
				else { indices[k] = 1; }
				k = k - 1;
			}
			idx = idx + 1;
		}
		std::cout << "phases automatically build" << std::endl;
	}
}

//ReadAndBuildPhaseTree: Build all the spiders of pre-defined possible orders of phases (structure-free: for each phase all phases are allowed next; cyclic control: same or only one predefined (basic)phase is allowed next) 
//Preprocessing Functionality: The possible order of phases is written to a file, and can be read instead of build from there on.
//INPUT1: a map phases_ with all possible phase objects (default structure free control: no limitations in order of phases)
//INPUT2: OR preprocessed decisiontree.txt file (see format OUTPUT2)
//OUTPUT1: a map phases_ with phase objects with an attribute list of phases that are allowed next (default structure-free control: full spider of next phases is allowed)
//OUTPUT2: decisiontreeOUT.txt file with all the possible phases of non-conflicting movements for the junctions in the format:
//[phaseid, nextphaseid] with multiple lines per phaseid to specify the nextphaseids
void readAndBuildPhaseTree(std::ifstream& treefile, std::ofstream& treefileOUT, networkPtr& net, optimizationSettings& settings, bool& buildPH)
{
	int phaseid, nextphaseid;
	int nrofPhases = net->getNrofPhases();

	if (buildPH) {
		//treefile needs to be build as well; combi of all phases as next phases
		for (int j = 1; j <= nrofPhases; ++j) {
			net->phases(0)->nextPhase(j, j);
			treefileOUT << 0 << "\t" << j << std::endl;
		}
		for (int i = 1; i <= nrofPhases; ++i) {
			//sorted
			for (int j = i; j <= nrofPhases; ++j) {
				net->phases(i)->nextPhase(j, j);
				treefileOUT << i << "\t" << j << std::endl;
			}
			for (int j = 1; j < i; ++j) {
				net->phases(i)->nextPhase(j, j);
				treefileOUT << i << "\t" << j << std::endl;
			}
		}
		std::cout << "tree automatically build" << std::endl;
	}
	else {
		//read tree file
		while (treefile >> phaseid >> nextphaseid)
		{
			net->phases(phaseid)->nextPhase(nextphaseid, nextphaseid);
		}
		std::cout << "tree file processed" << std::endl;
	}
}

//ReadVehicles: Read preprocessed vehicles with free flow arrival times / passage times at the stoplines of the dedicated links of the signal groups
//INPUT: vehiclesIN.txt file with preprocessed vehicles with free flow passage times at the signal groups in the format:
//[vehicleid, detectorid, detectorname, junctionname, signalgroupname, signalgroupidx, predicted (free-flow) passage time, lag passage time = adjusted predicted passage time (by delay previous junctions), real passage time, delay at this junction]
//OUTPUT: a map vehicles with vehicle objects with an attribute list of passage objects with the free flow passage times at the signal groups for each vehicle (also the order number of entrance of the vehicle is stored in the vehicle object)
//OUTPUT: a map controlIntervals with a decisionNode object per time interval with initialized statistics of the aggregated vehicle arrivals per time interval for the external signal groups (=exact demand)
void readVehicles(std::ifstream& vehIN, ControlIntervals& controlIntervals, optimizationSettings& settings, std::unordered_map<int, vehiclePtr>& vehicles, int& nrofSG)	
{
	int oid, dtid;
	std::string dtnam, sgnam, jncnam;
	double predtim, lagtim, realtim, deltim;
	int sgnr, maxsgnr = 0; //follows from maxnumber in file!

	std::unordered_map<int, int> qvehids; //temporary map to store the lastvehicle arriving at external signal group <sgnr, vehOID>

	while (vehIN >> oid >> dtid >> dtnam >> jncnam >> sgnam >> sgnr >> predtim >> lagtim >> realtim >> deltim)
	{
		vehiclePtr tmpveh = nullptr;

		//find vehicle in map
		auto it = vehicles.find(oid);
		if (it != vehicles.end()) {
			tmpveh = it->second;
		}
		else { //veh not in list
			tmpveh = std::make_shared<vehicle>(oid);
			vehicles[oid] = tmpveh;
		}
		int pasidx = tmpveh->getNrofPassages() + 1;
		passagePtr tmppas = std::make_shared<passage>(pasidx, dtid, dtnam, sgnam, jncnam, sgnr);
		tmppas->setPredTime(predtim);
		tmppas->setLagTime(lagtim); //initialized to predtim
		tmppas->setRealTime(realtim); //initialized to predtim
		tmppas->setDelay(deltim); //initialized to zero 
		tmpveh->passage(pasidx, tmppas);
		tmpveh->setNrofPassages(pasidx);

		//initialize arrivals; EXTERNAL ARRIVALS ONLY!!!->first passages
		if (pasidx == 1) {
			int k = (int)floor(predtim / settings.controlIntLngth + 1); //rounded downwards				
			double arr = controlIntervals.get(k)->arrivals(sgnr) + 1;
			controlIntervals.get(k)->arrival(sgnr, arr);
			controlIntervals.get(k)->arrivalLNK(sgnr, arr);

			//store individual vehicle order
			auto itsg = qvehids.find(sgnr);
			if (itsg != qvehids.end()) {
				vehiclePtr tmpqveh = vehicles[itsg->second];
				tmpqveh->setNextVehID(oid);
				tmpveh->setSeqNr(tmpqveh->getSeqNr() + 1);
				qvehids[sgnr] = oid; //store vehicle as last vehicle at the signal group						
			}
			else {
				qvehids[sgnr] = oid; //store vehicle as last vehicle at the signal group						
				tmpveh->setSeqNr(1); //vehicle is first vehicle
			}
		}
		if (sgnr > maxsgnr) { maxsgnr = sgnr; }
	}
	nrofSG = maxsgnr;

	//update cum arrivals
	for (int k = 0; k <= settings.nrofSimInt; ++k) {
		decisionNodePtr contrint = controlIntervals.get(k);
		double totarr = 0.0;
		double totcumarr = 0.0;
		for (int sgnr = 1; sgnr <= nrofSG; ++sgnr) {
			double arr = contrint->arrivals(sgnr); //if not defined, then zero, automatically?
			double cumarr = 0.0;
			if (k > 0) { cumarr = controlIntervals.get(k - 1)->cumarrivals(sgnr); } //if not defined, then zero, automatically?
			cumarr = cumarr + arr;
			contrint->cumarrival(sgnr, cumarr);
			contrint->cumarrivalLNK(sgnr, cumarr);
			totarr = totarr + arr;
			totcumarr = totcumarr + cumarr;
		}
		contrint->arrival(0, totarr);
		contrint->cumarrival(0, totcumarr);
		contrint->arrivalLNK(0, totarr);
		contrint->cumarrivalLNK(0, totcumarr);
	}
}

//WriteStatistics: Write the statistics (or predicted model output) of a decisionNode object from controlIntervals (or predecisionNodes/optimalNodes) to a file
//INPUT: a decisionNode object for a time interval with multiple arrays of statistical values per signal group for this time interval
//OUTPUT: a line per signalgroup in statistics.txt (or statisticspred.txt) file with the statistical values for this time interval with format:
//[useinterval(0/1), intervalidx, signalidx, signalstate(0/1), arrivalsLNK, cumarrivalsLNK, arrivals, cumarrivals, departures, cumdepartures, cumqueues, cumdelays, cumvehicles, cumtimes, head, tail, tmphead, tmptail, vehids(1..N)] (See for description of the statistics the decisionNode class in header file)
void writeStatistics(std::ofstream& statfile, networkPtr& net, int& tmpControlIntIdx, decisionNodePtr& tmpdn, int useint, std::unordered_map<int, vehiclePtr>& tmpvehicles, optimizationSettings& settings)
{
	auto signalGroups = net->signalGroups();
	
	//Additional print of statistics to compare complete prediction (reproduction of current decision interval + prediction horizon)
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();
		int internalsgnr = signalgr->getInternalSgnr();

		//CHECK: if number of vehicles in link is the same as in the individual vehicle positions list; (if ok: can be switched off!)
		if (signalgr->getInternalArr()) {
			int nrofveh = 0;
			int maxpos = tmpdn->getSizeVehids(signalgr->getInternalSgnr()) - 1;// 20;
			for (int tmppos = 1; tmppos <= maxpos; ++tmppos) {
				if (tmpdn->vehids(signalgr->getInternalSgnr(), tmppos) > 0) { nrofveh = nrofveh + 1; }
			}
			if ((nrofveh > 0) && (nrofveh != tmpdn->cumvehicles(sgnr))) { //should be ok now!
				std::cout << " ERROR STAT: number of vehicles not correct in individual vehicle list " << std::endl;
				std::cout << "interval" << tmpControlIntIdx << " sgnr " << sgnr << " #veh " << tmpdn->cumvehicles(sgnr) << " individuals " << nrofveh << std::endl;
				std::cout << std::endl;
			}
			if ((nrofveh > 0) && (tmpdn->leadvehids(sgnr) != tmpdn->vehids(signalgr->getInternalSgnr(), 1))) { //should be ok now!
				std::cout << " ERROR STAT: leading vehicle not correct in individual vehicle list " << std::endl;
				std::cout << "interval" << tmpControlIntIdx << " sgnr " << sgnr << " leadveh " << tmpdn->leadvehids(sgnr) << " firstveh " << tmpdn->vehids(signalgr->getInternalSgnr(), 1) << std::endl;
				std::cout << std::endl;
			}
		}
		//check passages through red (should always be corrected now!)
		if ((tmpdn->states(sgnr) == "RED") && (tmpdn->departures(sgnr) > 0)) {
			std::cout << " ERROR STAT: passages through RED " << std::endl;
			std::cout << "interval" << tmpControlIntIdx << " sgnr " << sgnr << " state " << tmpdn->states(sgnr) << " departures " << tmpdn->departures(sgnr) << std::endl;
			std::cout << std::endl;
		}

		int tmpstate = 0;
		if (tmpdn->states(sgnr) == "GRN") { tmpstate = 1; }
		statfile << std::setprecision(0) << std::fixed;
		statfile << useint << "\t" << (tmpControlIntIdx) << "\t" << sgnr << "\t" << tmpstate;

		statfile << std::setprecision(2) << std::fixed;
		statfile << "\t" << tmpdn->arrivalsLNK(sgnr) << "\t" << tmpdn->cumarrivalsLNK(sgnr) << "\t" << tmpdn->arrivals(sgnr) << "\t" << tmpdn->cumarrivals(sgnr)
			<< "\t" << tmpdn->departures(sgnr) << "\t" << tmpdn->cumdepartures(sgnr) << "\t" << tmpdn->cumqueues(sgnr)
			<< "\t" << tmpdn->cumdelays(sgnr) << "\t" << tmpdn->cumvehicles(sgnr) << "\t" << tmpdn->cumtimes(sgnr)
			<< "\t" << tmpdn->heads(sgnr) << "\t" << tmpdn->tails(sgnr) << "\t" << tmpdn->tmpheads(sgnr) << "\t" << tmpdn->tmptails(sgnr);

		statfile << std::setprecision(0) << std::fixed;
		if (signalgr->getInternalArr()) {
			statfile << "\t" << tmpdn->vehids(internalsgnr, 1) << "\t" << tmpdn->vehids(internalsgnr, 2) << "\t" << tmpdn->vehids(internalsgnr, 3) << "\t" << tmpdn->vehids(internalsgnr, 4) << "\t" << tmpdn->vehids(internalsgnr, 5) << "\t" << tmpdn->vehids(internalsgnr, 6)
				<< "\t" << tmpdn->vehids(internalsgnr, 7) << "\t" << tmpdn->vehids(internalsgnr, 8) << "\t" << tmpdn->vehids(internalsgnr, 9) << "\t" << tmpdn->vehids(internalsgnr, 10) << "\t" << tmpdn->vehids(internalsgnr, 11) << "\t" << tmpdn->vehids(internalsgnr, 12)
				<< "\t" << tmpdn->vehids(internalsgnr, 13) << "\t" << tmpdn->vehids(internalsgnr, 14) << "\t" << tmpdn->vehids(internalsgnr, 15) << "\t" << tmpdn->vehids(internalsgnr, 16) << "\t" << tmpdn->vehids(internalsgnr, 17) << "\t" << tmpdn->vehids(internalsgnr, 18)
				<< std::endl;
		}
		else {
			int tmpleadvehid = tmpdn->leadvehids(sgnr);
			for (int i = 1; i <= 18; ++i) {
				if ((tmpleadvehid > 0) && (i <= tmpdn->cumvehicles(sgnr))) { statfile << "\t" << tmpleadvehid; }
				else { statfile << "\t" << 0; }
				if (tmpleadvehid > 0) { tmpleadvehid = tmpvehicles[tmpleadvehid]->getNextVehID(); }
			}
			statfile << std::endl;
		}
	}
	statfile << std::setprecision(0) << std::fixed;
	statfile << useint << "\t" << (tmpControlIntIdx) << "\t" << 0 << "\t" << tmpdn->phase()->oid();
	statfile << std::setprecision(2) << std::fixed;
	statfile << "\t" << tmpdn->arrivalsLNK(0) << "\t" << tmpdn->cumarrivalsLNK(0) << "\t" << tmpdn->arrivals(0) << "\t" << tmpdn->cumarrivals(0) << "\t"
		<< tmpdn->departures(0) << "\t" << tmpdn->cumdepartures(0) << "\t" << tmpdn->cumqueues(0) << "\t"
		<< tmpdn->cumdelays(0) << "\t" << tmpdn->cumvehicles(0) << "\t" << tmpdn->cumtimes(0)
		<< "\t" << tmpdn->heads(0) << "\t" << tmpdn->tails(0) << "\t" << tmpdn->tmpheads(0) << "\t" << tmpdn->tmptails(0);
	statfile << std::setprecision(0) << std::fixed;
	for (int i = 1; i <= 18; ++i) {
		statfile << "\t" << 0;
	}
	statfile << std::endl;
}

//GenerateRandomDisturbances: Generates (randomly) the uncertainty model parameter scenarios for the upcoming prediction horizon (for robust control)
//INPUT: the average parameter values (mean) per signal group (stored in map signalGroups_)
//OUTPUT: randomParameters object with for each parameter type a map with per scenario, per timeinterval, per signal group, the (random) parameter value 
//NOTE: example code for random generation for all parameter types is available. For now the parameter scenarios for the turn fractions are defined by fixed values in splitfractions.txt file (See ReadSplitFractions) 
void GenerateRandomDisturbances(optimizationSettings& settings, int& localControlIntIdx, networkPtr& net, ControlIntervals& controlIntervals, randomParameters& randomParams, int randomstart, int scnrstart, int scnrend, int levidxstart, int levidxend, bool prepro)
{
	srand(randomstart);
	//rand() to generate random seeds for the separate random processes!
	//per parameter value; distribution with avg param in input files (mean disturbed values!); + standard deviation!!! 
	if (settings.nrofScenarios > 0) {
		if (settings.randomSaturationRates) {
			std::default_random_engine generatorRandomSatrates(rand()); //random seed for satrates 
			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				for (int levelidx = levidxstart; levelidx <= levidxend; ++levelidx) {
					for (auto [signalgrid, signalgr] : net->signalGroups()) {
						int sgnr = signalgr->getSgnr();
						//TYPE I (example code): modelparameter, example satrate (similar: speed (travtime), queuespeed, vehlength / linklength -> linkcapacity)
						double meansatrate = signalgr->getMeanSatrate();
						double stddevsatrate = signalgr->getStddevSatrate();
						std::normal_distribution<double> distributionRandomSatrates(meansatrate, stddevsatrate); //normal distribution with mean & standard deviation 						 
						//draw random values !!! ONLY IF SCNR > 0; SCNR==0 is the average case!
						double disturbedsatrate = meansatrate; 
						if (scnr > 0) { disturbedsatrate = distributionRandomSatrates(generatorRandomSatrates); }//random value drawn from distribution;
						//this works, however check on min/max values; or use truncated distributions...
						randomParams.setRandomSatrate(scnr, levelidx, sgnr, disturbedsatrate); //store for the upcoming decision sequence			
					}
				}
			}
		}
		if (settings.randomTravelTimes) {
			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				for (int levelidx = levidxstart; levelidx <= levidxend; ++levelidx) {
					for (auto [signalgrid, signalgr] : net->signalGroups()) {
						int sgnr = signalgr->getSgnr();
						////TYPE I (example code): modelparameter, example satrate (similar: speed (travtime), queuespeed, vehlength / linklength -> linkcapacity)
						int meannroftravint = signalgr->getMeanNrofTravIntervals();
						int stddevnroftravint = signalgr->getStddevNrofTravIntervals(); //not used yet: random generation by speed & distance???
						//draw random values !!! ONLY IF SCNR > 0; SCNR==0 is the average case!
						int disturbednroftravint = meannroftravint; 						
						//TODO random drawing (not used yet)
						randomParams.setRandomNrofTravInt(scnr, levelidx, sgnr, disturbednroftravint); //store for the upcoming decision sequence			
					}
				}
			}
		}
		if ((settings.randomState) && (prepro)) { //state disturbance only in preprocessing!!!
			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				for (auto [signalgrid, signalgr] : net->signalGroups()) {
					int sgnr = signalgr->getSgnr();
					int meanstateerror = signalgr->getMeanStateError();
					int stddevstateerror = signalgr->getStddevStateError();
					double cumQueue = controlIntervals.get(localControlIntIdx - 1)->cumqueues(sgnr); //queue state at localcontrintidx-1 == NOW
					double meanstate = cumQueue * (1.0 + meanstateerror / 100.0); //e.g. meanstateerror = 1%, then meanstate is queuevalue + 1%;					
					double stddevstate = cumQueue * (stddevstateerror / 100.0); //e.g. stddeverror=10%, then stddevstate is 10% of queuevalue
					//TODO draw random values from distribution with meanstate and stddevstate -> disturbed state!!! ONLY IF SCNR > 0; SCNR==0 is the average case!					 
					double disturbedstate = meanstate; 
					//TODO random drawing (not used yet)
					//calculate back to stateerror value
					int stateerror = 0;
					if (cumQueue > 0) { stateerror = (int)round(((disturbedstate / cumQueue) - 1.0) * 100.0); }
					randomParams.setRandomStateError(scnr, 0, sgnr, stateerror); //store always at zero time idx! Note levidxstart>=1!
				}
			}
		}
		if (settings.randomDemand) {
			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				for (int levelidx = levidxstart; levelidx <= levidxend; ++levelidx) {
					for (auto [signalgrid, signalgr] : net->signalGroups()) {
						int sgnr = signalgr->getSgnr();
						//TYPE II: demand error 
						//external signal groups only!
						if (!(signalgr->getInternalArr())) {
							int meandemanderror = signalgr->getMeanDemandError();
							int stddevdemanderror = signalgr->getStddevDemandError();
							double demand = 0.0;
							if (prepro) { demand = controlIntervals.get(localControlIntIdx - 1 + levelidx)->arrivals(sgnr); } //initialise to undisturbed value; NOW+decint, localControlIntIdx -1 +levidx
							else { demand = controlIntervals.get(localControlIntIdx - 1 + settings.nrofDecisionInt + levelidx)->arrivals(sgnr); } //initialise to undisturbed value; NOW+#decint+predint, localControlIntIdx -1 +#decint +levidx							
							double meandemand = demand * (1.0 + meandemanderror / 100.0); //e.g. meandemanderror = 1%, then meandemand is demandvalue + 1%;
							double stddevdemand = demand * (stddevdemanderror / 100.0); //e.g. stddeverror=10%, then stddevdemand is 10% of demandvalue
							//TODO draw random values from distribution with meandemand and stddevdemand -> disturbed demand!!! ONLY IF SCNR > 0; SCNR==0 is the average case!					 
							double disturbeddemand = meandemand; 
							//TODO random drawing (not used yet)
							//calculate back to demanderror value
							int demanderror = 0;
							if (demand > 0) { demanderror = (int)round(((disturbeddemand / demand) - 1.0) * 100.0); }
							randomParams.setRandomDemandError(scnr, levelidx, sgnr, demanderror); 
						}
					}
				}
			}
		}
		if (settings.randomFractions) {

			//update scenarioContainer with the active scenarios;
			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				bool scenarioactive = settings.scenarioActive[scnr];
				if (scenarioactive) { //add to container
					//if number of scenarios too large in container, then remove an element
					if (settings.scenarioContainer.size()>=settings.nrofScenariosSubset) {
						
						//check if new scenario already in container, then remove identical scenario otherwise remove last element
						int idx = -1;
						for (int i = 0; i < settings.scenarioContainer.size(); ++i) {
							if (settings.scenarioContainer[i] == scnr) { idx = i; }
						}
						if (idx >= 0) { // already in container: delete
							settings.scenarioContainer.erase(settings.scenarioContainer.begin()+idx);
						}
						else { //delete last element
							settings.scenarioContainer.erase(settings.scenarioContainer.end()-1);
						}
					}
					//add new scenario as first element
					settings.scenarioContainer.insert(settings.scenarioContainer.begin(),scnr); 
				}
			}

			for (int scnr = scnrstart; scnr <= scnrend; ++scnr) {
				for (int levelidx = levidxstart; levelidx <= levidxend; ++levelidx) {

					//set active scenarios; 
					//check if scenario number is in container; yes activate
					bool scenarioactive = false;
					for (int i = 0; i < settings.scenarioContainer.size(); ++i) {
						if (settings.scenarioContainer[i] == scnr) { scenarioactive = true; } 
					}
					randomParams.setScenarioActive(scnr, levelidx, scenarioactive);
					
					//set scenarioweights; 1/#scenarios per default (for randomly drawn scenarios); 
					double scenarioweight = 0.0;
					if (scnr > 0) { scenarioweight = 1.0 / settings.nrofScenarios;} //nominal scenario gets weight 0.0!				
					scenarioweight = settings.scenarioWeight[scnr]; //for now predefined scenarios with probabilities: read from settings and overwrite default (time & place independent for now)
					randomParams.setScenarioWeight(scnr, levelidx, scenarioweight);

					//check scenarioperiod //only if periods are defined
					bool scenarioperiod = true;
					if (settings.nrofScenarioPeriods > 0) {
						scenarioperiod = false;
						int tmptimeidx = 0;
						if (prepro) { tmptimeidx = localControlIntIdx - 1 + levelidx; }
						if (!prepro) { tmptimeidx = localControlIntIdx - 1 + settings.nrofDecisionInt + levelidx; }						
						for (int peridx = 1; peridx <= settings.nrofScenarioPeriods; ++peridx) {
							if ( (tmptimeidx > settings.scenarioPeriods[peridx][0]) && (tmptimeidx <= settings.scenarioPeriods[peridx][1]) ) {
								//model uncertainty scenarios are only active in this period 
								scenarioperiod = true;
							}
						}
					}

					//set random parameters
					for (auto [signalgrid, signalgr] : net->signalGroups()) {
						int sgnr = signalgr->getSgnr();

						//type III fractions
						for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {//all next signal groups with real fractions; //reading only!
							signalGroupPtr nextsg = net->signalGroups()[nextsgid]; //to signal group to store 
							int nextsgnr = nextsg->getSgnr();
							int disturbedfraction = frac; //outside scenario period the true fractions
							if (scenarioperiod) { //inside scenario period the mean/average (disturbed) fractions (scnr==0), and scenario realizations (scnr>0)
								int meanfraction = signalgr->meanNextSignalGroups()[nextsgid];
								int stddevfraction = signalgr->stddevNextSignalGroups()[nextsgid];
								//TODO draw random values (only main direction; adapt other directions?; ONLY IF SCNR > 0; SCNR==0 is the average case!				
								disturbedfraction = meanfraction; //for now average fraction, and scenario based fractions from file  
								if (scnr > 0) { disturbedfraction = signalgr->scenarioNextSignalGroups(scnr)[nextsgid]; }
								else { if (settings.scenarioLog > 0) { disturbedfraction = signalgr->scenarioNextSignalGroups(settings.scenarioLog)[nextsgid]; } }
							}
							randomParams.setRandomNextSignalGroup(scnr, levelidx, sgnr, nextsgid, disturbedfraction); 
							randomParams.setRandomPrevSignalGroup(scnr, levelidx, nextsgnr, signalgrid, disturbedfraction);  //mirroring!!!
						}
						if (signalgr->getExternalBoundary() > 0) {//external signal group
							for (auto [prevsgid, frac] : signalgr->prevSignalGroups()) {//all prev signal groups with real fractions from centroid!!!; //reading only!
								int disturbedfraction = frac; //outside scenario period the true fractions
								if (scenarioperiod) { //inside scenario period the mean/average (disturbed) fractions (scnr==0), and scenario realizations (scnr>0)
									int meanfraction = signalgr->meanPrevSignalGroups()[prevsgid];
									int stddevfraction = signalgr->stddevPrevSignalGroups()[prevsgid];
									//TODO draw random values (only main direction; adapt other directions?; ONLY IF SCNR > 0; SCNR==0 is the average case!
									disturbedfraction = meanfraction; //for now average fraction, and scenario based fractions from file 
									if (scnr > 0) { disturbedfraction = signalgr->scenarioPrevSignalGroups(scnr)[prevsgid]; }
									else { if (settings.scenarioLog > 0) { disturbedfraction = signalgr->scenarioPrevSignalGroups(settings.scenarioLog)[prevsgid]; } }
								}
								randomParams.setRandomPrevSignalGroup(scnr, levelidx, sgnr, prevsgid, disturbedfraction);								
							}
						}
					}
				}
			}
		}
	}
}

//CalculateIndividuals: Calculates the individual vehicle departures at a signal group for a time interval (based on the known aggregated number of departures)
//INPUT: Decicion node for a time interval with for each signal group the current (ordered!) list of queued vehicles present on the dedicated link. 
//OUTPUT: Decicion node for a time interval with for each signal group the updated (ordered!) list of queued vehicles present on the dedicated link.
int CalculateIndividuals(decisionNodePtr& nextdn, decisionNodePtr& mindn, optimizationSettings& settings, networkPtr& net, signalGroupPtr& signalgr, double& deltaDep, std::unordered_map<int, vehiclePtr>& tmpvehicles, bool spacerestrict, std::unordered_map<int, int>& tmpnextSignalGroups, bool disturbed)
{
	bool headtail = settings.headtail; //disturbed: take PRED model settings& parameters!
	if (!disturbed) { headtail = settings.headtailSTAT; } //not disturbed if function call from STAT: take SIM model settings & parameters!

	auto signalGroups = net->signalGroups();
	int sgnr = signalgr->getSgnr();

	deltaDep = floor(deltaDep); //assure integer value; not really necessary...
	for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) { tmpnextSignalGroups[nextsgid] = 0; }
	int n = 0; //number of vehicles that can depart from queue
	bool spacedownstream = true;

	while ((n < deltaDep) && (spacedownstream)) {//while <=deltaDep & space downstream!} 

		int tmpvehid = nextdn->leadvehids(sgnr);
		vehiclePtr tmpveh = tmpvehicles[tmpvehid]; //leading vehicle
		int passnr = 0;
		int nextsgnr = 0;
		
		signalGroupPtr nextsggr = nullptr;
		for (auto [tmppassnr, tmppass] : tmpveh->passages()) {
			if (tmppass->sgnr() == sgnr) {
				passnr = tmppassnr;
			}
		}

		if ((passnr > 0) && (passnr < tmpveh->getNrofPassages())) {//there is a next passage

			nextsgnr = tmpveh->passages()[passnr + 1]->sgnr();

			for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {
				if (signalGroups[nextsgid]->getSgnr() == nextsgnr) { nextsggr = signalGroups[nextsgid]; }
			}

			if ((nextsgnr > 0) && (nextsggr != nullptr)) {//pass vehicle if there is space

				if (spacerestrict) {					
					double deltaQueue = floor(nextsggr->getDistance() / nextsggr->getVehicleDistance()) - (mindn->cumvehicles(nextsgnr));
					if ((disturbed)) { deltaQueue = floor(nextsggr->getDisturbedDistance() / nextsggr->getDisturbedVehicleDistance()) - (mindn->cumvehicles(nextsgnr)); }
					if (headtail) { //possible disturbance of max cap
						deltaQueue = floor(nextsggr->getDistance() / nextsggr->getVehicleDistance()) - mindn->tails(nextsgnr) - (mindn->cumvehicles(nextsgnr) - mindn->cumqueues(nextsgnr));
						if ((disturbed)) { deltaQueue = floor(nextsggr->getDisturbedDistance() / nextsggr->getDisturbedVehicleDistance()) - mindn->tails(nextsgnr) - (mindn->cumvehicles(nextsgnr) - mindn->cumqueues(nextsgnr)); }
					}
					if (deltaQueue - tmpnextSignalGroups[nextsggr->oid()] < 1) { //no space
						spacedownstream = false;
					}
				}
				if (spacedownstream) {//there is space
					n = n + 1;//pass vehicle
					tmpnextSignalGroups[nextsggr->oid()] = tmpnextSignalGroups[nextsggr->oid()] + 1;//pass vehicle
					//transfer vehicle to next link
					int maxpos = nextdn->getSizeVehids(nextsggr->getInternalSgnr()) - 1; 
					int pos = 0;
					bool vehtransfered = false;
					while ((pos < maxpos) && (!vehtransfered)) {
						pos = pos + 1;
						if (nextdn->vehids(nextsggr->getInternalSgnr(), pos) == 0) {
							nextdn->vehid(nextsggr->getInternalSgnr(), pos, tmpvehid);
							if (nextdn->leadvehids(nextsgnr) == 0) { nextdn->leadvehid(nextsgnr, tmpvehid); } //leading vehicle if link was empty...
							vehtransfered = true;
						}
					}
				}
			}
		}
		else {
			//free outflow from network
			n = n + 1;//pass vehicle
		}

		//remove vehicle from own link; (if not blocked!)
		if (spacedownstream) {
			if (signalgr->getInternalArr()) {

				int maxpos = nextdn->getSizeVehids(signalgr->getInternalSgnr()) - 1; 
				int pos = 0;
				bool vehtransfered = false;
				while ((pos < maxpos) && (!vehtransfered)) {
					pos = pos + 1;
					nextdn->vehid(signalgr->getInternalSgnr(), pos, nextdn->vehids(signalgr->getInternalSgnr(), pos + 1));
					if (nextdn->vehids(signalgr->getInternalSgnr(), pos) == 0) { vehtransfered = true; }
				}
				nextdn->leadvehid(sgnr, nextdn->vehids(signalgr->getInternalSgnr(), 1));
			}
			else {//external
				nextdn->leadvehid(sgnr, tmpveh->getNextVehID());
			}
		}
	}
	return n;
}

//CalculateHeadTail: Calculates the head and tail position of the queue at a signal group for a time interval (based on known arrivals at and departures from the queue at the signal group for the time interval)
//INPUT: Current decision node (for the current timeidx) with for the signal group the current queue head, tmphead, tail, tmptail
//OUTPUT: Next decicion node (for the next timeidx) with for the signal group the calculated queue head, tmphead, tail, tmptail
void CalculateHeadTail(decisionNodePtr& nextdn, decisionNodePtr& mindn, optimizationSettings& settings, signalGroupPtr& signalgr, bool disturbed)
{
	double deltahead = round(signalgr->getQueuerate() * settings.controlIntLngth);
	if (disturbed) { deltahead = round(signalgr->getDisturbedQueuerate() * settings.controlIntLngth); }
	double tmpdeltahead = deltahead;

	int sgnr = signalgr->getSgnr();
	double nextCumQueue = nextdn->cumqueues(sgnr);

	//Keep track of leading headtail == first queue that is solving
	if (mindn->tails(sgnr) > 0) { //only if there was a queue

		//Update tail
		nextdn->tail(sgnr, mindn->tails(sgnr) + nextdn->arrivals(sgnr));

		//Update head
		double currhead = mindn->heads(sgnr);
		if (currhead > 0) { nextdn->head(sgnr, currhead + deltahead); } //4
		else { //currhead is zero; starts moving only if there is discharge from queue!
			if (nextdn->departures(sgnr) > 0) { nextdn->head(sgnr, currhead + deltahead / 2); } //2
		}
	}

	//Keep track of temporary headtail == last queue that is solving
	if ((nextdn->states(sgnr) == "RED") || ((nextdn->states(sgnr) == "GRN") && (nextCumQueue > 0) && (nextdn->departures(sgnr) <= 0))) { //RED ONLY (STAT UNCLEAR WHEN VEHICLE IS BLOCKED...)
		nextdn->tmphead(sgnr, 0);
		nextdn->tmptail(sgnr, nextCumQueue);
	}
	else { //nextdn->states(sgnr) == "GRN"
		if (mindn->tmptails(sgnr) > 0) { //only if there was a queue

			//Update tmptail
			nextdn->tmptail(sgnr, mindn->tmptails(sgnr) + nextdn->arrivals(sgnr));

			//Update tmphead
			double tmpcurrhead = mindn->tmpheads(sgnr);
			if (tmpcurrhead > 0) { nextdn->tmphead(sgnr, tmpcurrhead + tmpdeltahead); } //4
			else { //currhead is zero; starts moving only if there is discharge from queue!
				if (nextdn->departures(sgnr) > 0) { nextdn->tmphead(sgnr, tmpcurrhead + tmpdeltahead / 2); } //2
			}

		}
	}
	if ((nextdn->tmpheads(sgnr) >= nextdn->tmptails(sgnr)) || (nextCumQueue <= 0)) { //head meets tail before queue is dissolved; -> queue of driving vehicles is left;
		nextdn->tmphead(sgnr, 0);
		nextdn->tmptail(sgnr, 0);
	}

	// UPDATE leading headtail
	if (nextdn->heads(sgnr) >= nextdn->tails(sgnr)) {
		//nextCumQueue > 0: first queue is not dissolved yet -> multiple queues are there (speed shockwave "deltahead" > departures)
		//catch head and tail of last queue
		nextdn->head(sgnr, nextdn->tmpheads(sgnr));
		nextdn->tail(sgnr, nextdn->tmptails(sgnr));
	}
}

//CalculateStatistics: Evaluates a control decision for one time step by the store-and-forward prediction model with given parameter values (makes use of CalculateIndividuals and CalculateHeadTail)
//INPUT: Current decision node object (for the current timeidx) with multiple arrays of the current queue state statistics per signal group for this time interval
//OUTPUT: Next decision node object (for the next timeidx) with multiple arrays of the calculated queue state statistics per signal group for this time interval
//OUTPUT: Returns the cumulative costs objective value upto this time interval (to use in the control optimization process)
double CalculateStatistics(decisionNodePtr& nextdn, decisionNodePtr& mindn, optimizationSettings& settings, networkPtr& net, ControlIntervals& controlIntervals, int& localControlIntIdx, std::string logstr, std::unordered_map<int, decisionNodePtr>& preDecisionNodes, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, int recalc, std::unordered_map<int, vehiclePtr>& tmpvehicles, bool disturbed)
{
	//Calculate statistics from one node in decision tree (mindn) to next node in the decision tree (nextdn)
	//Or from one node in predecisionnodes to next node in predecisionnodes
	bool individualqueue = settings.individualqueue; //disturbed: take PRED model settings& parameters!
	bool headtail = settings.headtail; //disturbed: take PRED model settings& parameters!
	bool individualdemand = settings.individualdemand; //disturbed: take PRED model settings& parameters!
	if (!disturbed) { individualqueue = settings.individualqueueSTAT; headtail = settings.headtailSTAT; individualdemand = settings.individualdemandSTAT; } //not disturbed if function call from STAT: take SIM model settings & parameters!

	auto signalGroups = net->signalGroups();
	auto centroids = net->centroids();

	int currentTimeInt = mindn->timeint(); //time idx in the decision tree of mindn; 0 if statistics of prenodes are calculated
	int nextTimeInt = nextdn->timeint(); //time idx in the decision tree; 0 if statistics of prenodes are calculated
	
	int nrofPreDecisionNodes = (int) preDecisionNodes.size()-1; // time idx in prenodes of mindn; //store only node 1, nrofDecisionInt; //store also at zero position statistics NOW
	// If process is already in decision tree, then predecisionnodes are completely filled; preDecisionNodes.size()-1 == nrofdecisionint;
	if (nrofPreDecisionNodes < 0) { nrofPreDecisionNodes = 0; } //if statistics for controlint are calculated: predecision nodes is empty (or as if contains 0 position node)

	int currentControlIntIdx = localControlIntIdx - 1 + nrofPreDecisionNodes + currentTimeInt / settings.controlIntLngth;
	int nextControlIntIdx = currentControlIntIdx + 1;
	
	phasePtr currentPhase = mindn->phase();
	phasePtr nextPhase = nextdn->phase();

	//nextCumCost to be determined in this function: currentCumCost+delay!!!
	double currentCumCost = mindn->cumcost();
	double nextCumCost = 0.0;
	double nextCost = 0.0;

	//subtotals per junction;
	for (int jncnr = 0; jncnr <= net->junctions().size(); ++jncnr) {
		nextdn->cumjnccost(jncnr, mindn->cumjnccosts(jncnr));
	}

	// initialize values !!!
	// initialize state & duration if not in preprocessing (recalc==1)
	if (recalc < 1) {
		for (auto [signalgrid, signalgr] : signalGroups) {
			int sgnr = signalgr->getSgnr();
			nextdn->state(sgnr, "RED");
			nextdn->stateDuration(sgnr, -1);
		}
		for (auto [signalgrid, signalgr] : nextPhase->signalGroups()) {
			int sgnr = signalgr->getSgnr();
			nextdn->state(sgnr, "GRN");
			if (mindn->states(sgnr) == "GRN") { nextdn->stateDuration(sgnr, mindn->stateDurations(sgnr) + settings.controlIntLngth); }
			else { nextdn->stateDuration(sgnr, settings.controlIntLngth - settings.lossTime); }
		}
	}

	//Make sure all statistic fields in nextnode are reset 
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();
		int internalsgnr = signalgr->getInternalSgnr();
		nextdn->arrivalLNK(sgnr, 0.0);
		nextdn->cumarrivalLNK(sgnr, 0.0);
		nextdn->arrival(sgnr, 0.0); 
		nextdn->cumarrival(sgnr, 0.0);
		nextdn->departure(sgnr, 0.0);
		nextdn->cumdeparture(sgnr, 0.0);
		nextdn->cumqueue(sgnr, 0.0);
		nextdn->cumdelay(sgnr, 0.0);
		nextdn->cumvehicle(sgnr, 0.0);
		nextdn->cumtime(sgnr, 0.0);
		nextdn->head(sgnr, 0.0);
		nextdn->tail(sgnr, 0.0);
		nextdn->tmphead(sgnr, 0.0);
		nextdn->tmptail(sgnr, 0.0);
		
		if (individualqueue) { //initialize individual vehicle positions to previous node; 
			nextdn->leadvehid(sgnr, mindn->leadvehids(sgnr)); 
			if (signalgr->getInternalArr()) {
				int maxpos = nextdn->getSizeVehids(internalsgnr) - 1; 
				for (int pos = 0; pos <= maxpos; ++pos) {
					nextdn->vehid(internalsgnr, pos, mindn->vehids(internalsgnr, pos));
				}
			}
		}
	}
	nextdn->arrivalLNK(0, 0.0);
	nextdn->cumarrivalLNK(0, 0.0);
	nextdn->arrival(0, 0.0);
	nextdn->cumarrival(0, 0.0);
	nextdn->departure(0, 0.0);
	nextdn->cumdeparture(0, 0.0);
	nextdn->cumqueue(0, 0.0);
	nextdn->cumdelay(0, 0.0);
	nextdn->cumvehicle(0, 0.0);
	nextdn->cumtime(0, 0.0);
	nextdn->head(0, 0.0);
	nextdn->tail(0, 0.0);
	nextdn->tmphead(0, 0.0);
	nextdn->tmptail(0, 0.0);
	
	//UPDATE ARRIVALS IN QUEUE!
	double totDeltaArr = 0.0;
	double totCumArr = 0.0;
	for (auto [signalgrid, signalgr] : signalGroups) {
		
		int sgnr = signalgr->getSgnr();

		//external arrivals; deltaArr zero for internal arrivals
		double currentCumArr = mindn->cumarrivals(sgnr); //may be disturbed		
		double deltaArr = controlIntervals.get(nextControlIntIdx)->arrivals(sgnr); //initialise to undisturbed value;

		//external arrivals
		if (!(signalgr->getInternalArr())) {
			  
			if (!individualqueue) {
				//redistribute concerning fractions 
				double demand = 0.0; //sum entrance
				for (auto [tmpsgid, frac] : signalgr->prevSignalGroups()) {//all sggroups from same entrance;
					int tmpsgnr = signalGroups[tmpsgid]->getSgnr();
					demand = demand + controlIntervals.get(nextControlIntIdx)->arrivals(tmpsgnr);
				}

				if (!individualdemand) { //Prediction based on average demand
					int centroididx = signalgr->getExternalBoundary();
					demand = centroids[centroididx]->getDemand(nextControlIntIdx);
				}

				deltaArr = demand * signalgr->prevSignalGroups()[signalgrid] / 100;
			}

			if ((disturbed)) {
				//pick the right controlinterval //assume shift in time is the same for externals of same border!!!
				int shiftControlIntIdx = signalgr->getNrofTravIntervals() - signalgr->getDisturbedNrofTravIntervals();
				decisionNodePtr tmpCtrInt = controlIntervals.get(nextControlIntIdx + shiftControlIntIdx);
				deltaArr = tmpCtrInt->arrivals(sgnr);

				if (!individualqueue) {
					//redistribute concerning disturbed fractions 
					double demand = 0.0; //sum entrance
					for (auto [tmpsgid, frac] : signalgr->disturbedPrevSignalGroups()) {//all sggroups from same entrance;
						int tmpsgnr = signalGroups[tmpsgid]->getSgnr();
						demand = demand + tmpCtrInt->arrivals(tmpsgnr);
					}

					if (!individualdemand) { //Prediction based on average demand
						int centroididx = signalgr->getExternalBoundary();
						demand = centroids[centroididx]->getDemand(nextControlIntIdx + shiftControlIntIdx);
					}

					deltaArr = demand * signalgr->disturbedPrevSignalGroups()[signalgrid] / 100;
				}

				//disturb magnitude
				if (!individualqueue) { deltaArr = deltaArr * (1.0 + signalgr->getDemandError() / 100.0); }				
			}
		}

		//NETWORK FUNCTIONALITY: internal arrivals 
		if (signalgr->getInternalArr()) {
			deltaArr = 0.0;

			//use intermediate variable: arrivals in link -----> arrivals in queue
			double arrLNK = mindn->arrivalsLNK(sgnr);
			double cumarrLNK = mindn->cumarrivalsLNK(sgnr);

			int nrofint = signalgr->getNrofTravIntervals(); //already calculated when reading signalgroups // constant during simulation;
			if ((disturbed)) { nrofint = signalgr->getDisturbedNrofTravIntervals(); }

			//Access directly the right time interval of the departure flow;
			if (nrofint >= 2) {
				int maxTree = mindn->timeint() / settings.controlIntLngth;
				if (nrofint > maxTree) {
					int maxPreTree = nrofPreDecisionNodes + maxTree;
					if (nrofint > maxPreTree) {
						int maxCtrInt = localControlIntIdx + maxPreTree;
						if (nrofint > maxCtrInt) {
							//std::cout << logstr << "warning: departure interval out of scope! " << std::endl;
						}
						else { //maxPreTree<nrofint<maxCtrInt 
							auto prevControlInterval = controlIntervals.get(maxCtrInt - nrofint);
							arrLNK = prevControlInterval->arrivalsLNK(sgnr);
							cumarrLNK = prevControlInterval->cumarrivalsLNK(sgnr);
						}
					}
					else { //maxTree<nrofint<=maxPreTree  
						decisionNodePtr tmpdn = preDecisionNodes[maxPreTree + 1 - nrofint]; 
						arrLNK = tmpdn->arrivalsLNK(sgnr);
						cumarrLNK = tmpdn->cumarrivalsLNK(sgnr);
					}
				}
				else { //2<=nrofint<=maxTree
					decisionNodePtr tmpdn = mindn; //ini: 1 interval back in decision tree
					for (int n = 2; n <= nrofint; ++n) {//2 or more intervals back in decision tree
						tmpdn = decisionNodes[tmpdn->prevNodeID()];
					}
					arrLNK = tmpdn->arrivalsLNK(sgnr);
					cumarrLNK = tmpdn->cumarrivalsLNK(sgnr);
				}
			} //0<=nrofint<2 then tmpdn=mindn; default;

			deltaArr = arrLNK;
			deltaArr = cumarrLNK - currentCumArr;
			if (deltaArr < 0.000000001) { deltaArr = 0; } //Just to be sure: vehicles passed through red can miss the statistics...
			
		}
		double nextCumArr = currentCumArr + deltaArr; //may be disturbed
		nextdn->arrival(sgnr, deltaArr);
		nextdn->cumarrival(sgnr, nextCumArr);

		if (!(signalgr->getInternalArr())) {
			nextdn->arrivalLNK(sgnr, deltaArr);
			nextdn->cumarrivalLNK(sgnr, nextCumArr);
		}
		else { //internal link arrivals are initialized to zero
			nextdn->arrivalLNK(sgnr, 0);
			nextdn->cumarrivalLNK(sgnr, mindn->cumarrivalsLNK(sgnr));
		}

		//Update totals
		totDeltaArr = totDeltaArr + deltaArr;
		totCumArr = totCumArr + nextCumArr;
	}
	nextdn->arrival(0, totDeltaArr);
	nextdn->cumarrival(0, totCumArr);
	
	// UPDATE DEPARTURES
	double totDeltaDep = 0.0;
	double totCumDep = 0.0;
	for (auto [signalgrid, signalgr] : signalGroups) {

		int sgnr = signalgr->getSgnr();
		double currentCumDep = mindn->cumdepartures(sgnr);
		double currentCumQueue = mindn->cumqueues(sgnr);
		double deltaDep = 0.0; //RED	
		double deltaArr = nextdn->arrivals(sgnr);//result of previous loop;
		if (nextdn->states(sgnr) == "GRN") { //GRN

			if (currentCumQueue > 0) {//discharge from queue
				deltaDep = 0.0;

				double satrate = signalgr->getSatrate();
				if ((disturbed)) { satrate = signalgr->getDisturbedSatrate(); }

				deltaDep = settings.controlIntLngth * satrate * signalgr->getNrofLanes(); //3
				if (nextdn->stateDurations(sgnr) < 2 * settings.controlIntLngth) {
					deltaDep = (2.0 / 3.0) * deltaDep; //2
					if (nextdn->stateDurations(sgnr) < settings.controlIntLngth) {
						deltaDep = (1.0 / 2.0) * deltaDep; //1
					}
				}

				if (deltaDep > currentCumQueue + deltaArr) { //end of queue
					deltaDep = currentCumQueue + deltaArr;
				}
			}
			else {//no queue; what arrives can pass through ...//only exact when storing individual passages in decision nodes
				deltaDep = deltaArr;
				if (nextdn->stateDurations(sgnr) < settings.controlIntLngth) {
					if (deltaDep > ceil((1.0 / 2.0) * deltaDep)) { deltaDep = ceil((1.0 / 2.0) * deltaDep); } //1,2 -> 1; 3,4 -> 2;					
				} 
			}

			bool blocked = false;
			if (individualqueue) {
				std::unordered_map<int, int> tmpnextSignalGroups; //temporary next signal group "fractions" with number of vehicles (part of deltaDep) <nextsgid,#>
				int n = CalculateIndividuals(nextdn, mindn, settings, net, signalgr, deltaDep, tmpvehicles, true, tmpnextSignalGroups, disturbed);

				//macro quantities
				if ((n == 0) && (n < deltaDep)) {//stream is completely blocked
					blocked = true;
				}
				deltaDep = n;
				for (auto [nextsgid, frac] : tmpnextSignalGroups) {
					auto nextsggr = signalGroups[nextsgid];
					int nextsgnr = nextsggr->getSgnr();
					nextdn->arrivalLNK(nextsgnr, frac);
					nextdn->cumarrivalLNK(nextsgnr, mindn->cumarrivalsLNK(nextsgnr) + frac);
				}
			}
			else {
				//check if vehicles can be stored downstream... 
				std::unordered_map<int, int> tmpnextSignalGroups; //temporarily store nextsignalgroup fractions: disturbed or undisturbed
				if ((disturbed)) {
					for (auto [nextsgid, frac] : signalgr->disturbedNextSignalGroups()) {
						tmpnextSignalGroups[nextsgid] = frac;
					}
				}
				else {
					for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {
						tmpnextSignalGroups[nextsgid] = frac;
					}
				}
				double minDepFlowFrac = 1.0; //all can flow through			
				for (auto [nextsgid, frac] : tmpnextSignalGroups) {
					auto nextsggr = signalGroups[nextsgid];
					int nextsgnr = nextsggr->getSgnr();
					double deltaQueue = floor(nextsggr->getDistance() / nextsggr->getVehicleDistance()) - (mindn->cumvehicles(nextsgnr));
					if ((disturbed)) { deltaQueue = floor(nextsggr->getDisturbedDistance() / nextsggr->getDisturbedVehicleDistance()) - (mindn->cumvehicles(nextsgnr)); }
					if (headtail) {//possible disturbance of max cap
						deltaQueue = floor(nextsggr->getDistance() / nextsggr->getVehicleDistance()) - mindn->tails(nextsgnr) - (mindn->cumvehicles(nextsgnr) - mindn->cumqueues(nextsgnr));
						if ((disturbed)) { deltaQueue = floor(nextsggr->getDisturbedDistance() / nextsggr->getDisturbedVehicleDistance()) - mindn->tails(nextsgnr) - (mindn->cumvehicles(nextsgnr) - mindn->cumqueues(nextsgnr)); }
					}
					if (deltaQueue < 0.0001 - 0.000000001) { deltaQueue = 0.0; } //default: as if dep ==0.01 and frac == 1%;
					if (deltaQueue < ((frac/100.0)/100.0) - 0.000000001) { deltaQueue = 0.0; } //as if dep ==0.01 and frac == frac%;

					double depFlowFrac = 0.0; //no space if deltaQueue == 0;
					if (deltaQueue > 0.0) { //space
						if ((deltaDep > 0.0) && (frac > 0.0)) { depFlowFrac = deltaQueue / (deltaDep * (frac / 100.0)); }
						else { depFlowFrac = 1.0; }
					}
					if (depFlowFrac < minDepFlowFrac) { minDepFlowFrac = depFlowFrac; }
				}
				deltaDep = minDepFlowFrac * deltaDep;
				if ((minDepFlowFrac <= 0)) {//stream is completely blocked
					blocked = true;
				}
				// traverse departures to next link:
				for (auto [nextsgid, frac] : tmpnextSignalGroups) {
					auto nextsggr = signalGroups[nextsgid];
					int nextsgnr = nextsggr->getSgnr();
					nextdn->arrivalLNK(nextsgnr, deltaDep * (frac / 100.0));
					nextdn->cumarrivalLNK(nextsgnr, mindn->cumarrivalsLNK(nextsgnr) + deltaDep * (frac / 100.0));
				} 
			}
			if ((blocked) && (settings.blockSignals)) {//stream is completely blocked
				nextdn->state(sgnr, "RED");
				nextdn->stateDuration(sgnr, -1);
			}
		}
		
		double nextCumDep = currentCumDep + deltaDep; //to be stored in next decicion node

		// storage at next decision node;
		nextdn->departure(sgnr, deltaDep);
		nextdn->cumdeparture(sgnr, nextCumDep);
		
		//Update totals
		totDeltaDep = totDeltaDep + deltaDep;
		totCumDep = totCumDep + nextCumDep;

	}
	nextdn->departure(0, totDeltaDep);
	nextdn->cumdeparture(0, totCumDep);

	//Calculate objective: nrof veh in queue ->Delay; or nrof veh on link ->TTS
	double totCumQueue = 0.0;
	double totCumDelay = 0.0;
	double totCumVeh = 0.0;
	double totCumTime = 0.0;
	double totDeltaArrLNK = 0.0; //summation can be done from here
	double totCumArrLNK = 0.0; //summation can be done from here
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();

		double nextCumVeh = mindn->cumvehicles(sgnr) + nextdn->arrivalsLNK(sgnr) - nextdn->departures(sgnr);
		if (nextCumVeh < 0.000000001) { nextCumVeh = 0.0; } //rounding errors, small negative values
		nextdn->cumvehicle(sgnr, nextCumVeh);
		nextdn->cumtime(sgnr, mindn->cumtimes(sgnr) + nextCumVeh * settings.controlIntLngth);

		double nextCumQueue = mindn->cumqueues(sgnr) + nextdn->arrivals(sgnr) - nextdn->departures(sgnr);
		if (nextCumQueue < 0.000000001) { nextCumQueue = 0.0; } //rounding errors, small negative values
		nextdn->cumqueue(sgnr, nextCumQueue);
		nextdn->cumdelay(sgnr, mindn->cumdelays(sgnr) + nextCumQueue * settings.controlIntLngth);

		//Update headtail of queue
		if ((headtail)&&(signalgr->getInternalArr())) { //internal signal groups only
			CalculateHeadTail(nextdn, mindn, settings, signalgr, disturbed);
		}

		//Update totals
		totCumQueue = totCumQueue + nextCumQueue;
		totCumDelay = totCumDelay + nextdn->cumdelays(sgnr);
		totCumVeh = totCumVeh + nextCumVeh;
		totCumTime = totCumTime + nextdn->cumtimes(sgnr);
		totDeltaArrLNK = totDeltaArrLNK + nextdn->arrivalsLNK(sgnr);
		totCumArrLNK = totCumArrLNK + nextdn->cumarrivalsLNK(sgnr);

		//Update subtotals (only for active cost mode)
		int jncnrII = std::stoi(signalgr->jncname(), nullptr); //own junction
		int jncnrI = jncnrII; //upstream junction initialized to own junction
		//Split nextcumveh in I: vehicles at beginning of link (belonging to junction stat upstream) and II: vehicles at end of link (belonging to own junction)
		double nextCumVehI = 0;
		double nextCumVehII = nextCumVeh;
		if (signalgr->getInternalArr()) {
			nextCumVehII = mindn->cumarrivalsLNK(sgnr)-nextdn->cumdepartures(sgnr);
			nextCumVehI = nextCumVeh - nextCumVehII;
			//AND jncnrI is upstream junction
			for (auto [prevsgid, frac] : signalgr->prevSignalGroups()) {
				signalGroupPtr prevsggr = signalGroups[prevsgid];
				if (frac > 0) { jncnrI = std::stoi(prevsggr->jncname(), nullptr); }
			}		
		}
		double deltaCostII = nextCumVehII * settings.controlIntLngth; //ADDITIVE !!!
		double deltaCostI = nextCumVehI * settings.controlIntLngth; //ADDITIVE !!!
		if (settings.objdelay) {
			deltaCostII = nextCumQueue * settings.controlIntLngth; //delay //ADDITIVE
			deltaCostI = 0.0;
		}
		nextdn->cumjnccost(jncnrII, nextdn->cumjnccosts(jncnrII) + deltaCostII);
		nextdn->cumjnccost(jncnrI, nextdn->cumjnccosts(jncnrI) + deltaCostI);
		nextdn->cumjnccost(0, nextdn->cumjnccosts(0) + deltaCostII + deltaCostI); //should be equal to nextcumcost
			
	}
	nextdn->cumqueue(0, totCumQueue);
	nextdn->cumdelay(0, totCumDelay);
	nextdn->cumvehicle(0, totCumVeh);
	nextdn->cumtime(0, totCumTime);
	nextdn->arrivalLNK(0, totDeltaArrLNK);
	nextdn->cumarrivalLNK(0, totCumArrLNK);
	
	//SWITCH to TTS <> delay
	nextCost = totCumVeh * settings.controlIntLngth; //TTS
	if (settings.objdelay) {
		nextCost = totCumQueue * settings.controlIntLngth; //delay
	}
	//delay in the decision process	
	nextCumCost = currentCumCost + nextCost;

	//check:
	if (abs(nextdn->cumjnccosts(0) - nextCumCost)>0.0001) { //is ok now!
		std::cout << "ERROR: total junction costs not equal to total costs " << std::endl;
		std::cout << "interval " << nextControlIntIdx << " totjunc " << nextdn->cumjnccosts(0) << " tot " << nextCumCost << std::endl;	
		std::cout << std::endl;
	}

	nextdn->setCost(nextCost); //known!
	nextdn->setCumCost(nextCumCost); //known!

	return nextCumCost;
}

//ProcessDecision: Evaluates a candidate control decision sequence for a full time path (prediction horizon) by the store-and-forward prediction model for all parameter scenarios (makes use of CalculateStatistics)
//INPUT: a map with (shifted) start decisionNode objects per scenario (for the (shifted) start timeidx) with multiple arrays of the (shifted) initial queue state statistics per signal group for this (shifted) start time interval and scenario
//OUTPUT: a map with decisionNode objects per time interval and per scenario with multiple arrays of the calculated queue state statistics per signal group for this time interval and scenario
//OUTPUT: Returns the cumulative costs objective value, averaged/maximized/... over all scenarios, of the decision sequence for the full time path (to use in the control optimization process)
double ProcessDecision(std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, std::vector<int>& nodearray, optimizationSettings& settings, networkPtr& net, ControlIntervals& controlIntervals, int& localControlIntIdx, std::string logstr, std::unordered_map<int, std::unordered_map<int, decisionNodePtr>>& preDecisionNodes, std::unordered_map<int, vehiclePtr>& tmpvehicles, randomParameters& randomParams)
{
	//this function is made for GA algorithm!
	//initialize current node to startnode;
	nrofDecisionNodes = settings.nrofScenarios + 1; //startnode stored as first element in decisionNodes //supernode inclusief subnodes!
	//Note last supernodeid stored at decisionnodes[nrofdecisionnodes-nrofscenarios]=decisionnodes[1]!
	double supercumcosts = 0.0;
	for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
		int prevdnnr = nrofDecisionNodes - settings.nrofScenarios; //last supernode ID!
		decisionNodePtr prevdn = decisionNodes[prevdnnr]; 

		int nextPhasenr = nodearray[idx]; //decision to calculate
		phasePtr nextPhase = net->phases(nextPhasenr);
		int nextTimeInt = prevdn->timeint() + settings.controlIntLngth; //check seconds;
		//int nextLevelIdx = nextTimeInt / settings.controlIntLngth; //not necessary: equals idx!!!

		//Loop over all scenarios!
		double maxscnrcost = 0;
		int maxscnr = 0;
		double sumscnrcost = 0;
		double weightedsumscnrcost = 0;		
		for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
			int subprevdnnr = prevdn->subNodeIDs(scnr);
			decisionNodePtr subprevdn = decisionNodes[subprevdnnr];

			decisionNodePtr subnextdn{};
			nrofDecisionNodes = nrofDecisionNodes + 1;
			if (decisionNodes.size() <= nrofDecisionNodes) {
				subnextdn = std::make_shared<decisionNode>(nrofDecisionNodes, nextPhase, nextTimeInt, -1, -1, subprevdnnr);
				decisionNodes[nrofDecisionNodes] = subnextdn;
			}
			else {
				subnextdn = decisionNodes[nrofDecisionNodes];
				subnextdn->setPhase(nextPhase);
				subnextdn->setTimeInt(nextTimeInt);
				subnextdn->setCost(-1); //not known yet
				subnextdn->setCumCost(-1); //not known yet
				subnextdn->setPrevNodeID(subprevdnnr);
			}
			//define supernode structure
			if (scnr == 0) { //supernode
				for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) {
					subnextdn->subNodeID(tmpscnr, nrofDecisionNodes + tmpscnr);
				}
				subnextdn->setSuperCumCost(0.0); //initialize to zero;
				
			}
			else { //subnode
				for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) {
					subnextdn->subNodeID(tmpscnr, -1); //empty!
				}
				subnextdn->setSuperCumCost(0.0);
			}

			//override parameter values
			//before calculating statistics, set disturbed parameters for model calculation! (NOT TO LOCAL SIGNAL GROUPS!!!)
			if (settings.nrofScenarios > 0) {
				for (auto [signalgrid, signalgr] : net->signalGroups()) {
					int sgnr = signalgr->getSgnr();
					if (settings.randomSaturationRates) {
						signalgr->setDisturbedSatrate(randomParams.getRandomSatrate(scnr, idx, sgnr));
					}
					if (settings.randomTravelTimes) {
						signalgr->setDisturbedNrofTravIntervals(randomParams.getRandomNrofTravInt(scnr, idx, sgnr));
					}
					if (settings.randomDemand) {
						if (!(signalgr->getInternalArr())) { signalgr->setDemandError(randomParams.getRandomDemandError(scnr, idx, sgnr)); }
					}
					if (settings.randomFractions) {
						for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {//all next signal groups with real fractions; //reading only!
							signalgr->disturbedNextSignalGroup(nextsgid, randomParams.getRandomNextSignalGroup(scnr, idx, sgnr, nextsgid));
						}
						for (auto [prevsgid, frac] : signalgr->prevSignalGroups()) {//all prev signal groups with real fractions (also from centroid!!!); //reading only!
							signalgr->disturbedPrevSignalGroup(prevsgid, randomParams.getRandomPrevSignalGroup(scnr, idx, sgnr, prevsgid));
						}
					}
				}
			}
			double nextCumCost = CalculateStatistics(subnextdn, subprevdn, settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes[scnr], decisionNodes, nrofDecisionNodes, 0, tmpvehicles, true);
			sumscnrcost = sumscnrcost + nextCumCost; //remember sum for average cost calculation
			weightedsumscnrcost = weightedsumscnrcost + (randomParams.getScenarioWeight(scnr, idx) * nextCumCost);
			if (randomParams.getScenarioActive(scnr, idx)) {
				if (nextCumCost > maxscnrcost) { maxscnrcost = nextCumCost; maxscnr = scnr; } //remember max for worst-case	
			}
		}
		//set supernode costs 
		decisionNodePtr nextdn = decisionNodes[nrofDecisionNodes - settings.nrofScenarios]; //supernode!
		//CALCULATE COSTS AVERAGE OVER SCENARIOS; OR MAX COSTS; 
		supercumcosts = nextdn->cumcost(); //nominal scenario costs //default non-robust control with average (disturbed) parameter settings.
		if (settings.objrobust == 1) { supercumcosts = nextdn->cumcost(); } //one of the scenario's, f.e. nominal scenario with average (disturbed) model parameters;
		if (settings.objrobust == 2) { supercumcosts = decisionNodes[nextdn->subNodeIDs(settings.scenarioLog)]->cumcost(); } //one of the scenario's specified by scenarioLog; (note if nrofscenarios>0, scenarioLog==0 field)
		if (settings.objrobust == 3) { supercumcosts = maxscnrcost; } //worst-case cost
		if (settings.objrobust == 4) { supercumcosts = weightedsumscnrcost; } //average weighted cost, excluding nominal scenario (weight=0)
		if (settings.objrobust == 5) { supercumcosts = (sumscnrcost - nextdn->cumcost()) / (settings.nrofScenarios); } //average cost, excluding nominal scenario

		nextdn->setSuperCumCost(supercumcosts); //last supernode!
	}
	return supercumcosts; //last supernode costs at last time interval == path costs!
}

//PreProcessDecision: ReEvaluates the actual control decision sequence for a full time path (update horizon) by the store-and-forward prediction model for all parameter scenarios (makes use of CalculateStatistics) AND SHIFTS THE INITIAL STATE ACCORDINGLY
//INPUT: a map optimalNodes with decisionNode objects per time interval with the actual optimal control decision and predicted cumulative objective costs for this time interval (averaged/maximized/... over all scenarios)
//INPUT: a map optimalNodes with decisionNode objects per time interval with multiple arrays of the predicted queue state statistics per signal group for this time interval (and scenarioLOG scenario)
//INPUT: the controlInterval object (for the start timeidx) with multiple arrays of the initial queue state statistics per signal group for this start time interval
//OUTPUT: a map preDecisionNodes with decisionNode objects per time interval and per scenario with multiple arrays of the recalculated queue state statistics per signal group for this time interval and scenario
//OUTPUT: a map with shifted start decisionNode objects per scenario (for the (shifted) start timeidx) with multiple arrays of the (shifted) initial queue state statistics per signal group for this (shifted) start time interval and scenario (to use as starting point in the control optimization)
//OUTPUT: statisticspred.txt file lines with the earlier predicted state statistics AND the reevaluated state statistics of the actual control decision sequence (see WriteStatistics)
void PreProcessDecision(optimizationSettings& settings, std::unordered_map<int, decisionNodePtr>& optimalNodes, networkPtr& net, ControlIntervals& controlIntervals, std::mutex& optimalNodes_mutex, std::unordered_map<int, std::unordered_map<int, decisionNodePtr>>& preDecisionNodes, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, int& localControlIntIdx, std::ofstream& tmplogfile, std::ofstream& statpredfile, std::string logstr, std::unordered_map<int, vehiclePtr>& tmpvehicles, randomParameters& randomParams)
{
	std::cout << logstr << "preprocessing .... " << std::endl;

	auto signalGroups = net->signalGroups();

	std::unordered_map<int, decisionNodePtr> tmpOptNodes{}; //tmp structure to preserve the current active optimal solution for locally reading 
	{
		std::lock_guard<std::mutex> lock(optimalNodes_mutex);
		for (auto [tmpidx, tmpdn] : optimalNodes) {
			tmpOptNodes[tmpidx] = tmpdn;
		}
		optimalNodes.clear();
	}

	//NOTE: optimal nodes contains only the path of supernodes (with state voor nominal scenario (specified scenarioLOG number, stored at [0] for writing))
	//print previous solution to file (preserve statistics before these are overwritten in tmpoptnodes)
	//print to check predicted and actual control sequence!
	if (localControlIntIdx > settings.nrofDecisionInt) {
		for (int l = 1; l <= settings.nrofPredictionInt; ++l) { //print complete prediction horizon even if last part is not used;
			//print to compare predicted delay to realized delay: //print complete prediction horizon even if last part is not used;
			decisionNodePtr tmpdn = tmpOptNodes[l];
			int useint = 0; //integer 1=use to compare statistics;
			if (l <= settings.nrofDecisionInt) { useint = 1; }
			int tmpControlIntIdx = localControlIntIdx - 1 + l;
			writeStatistics(statpredfile, net, tmpControlIntIdx, tmpdn, useint, tmpvehicles, settings);
		}
	}
	
	// update the statistics: follow phases fom optimal nodes (or default: all red phases) and recalculate statistics!!! 
	// result is stored in preDecisionNodes [scnr:0-nrofScenarios][l:1-nrofDecisionInt]
	preDecisionNodes.clear(); //empty values from previous decision
	double cumdelay = 0.0;
	for (int l = 1; l <= settings.nrofDecisionInt; ++l) {//loop through current active decision sequence

		for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {//loop over scenarios

			decisionNodePtr prevpredn{};
			if (l == 1) {
				// initialize to the current state; //same for each scenario
				// initialize cumulatives now == localControlIntIdx-1
				auto prevControlInterval = controlIntervals.get(localControlIntIdx - 1);
				prevpredn = std::make_shared<decisionNode>(0, prevControlInterval->phase(), 0, 0, 0, -1); // in predecision nodes the time interval, cost, cumcost is 0 at time now; 
				for (int jncnr = 0; jncnr <= net->junctions().size(); ++jncnr) { prevpredn->cumjnccost(jncnr, 0.0); } //subtotals;

				preDecisionNodes[scnr][0] = prevpredn; // prevpredn is always in array!!!
				for (auto [signalgrid, signalgr] : signalGroups) {
					int sgnr = signalgr->getSgnr();
					int internalsgnr = signalgr->getInternalSgnr();
					prevpredn->state(sgnr, prevControlInterval->states(sgnr));
					prevpredn->stateDuration(sgnr, prevControlInterval->stateDurations(sgnr));
					prevpredn->arrivalLNK(sgnr, prevControlInterval->arrivalsLNK(sgnr));
					prevpredn->cumarrivalLNK(sgnr, prevControlInterval->cumarrivalsLNK(sgnr));
					prevpredn->arrival(sgnr, prevControlInterval->arrivals(sgnr)); //real statistics
					prevpredn->cumarrival(sgnr, prevControlInterval->cumarrivals(sgnr)); //real statistics
					prevpredn->departure(sgnr, prevControlInterval->departures(sgnr));
					prevpredn->cumdeparture(sgnr, prevControlInterval->cumdepartures(sgnr));
					prevpredn->cumqueue(sgnr, prevControlInterval->cumqueues(sgnr));
					prevpredn->cumdelay(sgnr, prevControlInterval->cumdelays(sgnr));
					prevpredn->cumvehicle(sgnr, prevControlInterval->cumvehicles(sgnr));
					prevpredn->cumtime(sgnr, prevControlInterval->cumtimes(sgnr));
					prevpredn->head(sgnr, prevControlInterval->heads(sgnr));
					prevpredn->tail(sgnr, prevControlInterval->tails(sgnr));
					prevpredn->tmphead(sgnr, prevControlInterval->tmpheads(sgnr));
					prevpredn->tmptail(sgnr, prevControlInterval->tmptails(sgnr));
					if (settings.individualqueue) { //initialize individual vehicle positions 
						prevpredn->leadvehid(sgnr, prevControlInterval->leadvehids(sgnr));
						if (signalgr->getInternalArr()) {
							int maxpos = prevpredn->getSizeVehids(internalsgnr) - 1; 
							for (int pos = 0; pos <= maxpos; ++pos) {
								prevpredn->vehid(internalsgnr, pos, prevControlInterval->vehids(internalsgnr, pos));
							}
						}
					}

					// Disturb the real queue of the current time interval == disturbance of the state; //different per scenario!
					// Keyword: DISTURBANCE
					double cumQueue = prevpredn->cumqueues(sgnr);
					if ((cumQueue > 0) && (!settings.individualqueue)) {
						//random state error, calculated only if nrofscenarios>0 & randomState==true, otherwise fixed state error from file;
						if ((settings.nrofScenarios > 0) && (settings.randomState)) {
							signalgr->setStateError(randomParams.getRandomStateError(scnr, 0, sgnr)); //only local, not needed in other functions... 
						}
						//fractional
						double distCumQueue = cumQueue * (1.0 + signalgr->getStateError() / 100.0);
						prevpredn->cumqueue(sgnr, distCumQueue);
						prevpredn->cumvehicle(sgnr, prevpredn->cumvehicles(sgnr) - cumQueue + distCumQueue); //disturbed nrof veh on link as well!
						//if disturbance need to be active for headtail: adapt head & tail of queue as well! head is updated automatically in CalcHeadTail
						prevpredn->tail(sgnr, prevpredn->tails(sgnr) - cumQueue + distCumQueue);
						prevpredn->tmptail(sgnr, prevpredn->tmptails(sgnr) - cumQueue + distCumQueue);
					}
				}
			}
			else { //l>1
				prevpredn = preDecisionNodes[scnr][l - 1]; //all statistics are there
			}

			decisionNodePtr predn;
			phasePtr prePhase = nullptr;
			if (tmpOptNodes.empty()) {
				prePhase = net->phases(0); // all red
				predn = std::make_shared<decisionNode>(l, prePhase, 0, 0, 0, -1); //costs are filled automatically

				// initialize values !!!
				for (auto [signalgrid, signalgr] : signalGroups) {
					int sgnr = signalgr->getSgnr();
					predn->state(sgnr, "RED");
					predn->stateDuration(sgnr, -1);
				}
			}
			else {
				decisionNodePtr tmpdn = tmpOptNodes[l];
				prePhase = tmpdn->phase();
				predn = std::make_shared<decisionNode>(l, prePhase, 0, 0, 0, -1);

				// initialize values !!!
				for (auto [signalgrid, signalgr] : signalGroups) {
					int sgnr = signalgr->getSgnr();
					predn->state(sgnr, tmpdn->states(sgnr));
					predn->stateDuration(sgnr, tmpdn->stateDurations(sgnr));
				}
			}

			//before calculating statistics, set disturbed parameters for model calculation! (NOT TO LOCAL SIGNAL GROUPS!!!)
			if (settings.nrofScenarios > 0) {
				for (auto [signalgrid, signalgr] : net->signalGroups()) {
					int sgnr = signalgr->getSgnr();
					if (settings.randomSaturationRates) {
						signalgr->setDisturbedSatrate(randomParams.getRandomSatrate(scnr, l, sgnr));
					}
					if (settings.randomTravelTimes) {
						signalgr->setDisturbedNrofTravIntervals(randomParams.getRandomNrofTravInt(scnr, l, sgnr));
					}
					if (settings.randomDemand) {
						if (!(signalgr->getInternalArr())) { signalgr->setDemandError(randomParams.getRandomDemandError(scnr, l, sgnr)); }
					}
					if (settings.randomFractions) {
						for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {//all next signal groups with real fractions; //reading only!
							signalgr->disturbedNextSignalGroup(nextsgid, randomParams.getRandomNextSignalGroup(scnr, l, sgnr, nextsgid));
						}
						for (auto [prevsgid, frac] : signalgr->prevSignalGroups()) {//all prev signal groups with real fractions (also from centroid!!!); //reading only!
							signalgr->disturbedPrevSignalGroup(prevsgid, randomParams.getRandomPrevSignalGroup(scnr, l, sgnr, prevsgid));
						}
					}
				}
			}

			std::unordered_map<int, decisionNodePtr> emptyDecisionNodes; //empty dummy list
			int emptyNrofDecisionNodes = 0;
			cumdelay = CalculateStatistics(predn, prevpredn, settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes[scnr], emptyDecisionNodes, emptyNrofDecisionNodes, 1, tmpvehicles, true); 

			preDecisionNodes[scnr][l] = predn; //store at the end; when statistics are already calculated! otherwise nrofPreDecisionNodes not ok!
		}
	}

	//Additional print of statistics to compare complete prediction (reproduction of current decision interval + prediction horizon)
	for (int l = 1; l <= settings.nrofDecisionInt; ++l) {//loop through current decision interval
		int tmpControlIntIdx = localControlIntIdx - 1 + l;
		int useint = 0; //integer 1=use to compare statistics;	
		decisionNodePtr tmpdn = preDecisionNodes[0][l]; //always print nominal scenario; (specified scenarioLOG number, stored at [0] for writing)
		writeStatistics(statpredfile, net, tmpControlIntIdx, tmpdn, useint, tmpvehicles, settings);
	}

	// initialize decision tree: initilization to the startDecisionNode; 
	// Select from the optimal nodes, the node that is the starting point for the next decision proces;
	// This will be the node after nrofDecisionIntervals; Note that nrofDecisionIntervals<=nrofPredictionIntervals
	// (make a supernode with subnodes as a hardcopy of the nodes in preDecisionNodes[scnr][settings.nrofDecisionInt] node)
	decisionNodePtr startDecisionNode{};
	phasePtr activePhase = nullptr;
	if (tmpOptNodes.empty()) {
		activePhase = net->phases(0); // all red
	}
	else {
		startDecisionNode = tmpOptNodes[settings.nrofDecisionInt];
		activePhase = startDecisionNode->phase();
	}
	decisionNodePtr dn{};
	nrofDecisionNodes = 0;
	
	//per scenario a decision node; scnr=0 is the supernode, with the nominal statistics
	//costs for optimization (cumcost, cumjnccost, supercumcost) is reinitialized to zero at start of the tree:
	for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {

		nrofDecisionNodes = nrofDecisionNodes + 1;

		if (decisionNodes.size() <= nrofDecisionNodes) {
			dn = std::make_shared<decisionNode>(nrofDecisionNodes, activePhase, 0, 0, 0, -1);	//reinitialize to startDecisionNode
			decisionNodes[nrofDecisionNodes] = dn;
		}
		else {
			dn = decisionNodes[nrofDecisionNodes];
			dn->setPhase(activePhase);
			dn->setTimeInt(0);
			dn->setCost(0.0);
			dn->setCumCost(0.0);
			dn->setPrevNodeID(-1);
		}
		// initialize values !!!
		for (int jncnr = 0; jncnr <= net->junctions().size(); ++jncnr) { dn->cumjnccost(jncnr, 0.0); } //subtotals;

		if (tmpOptNodes.empty()) {
			for (auto [signalgrid, signalgr] : signalGroups) {
				int sgnr = signalgr->getSgnr();
				dn->state(sgnr, "RED");
				dn->stateDuration(sgnr, -1);
			}
		}
		else {
			for (auto [signalgrid, signalgr] : signalGroups) {
				int sgnr = signalgr->getSgnr();
				dn->state(sgnr, startDecisionNode->states(sgnr));
				dn->stateDuration(sgnr, startDecisionNode->stateDurations(sgnr));
			}
		}

		//define supernode structure
		bool printpred = false;
		if (scnr==0) { //supernode
			for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) {
				dn->subNodeID(tmpscnr, nrofDecisionNodes + tmpscnr);
			}
			dn->setSuperCumCost(0.0); //initialize to zero;
			printpred = true; //only print nominal recalculation of statistics;
		}
		else { //subnode
			for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) {
				dn->subNodeID(tmpscnr,-1); //empty!
			}
			dn->setSuperCumCost(0.0);
		}

		//store updated statistics in startdecisionnode...->copy values from last predn to dn; l=nrofDecisionInt;
		decisionNodePtr predn = preDecisionNodes[scnr][settings.nrofDecisionInt];
		for (auto [signalgrid, signalgr] : signalGroups) {
			int sgnr = signalgr->getSgnr();
			int internalsgnr = signalgr->getInternalSgnr();
			dn->arrivalLNK(sgnr, predn->arrivalsLNK(sgnr));
			dn->cumarrivalLNK(sgnr, predn->cumarrivalsLNK(sgnr));
			dn->arrival(sgnr, predn->arrivals(sgnr));
			dn->cumarrival(sgnr, predn->cumarrivals(sgnr));
			dn->departure(sgnr, predn->departures(sgnr));
			dn->cumdeparture(sgnr, predn->cumdepartures(sgnr));
			dn->cumqueue(sgnr, predn->cumqueues(sgnr));
			dn->cumdelay(sgnr, predn->cumdelays(sgnr));
			dn->cumvehicle(sgnr, predn->cumvehicles(sgnr));
			dn->cumtime(sgnr, predn->cumtimes(sgnr));
			dn->head(sgnr, predn->heads(sgnr));
			dn->tail(sgnr, predn->tails(sgnr));
			dn->tmphead(sgnr, predn->tmpheads(sgnr));
			dn->tmptail(sgnr, predn->tmptails(sgnr));
			if (settings.individualqueue) { //initialize individual vehicle positions 
				dn->leadvehid(sgnr, predn->leadvehids(sgnr));
				if (signalgr->getInternalArr()) {
					int maxpos = dn->getSizeVehids(internalsgnr) - 1; //20;
					for (int pos = 0; pos <= maxpos; ++pos) {
						dn->vehid(internalsgnr, pos, predn->vehids(internalsgnr, pos));
					}
				}
			}
		}
		dn->arrivalLNK(0, predn->arrivalsLNK(0));
		dn->cumarrivalLNK(0, predn->cumarrivalsLNK(0));
		dn->arrival(0, predn->arrivals(0));
		dn->cumarrival(0, predn->cumarrivals(0));
		dn->departure(0, predn->departures(0));
		dn->cumdeparture(0, predn->cumdepartures(0));
		dn->cumqueue(0, predn->cumqueues(0));
		dn->cumdelay(0, predn->cumdelays(0));
		dn->cumvehicle(0, predn->cumvehicles(0));
		dn->cumtime(0, predn->cumtimes(0));
		dn->head(0, predn->heads(0));
		dn->tail(0, predn->tails(0));
		dn->tmphead(0, predn->tmpheads(0));
		dn->tmptail(0, predn->tmptails(0));
		//individual queue has no totals; 
	}

	//initialization
	tmpOptNodes.clear();

	std::cout << logstr << "preprocessing ready: start optimization loop " << std::endl;
}

//CopyOptimalDecision: Copies the optimal decision sequence for a full time path (prediction horizon) from the decision tree(BB)/pathset(GA) to the final (sub)optimal decision sequence for implementation
//INPUT: optimal path of decisionNode objects (in tree(BB) or in set (GA)) per timeinterval with the optimal control decision (and state statistics) and cumulative objective costs for this time interval (averaged/maximized/... over all scenarios)
//OUTPUT: a map optimalNodes with decisionNode objects per time interval with the optimal control decision and cumulative objective costs for this time interval (averaged/maximized/... over all scenarios)
//OUTPUT: a map optimalNodes with decisionNode objects per time interval with multiple arrays of the predicted queue state statistics per signal group for this time interval (and scenarioLOG scenario)
void CopyOptimalDecision(optimizationSettings& settings, networkPtr& net, std::unordered_map<int, decisionNodePtr>& optimalNodes, std::mutex& optimalNodes_mutex, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& optdnnr, int& localControlIntIdx, SYSTEMTIME& testtime, std::ofstream& tmplogfile)
{
	auto signalGroups = net->signalGroups();

	//print to file intermediate level costs to decide best intermediate bounding...
	GetSystemTime(&testtime);
	if (settings.activateMethod == 1) { tmplogfile << "BB:"; }
	if (settings.activateMethod == 2) { tmplogfile << "GA:"; }
	if (settings.activateMethod > 2) { tmplogfile << "XX:"; }
	tmplogfile << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << "\t";
	tmplogfile << localControlIntIdx << "\t";

	std::lock_guard lock(optimalNodes_mutex);
	optimalNodes.clear();
	int idx = optdnnr;
	int seqidx = settings.nrofPredictionInt;
	decisionNodePtr seqdn = decisionNodes[idx]; // optdn;
	int nrofOptimalNodes = 0;
	while (idx > 0)
	{
		seqdn = decisionNodes[idx];

		// make a copy of seqdn with the same properties: statistics are not needed (only for printing)
		decisionNodePtr tmpdn = std::make_shared<decisionNode>(seqdn->oid(), seqdn->phase(), seqdn->timeint(), seqdn->cost(), seqdn->cumcost(), seqdn->prevNodeID());

		//copy supernode info; subnodes not relevant! (and are not available anymore in optimal nodes!)
		tmpdn->setSuperCumCost(seqdn->getSuperCumCost()); //this is the objective costs!!!

		// info on junctions costs					
		for (int jncnr = 0; jncnr <= net->junctions().size(); ++jncnr) {
			tmpdn->cumjnccost(jncnr, seqdn->cumjnccosts(jncnr));
		}

		// copy also the other properties!
		for (auto [signalgrid, signalgr] : signalGroups) {
			int sgnr = signalgr->getSgnr();
			int internalsgnr = signalgr->getInternalSgnr();
			tmpdn->state(sgnr, seqdn->states(sgnr));
			tmpdn->stateDuration(sgnr, seqdn->stateDurations(sgnr));
			tmpdn->arrivalLNK(sgnr, seqdn->arrivalsLNK(sgnr));
			tmpdn->cumarrivalLNK(sgnr, seqdn->cumarrivalsLNK(sgnr));
			tmpdn->arrival(sgnr, seqdn->arrivals(sgnr));
			tmpdn->cumarrival(sgnr, seqdn->cumarrivals(sgnr));
			tmpdn->departure(sgnr, seqdn->departures(sgnr));
			tmpdn->cumdeparture(sgnr, seqdn->cumdepartures(sgnr));
			tmpdn->cumqueue(sgnr, seqdn->cumqueues(sgnr));
			tmpdn->cumdelay(sgnr, seqdn->cumdelays(sgnr));
			tmpdn->cumvehicle(sgnr, seqdn->cumvehicles(sgnr));
			tmpdn->cumtime(sgnr, seqdn->cumtimes(sgnr));
			tmpdn->head(sgnr, seqdn->heads(sgnr));
			tmpdn->tail(sgnr, seqdn->tails(sgnr));
			tmpdn->tmphead(sgnr, seqdn->tmpheads(sgnr));
			tmpdn->tmptail(sgnr, seqdn->tmptails(sgnr));
			if (settings.individualqueue) { //initialize individual vehicle positions 
				tmpdn->leadvehid(sgnr, seqdn->leadvehids(sgnr));
				if (signalgr->getInternalArr()) {
					int maxpos = tmpdn->getSizeVehids(internalsgnr) - 1; 
					for (int pos = 0; pos <= maxpos; ++pos) {
						tmpdn->vehid(internalsgnr, pos, seqdn->vehids(internalsgnr, pos));
					}
				}
			}
		}
		tmpdn->arrivalLNK(0, seqdn->arrivalsLNK(0));
		tmpdn->cumarrivalLNK(0, seqdn->cumarrivalsLNK(0));
		tmpdn->arrival(0, seqdn->arrivals(0));
		tmpdn->cumarrival(0, seqdn->cumarrivals(0));
		tmpdn->departure(0, seqdn->departures(0));
		tmpdn->cumdeparture(0, seqdn->cumdepartures(0));
		tmpdn->cumqueue(0, seqdn->cumqueues(0));
		tmpdn->cumdelay(0, seqdn->cumdelays(0));
		tmpdn->cumvehicle(0, seqdn->cumvehicles(0));
		tmpdn->cumtime(0, seqdn->cumtimes(0));
		tmpdn->head(0, seqdn->heads(0));
		tmpdn->tail(0, seqdn->tails(0));
		tmpdn->tmphead(0, seqdn->tmpheads(0));
		tmpdn->tmptail(0, seqdn->tmptails(0));
		//individual queue has no totals; 

		// add the copy to the optimal node list; if decision node values are overwritten, optimal node values are not!
		optimalNodes[seqidx] = tmpdn;

		idx = seqdn->prevNodeID();
		seqidx = seqidx - 1;
		nrofOptimalNodes = nrofOptimalNodes + 1;
		tmplogfile << seqdn->getSuperCumCost() << "\t(" << seqdn->phase()->oid() << ")\t"; //this is the overall objective costs!!!

	}
	tmplogfile << std::endl;

	//activate the max costs scenarios (used in the next decicion update) 
	//print to file costs of ACTIVE scenarios ordered from high costs to low costs
	if (settings.robustHeuristic) {
		
		//ini: false;
		for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
			settings.scenarioActive[scnr] = false;
		}
		
		//determine maxcost scenario
		double maxscenariocost = -1;
		int maxscenarionr = -1;
		int maxsubdnnr = -1;
		decisionNodePtr optdn = decisionNodes[optdnnr]; // optdn;
		for (int scnr = 1; scnr <= settings.nrofScenarios; ++scnr) { //0 is not included
			int suboptdnnr = optdn->subNodeIDs(scnr);
			decisionNodePtr suboptdn = decisionNodes[suboptdnnr]; //note for scnr==0, submindn=mindn!			
			if (suboptdn->cumcost() > maxscenariocost) { maxscenariocost = suboptdn->cumcost();  maxscenarionr = scnr; maxsubdnnr = suboptdnnr; }			
		}
		if ((maxscenarionr > 0)& (maxscenarionr <= settings.nrofScenarios)) { settings.scenarioActive[maxscenarionr] = true; } //activate max cost scenario
		
		//write max cost scenario (new memeber for container) & existing members in container; (order: new (first) ---> old (last in vector))
		for (int i = 0; i <= settings.scenarioContainer.size(); ++i) {//one more to write new scenario first
			int writescnr = 0;
			if (i==0) { writescnr = maxscenarionr; }
			else { writescnr = settings.scenarioContainer[i - 1]; }
			//write info for writescnr
			if (settings.activateMethod == 1) { tmplogfile << "BB:"; }
			if (settings.activateMethod == 2) { tmplogfile << "GA:"; }
			if (settings.activateMethod > 2) { tmplogfile << "XX:"; }
			tmplogfile << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << "\t";
			tmplogfile << writescnr << "\t"; //instead of tmplogfile << localControlIntIdx << "\t";
			int idx = optdn->subNodeIDs(writescnr);
			decisionNodePtr seqdn = decisionNodes[idx]; // optsubdn;
			while (idx > 0) {
				seqdn = decisionNodes[idx];
				tmplogfile << seqdn->cumcost() << "\t(" << seqdn->phase()->oid() << ")\t"; //these are the underlying scenario costs!!!
				idx = seqdn->prevNodeID();
			}
			tmplogfile << std::endl;
		}
	}
}

//MakeNewDecisionGA: Optimizes the decision sequence for the upcoming prediction horizon (starting in the (shifted!) initial state) by GeneticAlgorithm (GA) (makes use of GenerateRandomDisturbances, PreProcessDecision, ProcessDecision, CopyOptimalDecision)
//INPUT: the controlInterval object (for the start timeidx) with multiple arrays of the initial queue state statistics per signal group for this start time interval (Note:includes also demand info for the external signal groups)
//OUTPUT: a map optimalNodes with decisionNode objects per time interval with the optimal control decision and cumulative objective costs for this time interval (averaged/maximized/... over all scenarios)
//OUTPUT: tmplogfile.txt file with convergence information, i.e., the intermediate found suboptimal candidate control decision sequences and cumulative objective costs
void MakeNewDecisionGA(optimizationSettings& settings, std::unordered_map<int, decisionNodePtr>& optimalNodes, networkPtr& net, ControlIntervals& controlIntervals, std::mutex& optimalNodes_mutex, std::unordered_map<int, std::unordered_map<int, decisionNodePtr>>& preDecisionNodes, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, std::ofstream& tmplogfile, std::ofstream& statpredfile, threadswitchPtr& threadswitchSIM, std::string logstr, bool prepro, std::unordered_map<int, vehiclePtr>& tmpvehicles)
{
	auto localControlIntIdx = settings.controlIntIdx;

	// generate random parameter values for the upcoming time horizon 	
	randomParameters randomPreDecisionParams;
	GenerateRandomDisturbances(settings, localControlIntIdx, net, controlIntervals, randomPreDecisionParams, localControlIntIdx, 0, settings.nrofScenarios, 1, settings.nrofDecisionInt, true); 

	randomParameters randomDecisionParams;
	GenerateRandomDisturbances(settings, localControlIntIdx, net, controlIntervals, randomDecisionParams, localControlIntIdx, 0, settings.nrofScenarios, 1, settings.nrofPredictionInt, false); 

	SYSTEMTIME testtime;
	GetSystemTime(&testtime);
	tmplogfile << "GA:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;

	if (prepro) {
		std::cout << logstr << "start new decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		PreProcessDecision(settings, optimalNodes, net, controlIntervals, optimalNodes_mutex, preDecisionNodes, decisionNodes, nrofDecisionNodes, localControlIntIdx, tmplogfile, statpredfile, logstr,tmpvehicles, randomPreDecisionParams);
	}
	else {
		std::cout << logstr << "continue decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		std::cout << logstr << " WARNING: NOT SUPPORTED YET ! " << std::endl;
	}

	// dn is the staring point of all the possible paths 

	// main generator
	// same for each time the program runs; different for each function call;
	srand(localControlIntIdx); //rand() to generate random seeds for the separate random processes!

	int startMethod = 0; //0 RANDOM; 
	int generationMethod = 1; //1 UNIFORM; 0 FIXED POINT
	int nrofParents = 500; //even number!!!
	int mutationsize = 1;
	int tournamentsize = 4;
	int arrsizeTournamentPopulation = tournamentsize + 1;
	int arrsizeParents = nrofParents + 1;
	int nrofPopulation = 2 * nrofParents;
	int arrsizePopulation = nrofPopulation + 1;
	int nrofIntervals = settings.nrofPredictionInt;
	int arrsizeIntervals = nrofIntervals + 1;
	
	std::vector< std::vector<int> > population(arrsizePopulation, std::vector<int>(arrsizeIntervals, 0));  
	std::vector<double> populationDelay (arrsizePopulation, 0.0); //avg delay stored at 0 idx; 
	std::vector<int> populationPermutation (arrsizePopulation, 0);
	for (int popidx = 1; popidx <= nrofPopulation; ++popidx) { populationPermutation[popidx] = popidx; }
	
	std::vector< std::vector<int> > parents(arrsizeParents, std::vector<int>(arrsizeIntervals, 0)); 
	std::vector<double> parentsDelay (arrsizeParents, 0.0); //avg delay stored at 0 idx;
	std::vector<int> parentsPermutation (arrsizeParents, 0);
	for (int popidx = 1; popidx <= nrofParents; ++popidx) { parentsPermutation[popidx] = popidx; }

	std::vector<int> tournamentPopulation(arrsizeTournamentPopulation, 0);

	//find path with min delay
	double deltadelay = 999999;
	double avgdelay = 999999;
	double optdelay = 999999; // optimum solution found so far to give back to Aimsun
	int itnr = 0;
	int countmindelay = 0;

	while (!(threadswitchSIM->getAbortDecision())&&(deltadelay>1)&&(itnr<=100)) //or optavgdelay does not change anymore //or maximum it is exceeded
	{
		itnr = itnr + 1;

		double mindelay = 999999;
		double maxdelay = 0;
		double sumdelay = 0.0;
		int mindelayidx = -1;

		//clear population
		for (int popidx = 1; popidx <= nrofPopulation; ++popidx) {
			for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
				population[popidx][idx] = 0;
			}
			populationDelay[popidx] = 0.0;
		}

		//generate/update parents
		std::default_random_engine generatorRandomParents(rand());
		std::uniform_int_distribution<int> distributionRandomParents(1, net->getNrofPhases()); //uniform distribution between 1 and #phases

		if (itnr == 1) {
	
			// Generate a random population of parents (startMethod==0)	
			for (int popidx = 1; popidx <= nrofParents; ++popidx) {
				for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
					population[popidx][idx] = distributionRandomParents(generatorRandomParents); 
				}				
			}		
			
			// Calculate the delay for each of the paths		
			for (int popidx = 1; popidx <= nrofParents; ++popidx) {
				
				//Reinitialize decision tree
				nrofDecisionNodes = settings.nrofScenarios + 1;
				double delay = ProcessDecision(decisionNodes, nrofDecisionNodes, population[popidx], settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes, tmpvehicles, randomDecisionParams);
				//decisionNodes contains the pathS with statistics! REUSED
				populationDelay[popidx] = delay;

				// Calculate the optimal path found sofar in the parents 
				sumdelay = sumdelay + delay;
				if (delay < mindelay) { mindelay = delay; mindelayidx = popidx; }
				if (delay > maxdelay) { maxdelay = delay; }
				
			}
			deltadelay = abs(avgdelay - sumdelay / nrofParents);
			avgdelay = sumdelay / nrofParents;
			std::cout << logstr << " it " << itnr << " mindelay " << mindelay << " avgdelay " << avgdelay << " deltadelay " << deltadelay << std::endl;
		}
		else {
			// if iteration > 1:
			// Copy optimal population to parents (also their delay!)
			for (int popidx = 1; popidx <= nrofParents; ++popidx) {
				for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
					population[popidx][idx] = parents[popidx][idx];
				}
				double delay = parentsDelay[popidx];
				populationDelay[popidx] = delay;

				// Calculate the optimal path found sofar in the parents 
				sumdelay = sumdelay + delay;
				if (delay < mindelay) { mindelay = delay; mindelayidx = popidx; }
				if (delay > maxdelay) { maxdelay = delay; }
			}
			deltadelay = abs(avgdelay - sumdelay / nrofParents);
			avgdelay = sumdelay / nrofParents;
			std::cout << logstr << " it " << itnr << " mindelay " << mindelay << " avgdelay " << avgdelay << " deltadelay " << deltadelay << std::endl;
		}
		
		//check if the new set of parents contains a more optimal solution than optimum found so far;
		if (mindelay < optdelay) {
			std::cout << logstr << "decision sequence is optimal, optdelay " << mindelay << std::endl;
			
			optdelay = mindelay;
			int optdelayidx = mindelayidx; // idx to array sequence with optimal solution; this array need to be copied to optnodeseq
			//from array of phasesequence to path of decision nodes numbered from 1-nrofint;
			
			//Reinitialize decision tree (necessary to copy optimal decision sequence!) //only nominal!!!
			nrofDecisionNodes = settings.nrofScenarios + 1;
			double tmpdelay = ProcessDecision(decisionNodes, nrofDecisionNodes, population[optdelayidx], settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes, tmpvehicles, randomDecisionParams);
			//decisionNodes contains the pathS with statistics! REUSED
			
			std::cout << logstr << "CHECK decision sequence is optimal, optdelay " << tmpdelay << std::endl;

			//copy new optimal solution to optimal nodes
			// trace back optimal decision
			threadswitchSIM->setControlAvail(false);
			int optdnnr = nrofDecisionNodes - settings.nrofScenarios; //last added supernode to the tree! //nominal scenario with supercumcosts the average over all scenarios of the subnodes
			CopyOptimalDecision(settings, net, optimalNodes, optimalNodes_mutex, decisionNodes, optdnnr, localControlIntIdx, testtime, tmplogfile);
			threadswitchSIM->setControlAvail(true);
		}
		else {
			countmindelay = countmindelay + 1;
		}
		
		// generate random permutation size n
		std::shuffle(parentsPermutation.begin(), parentsPermutation.end(), std::default_random_engine(rand())); //change seed randomly;)
		// combine two subsequent parents to two childs

		std::default_random_engine generatorCrossOverPoint(rand());
		std::uniform_int_distribution<int> distributionCrossOverPoint(1, settings.nrofPredictionInt-1); //uniform distribution between 1 and #predictionint	
		int crossOverPoint;

		for (int popidx = 1; popidx <= nrofParents/2; ++popidx) {
			int popidx1 = popidx * 2 - 1;
			int popidx2 = popidx * 2;
			int parent1 = parentsPermutation[popidx1];
			if (parent1 == 0) { parent1 = parentsPermutation[0]; } //0 parent = 0 phase all red doet niet mee
			int parent2 = parentsPermutation[popidx2];
			if (parent2 == 0) { parent2 = parentsPermutation[0]; } //0 parent = 0 phase all red doet niet mee
			int child1 = nrofParents + (popidx * 2 - 1);
			int child2 = nrofParents + (popidx * 2);

			if (generationMethod == 1) { //UNIFORM
				for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
					crossOverPoint = distributionCrossOverPoint(generatorCrossOverPoint);
					if (crossOverPoint <= settings.nrofPredictionInt/2) {
						population[child1][idx] = population[parent1][idx];
						population[child2][idx] = population[parent2][idx];
					}
					else {
						population[child1][idx] = population[parent2][idx];
						population[child2][idx] = population[parent1][idx];
					}
				}
			}
			else {
				crossOverPoint = distributionCrossOverPoint(generatorCrossOverPoint);
				for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
					if (idx <= crossOverPoint) {
						population[child1][idx] = population[parent1][idx];
						population[child2][idx] = population[parent2][idx];
					}
					else {
						population[child1][idx] = population[parent2][idx];
						population[child2][idx] = population[parent1][idx];
					}

				}
			}
			//MUTATION???
			for (int i = 1; i <= mutationsize; ++i) {
				crossOverPoint = distributionCrossOverPoint(generatorCrossOverPoint);
				population[child1][crossOverPoint] = distributionRandomParents(generatorRandomParents);
				crossOverPoint = distributionCrossOverPoint(generatorCrossOverPoint);
				population[child2][crossOverPoint] = distributionRandomParents(generatorRandomParents);
			}

			// calculate and store delay for each of the childs
			// for each child (=array sequence of phases; calculate path statistics!)	

			//Reinitialize decision tree
			nrofDecisionNodes = settings.nrofScenarios + 1;
			double delay1 = ProcessDecision(decisionNodes, nrofDecisionNodes, population[child1], settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes, tmpvehicles, randomDecisionParams);
			//decisionNodes contains the pathS with statistics! REUSED			
			populationDelay[child1] = delay1;
			
			//Reinitialize decision tree
			nrofDecisionNodes = settings.nrofScenarios + 1;
			double delay2 = ProcessDecision(decisionNodes, nrofDecisionNodes, population[child2], settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes, tmpvehicles, randomDecisionParams);
			//decisionNodes contains the pathS with statistics! REUSED
			populationDelay[child2] = delay2;
		}

		// generate random permutation size 2n
		// select new optimal parents and store in new array (also copy their delay)		
		for (int i = 1; i <= tournamentsize/2; ++i) {		
			std::shuffle(populationPermutation.begin(), populationPermutation.end(), std::default_random_engine(rand())); //change seed randomly;)
			for (int j = 1; j <= nrofPopulation / tournamentsize; ++j) {
				double tournamentdelay = 999999;
				int popopt = -1;
				for (int k = 1; k <= tournamentsize; ++k) {
					int popidx = (j - 1)* tournamentsize + k;
					int pop = populationPermutation[popidx];
					if (pop == 0) { pop = populationPermutation[0]; }
					if (populationDelay[pop] < tournamentdelay) {
						popopt = pop;
						tournamentdelay = populationDelay[pop];
					}
				}
				int paridx = (i - 1) * nrofPopulation / tournamentsize + j;
				for (int idx = 1; idx <= settings.nrofPredictionInt; ++idx) {
					parents[paridx][idx] = population[popopt][idx];
					parentsDelay[paridx] = populationDelay[popopt];
				}
			}
		}
	}

	if (threadswitchSIM->getAbortDecision()) {
		GetSystemTime(&testtime);
		tmplogfile << "GA:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;

		std::cout << logstr << "aborted decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		threadswitchSIM->setAbortDecision(false);
	}
	else { //CONVERGENCE

		GetSystemTime(&testtime);
		tmplogfile << "GA:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;

		std::cout << logstr << "end new decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		threadswitchSIM->setDecision(false); //ready;)     
		threadswitchSIM->setControlAvail(true);
	}

}

//Branch: internal function of MakeNewDecision, i.e. branch functionality of the branch&bound method to optimize decision sequence
void Branch(optimizationSettings& settings, networkPtr& net, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, std::unordered_map<int, decisionNodePtr>& basicDecisionNodes, decisionNodePtr& mindn, phasePtr& nextPhase, int& nextdnnr, decisionNodePtr& nextdn, ControlIntervals& controlIntervals, int& localControlIntIdx, std::string logstr, std::unordered_map<int, decisionNodePtr>& preDecisionNodes)
{
	//Store delay already in decision node structure; if delay is too large decision node will be re-used (nrofdecision nodes is not increased)...
	//This to prevent copying from temporary data structure to decision node;
	auto signalGroups = net->signalGroups();
	int currentTimeInt = mindn->timeint();
	int nextTimeInt = currentTimeInt + settings.controlIntLngth;
	int mindnnr = mindn->oid();

	if (decisionNodes.size() <= nextdnnr) {
		nextdn = std::make_shared<decisionNode>(nextdnnr, nextPhase, nextTimeInt, -1, -1, mindnnr);
		decisionNodes[nextdnnr] = nextdn;
	}
	else {
		nextdn = decisionNodes[nextdnnr];
		nextdn->setPhase(nextPhase);
		nextdn->setTimeInt(nextTimeInt);
		nextdn->setCost(-1); //not known yet
		nextdn->setCumCost(-1); //not known yet
		nextdn->setPrevNodeID(mindnnr);
	}
	
	//replace by info on basicphases!
	nextdn->arrivalLNK(0, 0);
	nextdn->cumarrivalLNK(0, 0);
	nextdn->arrival(0, 0);
	nextdn->cumarrival(0, 0);
	nextdn->departure(0, 0);
	nextdn->cumdeparture(0, 0);
	nextdn->cumqueue(0, 0);
	nextdn->cumdelay(0, 0);
	nextdn->cumvehicle(0, 0);
	nextdn->cumtime(0, 0);
	nextdn->head(0, 0);
	nextdn->tail(0, 0);
	nextdn->tmphead(0, 0);
	nextdn->tmptail(0, 0);
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();
		int internalsgnr = signalgr->getInternalSgnr();
		int jncnr = std::stoi(signalgr->jncname(), nullptr);
		decisionNodePtr nextbasicdn = basicDecisionNodes[nextPhase->basicPhases()[jncnr]];

		//copy info on signal state
		nextdn->state(sgnr, nextbasicdn->states(sgnr));
		nextdn->stateDuration(sgnr, nextbasicdn->stateDurations(sgnr));

		//1:copy usable info from basic nodes
		nextdn->arrival(sgnr, nextbasicdn->arrivals(sgnr));
		nextdn->cumarrival(sgnr, nextbasicdn->cumarrivals(sgnr));
		nextdn->departure(sgnr, nextbasicdn->departures(sgnr));
		nextdn->cumdeparture(sgnr, nextbasicdn->cumdepartures(sgnr));
		nextdn->cumqueue(sgnr, nextbasicdn->cumqueues(sgnr));
		nextdn->cumdelay(sgnr, nextbasicdn->cumdelays(sgnr));
		nextdn->head(sgnr, nextbasicdn->heads(sgnr));
		nextdn->tail(sgnr, nextbasicdn->tails(sgnr));
		nextdn->tmphead(sgnr, nextbasicdn->tmpheads(sgnr));
		nextdn->tmptail(sgnr, nextbasicdn->tmptails(sgnr));
		if (settings.individualqueue) { //copy individual vehicle positions; without link arrived vehicles; == no arrivals at internal links;
			nextdn->leadvehid(sgnr, nextbasicdn->leadvehids(sgnr));  //ok for external; internal need to be updated after vehids are known
			if (signalgr->getInternalArr()) {
				int maxpos = nextdn->getSizeVehids(internalsgnr) - 1; 
				int deltapos =(int) nextbasicdn->arrivalsLNK(sgnr); //should be integers...
				for (int pos = 1; pos <= maxpos ; ++pos) {
					if (pos + deltapos > maxpos) { 
						nextdn->vehid(internalsgnr, pos, 0); 
					}
					else { 
						if (nextbasicdn->vehids(internalsgnr, pos + deltapos) == 0) {
							nextdn->vehid(internalsgnr, pos, 0);
						}
						else {
							nextdn->vehid(internalsgnr, pos, nextbasicdn->vehids(internalsgnr, pos));
						}
					}					
				}
				nextdn->leadvehid(sgnr, nextdn->vehids(internalsgnr,1));
			}
		}

		//2:initialize internal arrivals to zero; and copy external arrivals from basic nodes 
		if (signalgr->getInternalArr()) {
			nextdn->arrivalLNK(sgnr, 0); //default: zero link arrivals
			nextdn->cumarrivalLNK(sgnr, mindn->cumarrivalsLNK(sgnr));
			nextdn->cumvehicle(sgnr, mindn->cumvehicles(sgnr) + nextdn->arrivalsLNK(sgnr) - nextdn->departures(sgnr));
			if (nextdn->cumvehicles(sgnr) < 0.000000001) { nextdn->cumvehicle(sgnr, 0.0); } //rounding errors, small negative values
			nextdn->cumtime(sgnr, mindn->cumtimes(sgnr) + nextdn->cumvehicles(sgnr) * settings.controlIntLngth);
		}
		else {
			nextdn->arrivalLNK(sgnr, nextbasicdn->arrivalsLNK(sgnr));
			nextdn->cumarrivalLNK(sgnr, nextbasicdn->cumarrivalsLNK(sgnr));
			nextdn->cumvehicle(sgnr, nextbasicdn->cumvehicles(sgnr));
			nextdn->cumtime(sgnr, nextbasicdn->cumtimes(sgnr));	
		}
	}

	// set departures to the next links (transitions between junctions / basic phases)
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();
		int jncnr = std::stoi(signalgr->jncname(), nullptr);
		decisionNodePtr nextbasicdn = basicDecisionNodes[nextPhase->basicPhases()[jncnr]]; //used to determine the right inflows in downstream link!
		if (nextdn->states(sgnr) == "GRN") {
			// traverse departures to next link:
			for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) { //only used to determine next connections; fractions are not used;
				auto nextsggr = signalGroups[nextsgid];
				int nextsgnr = nextsggr->getSgnr();
				int nextinternalsgnr = nextsggr->getInternalSgnr();
				double arrnextLNK = nextbasicdn->arrivalsLNK(nextsgnr); //use arrivals next connection from basic nodes!

				if (settings.individualqueue) {
					//add individual passages to next signal group
					int maxpos = nextdn->getSizeVehids(nextinternalsgnr) - 1; 
					int deltapos = (int)nextbasicdn->arrivalsLNK(nextsgnr); //should be integer
					int pos = 0;
					bool vehcopied = false;
					int pos2 = 0;//initialize here for efficiency
					while ((pos < maxpos) && (!vehcopied)) {
						pos = pos + 1;
						if (nextbasicdn->vehids(nextinternalsgnr, pos + deltapos) == 0) {
							//copy tail
							for (int i = 0; i < deltapos; ++i) {
								int tmpvehid = nextbasicdn->vehids(nextinternalsgnr, pos + i);
								bool vehcopied2 = false;
								while ((pos2 < maxpos) && (!vehcopied2)) {
									pos2 = pos2 + 1;
									if (nextdn->vehids(nextinternalsgnr, pos2) == 0) {
										nextdn->vehid(nextinternalsgnr, pos2, tmpvehid);
										vehcopied2 = true;
									}
								}
							}
							vehcopied = true;
						}
					}
					nextdn->leadvehid(nextsgnr, nextdn->vehids(nextinternalsgnr, 1));
				}

				nextdn->arrivalLNK(nextsgnr, arrnextLNK);
				nextdn->cumarrivalLNK(nextsgnr, mindn->cumarrivalsLNK(nextsgnr) + arrnextLNK);
				nextdn->cumvehicle(nextsgnr, mindn->cumvehicles(nextsgnr) + nextdn->arrivalsLNK(nextsgnr) - nextdn->departures(nextsgnr));				
				if (nextdn->cumvehicles(nextsgnr) < 0.000000001) { nextdn->cumvehicle(nextsgnr, 0.0); } //rounding errors, small negative values
				nextdn->cumtime(nextsgnr, mindn->cumtimes(nextsgnr) + nextdn->cumvehicles(nextsgnr) * settings.controlIntLngth);
			}
		}
	}
	
	//update totals in separate loop! after depatures to next link are updated!
	for (auto [signalgrid, signalgr] : signalGroups) {
		int sgnr = signalgr->getSgnr();
		nextdn->arrival(0, nextdn->arrivals(0) + nextdn->arrivals(sgnr));
		nextdn->cumarrival(0, nextdn->cumarrivals(0) + nextdn->cumarrivals(sgnr));
		nextdn->departure(0, nextdn->departures(0) + nextdn->departures(sgnr));
		nextdn->cumdeparture(0, nextdn->cumdepartures(0) + nextdn->cumdepartures(sgnr));
		nextdn->cumqueue(0, nextdn->cumqueues(0) + nextdn->cumqueues(sgnr));
		nextdn->cumdelay(0, nextdn->cumdelays(0) + nextdn->cumdelays(sgnr));
		nextdn->arrivalLNK(0, nextdn->arrivalsLNK(0) + nextdn->arrivalsLNK(sgnr));
		nextdn->cumarrivalLNK(0, nextdn->cumarrivalsLNK(0) + nextdn->cumarrivalsLNK(sgnr));
		nextdn->cumvehicle(0, nextdn->cumvehicles(0) + nextdn->cumvehicles(sgnr));
		nextdn->cumtime(0, nextdn->cumtimes(0) + nextdn->cumtimes(sgnr));
		//head and tails donot have totals...
	}

	// info on junctions + totals					
	nextdn->cumjnccost(0, 0.0);
	for (int jncnr = 1; jncnr <= net->junctions().size(); ++jncnr) {
		decisionNodePtr nextbasicdn = basicDecisionNodes[nextPhase->basicPhases()[jncnr]];
		nextdn->cumjnccost(jncnr, nextbasicdn->cumjnccosts(jncnr));
		nextdn->cumjnccost(0, nextdn->cumjnccosts(0) + nextbasicdn->cumjnccosts(jncnr));
	}
	nextdn->setCumCost(nextdn->cumjnccosts(0));
	nextdn->setCost(nextdn->cumcost() - mindn->cumcost()); //to be complete...
}

//Bound: internal function of MakeNewDecision, i.e. bound functionality of the branch&bound method to optimize decision sequence
void Bound(optimizationSettings& settings, networkPtr& net, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, std::unordered_map<int, decisionNodePtr>& basicDecisionNodes, std::unordered_map<int, std::map<int, int>>& usedInMinDecisionNodes, std::unordered_map<int, std::map<int, std::map<int, int>>>& usedInMinDecisionNodesAll, decisionNodePtr& mindn, phasePtr& nextPhase, int& nextLevelIdx, int& nextdnnr, double& nextCumCost, double& optdelay, bool& endbound, bool& intermediatebound, bool& rejectstate, bool& storestate, bool& storestatetotal)
{
	//First: check if next phase needs to be kept, based on information in basic phases!
	//I nextCumCost < optdelay
	//II nextCumCost <= a*Cmin+b //heuristic: throw away too high intermediate solutions that will not likely lead to optimum solutions
	//III check state presence; //more expensive, but exact

	auto signalGroups = net->signalGroups();

	endbound = true; //bounded by I
	intermediatebound = false; //not bounded by II
	rejectstate = false; //not bounded by III
	storestate = false; //node is bounded by I, do not store
	storestatetotal = false; //node is bounded by I, do not store

	if (optdelay - nextCumCost > 0.000000001 ) { //(nextCumCost < optdelay)

		endbound = false; //not bounded by I
		storestate = true; //if no cmpdn available yet, store state for this phase
		storestatetotal = true; //if no cmpdn available yet, store state 

		int cmpdnnr = 0;
		int cmptotdnnr = 0;
		decisionNodePtr cmpdn = nullptr;
		decisionNodePtr cmptotdn = nullptr;
		if (settings.useTotalState) { 
			cmpdnnr = usedInMinDecisionNodes[0][nextLevelIdx]; 
			cmptotdnnr = usedInMinDecisionNodes[0][nextLevelIdx];
		}
		else { 
			cmpdnnr = usedInMinDecisionNodes[nextPhase->oid()][nextLevelIdx]; 
			cmptotdnnr = usedInMinDecisionNodes[0][nextLevelIdx];
		}

		if (cmptotdnnr > 0) {  //only continue if compare nodes are stored

			cmptotdn = decisionNodes[cmptotdnnr]; //best decision node so far for nextLevelIdx (greedy node, or even better)

			//intermediate bound is always on totcmpdn! phase independent!
			double a = 1.0 + (settings.boundpercentage - nextLevelIdx * ((double)settings.boundpercentage / settings.nrofPredictionInt)) / 100.0;
			double b = settings.boundabsolute - nextLevelIdx * ((double)settings.boundabsolute / settings.nrofPredictionInt);
			
			int boundCosta = (int)ceil(a * cmptotdn->getSuperCumCost() / settings.controlIntLngth) * settings.controlIntLngth; 
			int boundCostb = (int)ceil((cmptotdn->getSuperCumCost() + b) / settings.controlIntLngth) * settings.controlIntLngth; 

			int boundCost = boundCosta;
			if (boundCosta > boundCostb) { boundCost = boundCostb; }

			if (nextCumCost - boundCost > 0.000000001) { //(nextCumCost > boundCost)
				intermediatebound = true; // bounded by II			
				storestate = false;
				storestatetotal = false;
			}
			else {// nextcumcost <= boundCost //intermediatebound still false;
			
				if ((nextCumCost - cmptotdn->getSuperCumCost()) > -0.000000001) {//if (nextCumCost >= cmptotdn->getSuperCumCost()) 
					storestatetotal = false;
				}

				if (cmpdnnr > 0) { //only continue if compare nodes are stored
					cmpdn = decisionNodes[cmpdnnr]; //best decision node so far for nextLevelIdx (greedy node, or even better)
				
					if (nextdnnr != cmpdnnr) {//do not disgard node if it is the cmpdn!

						if ((nextCumCost - cmpdn->getSuperCumCost()) > -0.000000001) {//(nextCumCost >= getSuperCumCost())  
							storestate = false;
							storestatetotal = false;

							rejectstate = true;
							//check queues //if all queues in next decision node are larger then in same state node then reject the node
							for (auto [signalgrid, signalgr] : signalGroups) {
								int sgnr = signalgr->getSgnr();
								int jncnr = std::stoi(signalgr->jncname(), nullptr);
								decisionNodePtr nextbasicdn = basicDecisionNodes[nextPhase->basicPhases()[jncnr]];
								if ((cmpdn->cumqueues(sgnr) - nextbasicdn->cumqueues(sgnr)) > 0.000000001) { rejectstate = false; } //(nextbasicdn->cumqueues(sgnr) < cmpdn->cumqueues(sgnr))
							}
						}

						if ((!rejectstate) && (!storestate)) {
							//check also with other nodes in the tree? YES!!!
							decisionNodePtr tmpdn = nullptr;
							int nextphidx = nextPhase->oid();
							if (settings.useTotalState) { nextphidx = 0; }

							std::map<int, int>::iterator it = usedInMinDecisionNodesAll[nextphidx][nextLevelIdx].begin(); //list of nodes with minimum costs
							while ((!rejectstate) && (it != usedInMinDecisionNodesAll[nextphidx][nextLevelIdx].end())) {
								int tmpdnidx = it->first;
								int tmpdnnr = it->second;
								if (tmpdnnr > 0) {
									tmpdn = decisionNodes[tmpdnnr];
									if (nextdnnr != tmpdnnr) {
										if ((nextCumCost - tmpdn->getSuperCumCost()) > -0.000000001) { //(nextCumCost >= tmpdn->getSuperCumCost()) 
											rejectstate = true;
											//check queues //if all queues in next decision node are larger then in same state node then reject the node
											for (auto [signalgrid, signalgr] : signalGroups) {
												int sgnr = signalgr->getSgnr();
												int jncnr = std::stoi(signalgr->jncname(), nullptr);
												decisionNodePtr nextbasicdn = basicDecisionNodes[nextPhase->basicPhases()[jncnr]];
												if ((tmpdn->cumqueues(sgnr) - nextbasicdn->cumqueues(sgnr)) > 0.000000001) { rejectstate = false; } //(nextbasicdn->cumqueues(sgnr) < tmpdn->cumqueues(sgnr)) 
											}
										}
									}
								}
								it++;
							}
						}
					}
				}
			}
		}
	}
}

//MakeNewDecision: Optimizes the decision sequence for the upcoming prediction horizon (starting in the (shifted!) initial state) by Branch&Bound tree search method (BB) (makes use of GenerateRandomDisturbances, PreProcessDecision, CalculateStatistics, Branch, Bound, CopyOptimalDecision)
//INPUT: the controlInterval object (for the start timeidx) with multiple arrays of the initial queue state statistics per signal group for this start time interval (Note:includes also demand info for the external signal groups)
//OUTPUT: a map optimalNodes with decisionNode objects per time interval with the optimal control decision and cumulative objective costs for this time interval (averaged/maximized/... over all scenarios)
//OUTPUT: tmplogfile.txt file with convergence information, i.e., the intermediate found suboptimal candidate control decision sequences and cumulative objective costs
void MakeNewDecision(optimizationSettings& settings, std::unordered_map<int, decisionNodePtr>& optimalNodes, networkPtr& net, ControlIntervals& controlIntervals, std::mutex& optimalNodes_mutex, std::unordered_map<int, std::unordered_map<int, decisionNodePtr>>& preDecisionNodes, std::unordered_map<int, decisionNodePtr>& decisionNodes, int& nrofDecisionNodes, std::ofstream& tmplogfile, std::ofstream& statpredfile, threadswitchPtr& threadswitchSIM, std::string logstr, bool prepro, std::unordered_map<int, vehiclePtr>& tmpvehicles)
{
	auto localControlIntIdx = settings.controlIntIdx;

	// generate random parameter values for the upcoming time horizon 	
	randomParameters randomPreDecisionParams;
	GenerateRandomDisturbances(settings, localControlIntIdx, net, controlIntervals, randomPreDecisionParams, localControlIntIdx, 0, settings.nrofScenarios, 1, settings.nrofDecisionInt, true); 

	randomParameters randomDecisionParams;
	GenerateRandomDisturbances(settings, localControlIntIdx, net, controlIntervals, randomDecisionParams, localControlIntIdx, 0, settings.nrofScenarios, 1, settings.nrofPredictionInt, false); 
	
	SYSTEMTIME testtime;
	GetSystemTime(&testtime);
	tmplogfile << "BB:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;

	int nrofNodeEvaluations = 0;
	
	if (prepro) {

		std::cout << logstr << "start new decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		PreProcessDecision(settings, optimalNodes, net, controlIntervals, optimalNodes_mutex, preDecisionNodes, decisionNodes, nrofDecisionNodes, localControlIntIdx, tmplogfile, statpredfile, logstr, tmpvehicles, randomPreDecisionParams);

	}
	else {
		std::cout << logstr << "continue decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		std::cout << logstr << " WARNING: NOT SUPPORTED YET ! " << std::endl;
	}
	
	//Initialization
	std::unordered_map<int, std::map<double, std::map<int, decisionNodePtr>>> levelSearchNodes{};//ordered on dnnr as well (reproducability!)
	std::unordered_map<int, std::map<int, int>> usedInMinDecisionNodes{ }; // per phaseidx, for each levlidx, the index of the decision node with min delay
	std::unordered_map<int, std::map<int, std::map<int, int>>> usedInMinDecisionNodesAll{ }; // per phaseidx, for each levlidx, all the indices of the decision nodes in the tree 
	for (int phidx = 0; phidx <= net->getNrofPhases(); ++phidx) {
		for (int levelidx = 0; levelidx <= settings.nrofPredictionInt; ++levelidx) {
			usedInMinDecisionNodes[phidx][levelidx] = 0;
			usedInMinDecisionNodesAll[phidx][levelidx][0] = 0;
		}
	}
	
	levelSearchNodes.clear();
	levelSearchNodes[0][0][1] = decisionNodes[1]; //search list only contains supernodes (no subnodes!)

	if (settings.useTotalState) { usedInMinDecisionNodes[0][0] = 1; usedInMinDecisionNodesAll[0][0][1] = 1; } //1 node at levelidx 0; is always minimum over all phases;
	else {
		usedInMinDecisionNodes[decisionNodes[1]->phase()->oid()][0] = 1; usedInMinDecisionNodesAll[decisionNodes[1]->phase()->oid()][0][1] = 1;
		usedInMinDecisionNodes[0][0] = 1; usedInMinDecisionNodesAll[0][0][1] = 1;
	}
	int nrofPaths = 0;
	int nrofLevelSearchNodes = 1;
	int nrofDiscardedLevelSearchNodes = 0;
	int nrofSearchNodes = 1;
	double optdelay = 999999;
	int optdnnr = -1;
	decisionNodePtr optdn;

	//shortcut of the search proces; immediately a path to the search horizon;
	int greedydnnr = -1;
	decisionNodePtr greedydn;
	
	// List with basicDecisionNodes; Delay = sum delay junctions basic phases;
	std::unordered_map<int, std::unordered_map<int, decisionNodePtr>> basicDecisionNodes;
	std::unordered_map<int, std::unordered_map<int, decisionNodePtr>> tmpbasicDecisionNodes;

	std::unordered_map<int, std::unordered_map<int, double>> spidercosts; //reuse; always same size; #scenarios x #phases 0:nominal scenario
	std::unordered_map<int, double> superspidercosts; //reuse; always same size; #phases ; costs averaged over scenarios, averaged spidercosts

	int maxlevidx = -1;

	while ( (nrofSearchNodes>0) && !(threadswitchSIM->getAbortDecision()))   
	{
		//find node with min delay
		double mindelay = 999999;
		int mindnnr = -1;
		decisionNodePtr mindn;

		int levidx = -1; //keep track of the level
		
		if (greedydnnr > 0) {//first look at greedy shortcut
			mindn = greedydn;
			mindnnr = greedydnnr;
			mindelay = greedydn->getSuperCumCost(); 
			levidx = mindn->timeint() / settings.controlIntLngth;
		}
		else {
			// find the first level that is not empty
			for (int tmplevidx = settings.nrofPredictionInt; tmplevidx >= 0; --tmplevidx) {
				if (!(levelSearchNodes[tmplevidx].empty())) {
					levidx = tmplevidx;
				}
			}
			if (levidx >= 0) {
				std::map<double, std::map<int, decisionNodePtr>>::iterator it = levelSearchNodes[levidx].begin();
				mindelay = it->first; // minimum costs
				std::map<int, decisionNodePtr>::iterator it2 = it->second.begin(); //list of nodes with minimum costs
				mindnnr = it2->first;
				mindn = it2->second;
				nrofLevelSearchNodes = nrofLevelSearchNodes + 1;

				if (levidx > maxlevidx) {
					std::cout << logstr << "maxlevidx " << maxlevidx << " size level search list " << nrofLevelSearchNodes-1 << " discarded " << nrofDiscardedLevelSearchNodes << std::endl;
					maxlevidx = levidx;
					nrofLevelSearchNodes = 1; //reset count new level of search nodes!
					nrofDiscardedLevelSearchNodes = 0;
					std::cout << logstr << "maxlevidx " << maxlevidx << " size total search list " << nrofSearchNodes << std::endl;
				}		
			}
			else {//just to be sure: should not happen...
				std::cout << logstr << "error: level search lists are empty, total search list is not..." << levidx << std::endl;
			}	
		}

		// remove from search list						
		nrofSearchNodes = nrofSearchNodes - 1;
		if (levidx >= 0) { levelSearchNodes[levidx][mindelay].erase(mindnnr); }
		if (levelSearchNodes[levidx][mindelay].empty()) { levelSearchNodes[levidx].erase(mindelay); }

		//keep track of greedy decision
		greedydn = nullptr;
		greedydnnr = -1;
		// handle node
		phasePtr currentPhase = mindn->phase();
		int currentTimeInt = mindn->timeint();
		int currentLevelIdx = currentTimeInt / settings.controlIntLngth;
		double currentCumCost = mindn->getSuperCumCost(); 
		if (currentTimeInt + settings.controlIntLngth > settings.nrofPredictionInt * settings.controlIntLngth) //end of planning horizon: check optimum
		{
			nrofPaths = nrofPaths + 1; //keep track of number of paths in decision tree

			if ((optdelay - currentCumCost) > 0.000000001) { 
				std::cout << logstr << "decision node is optimal, optdelay " << currentCumCost << std::endl;
				
				optdn = mindn;
				optdnnr = mindnnr;
				optdelay = currentCumCost;

				//copy new optimal solution to optimal nodes
				// trace back optimal decision
				threadswitchSIM->setControlAvail(false);				
				CopyOptimalDecision(settings, net, optimalNodes, optimalNodes_mutex, decisionNodes, optdnnr, localControlIntIdx, testtime, tmplogfile);
				threadswitchSIM->setControlAvail(true);

			}
			else {
				std::cout << logstr << "decision node is not optimal, bounded by optimal delay " << currentCumCost << " >= " << optdelay << std::endl;
			}
		}
		else //no end of planning horizon yet: split node
		{
			//FIRST CHECK IF NODE CAN ALREADY BE THROWN AWAY
			bool endbound = true;
			bool intermediatebound = true;
			bool rejectstate = true;
			bool storestate = false;
			bool storestatetotal = false;
			
			for (int nextBasicPhasenr = 1; nextBasicPhasenr <= net->getNrofBasicPhases(); ++nextBasicPhasenr) { tmpbasicDecisionNodes[0][nextBasicPhasenr] = mindn; }				
			Bound(settings, net, decisionNodes, nrofDecisionNodes, tmpbasicDecisionNodes[0], usedInMinDecisionNodes, usedInMinDecisionNodesAll, mindn, currentPhase, currentLevelIdx, mindnnr, currentCumCost, optdelay, endbound, intermediatebound, rejectstate, storestate, storestatetotal);
			
			bool boundnode = true;
			if ((!endbound) && (!intermediatebound) && (!rejectstate)) { boundnode = false; }

			//stop if number of search nodes gets too large to explore in normal amount of time...
			if ((nrofLevelSearchNodes > settings.nrofLevelSearchNodes)&&(currentLevelIdx==maxlevidx)) { boundnode = true; nrofDiscardedLevelSearchNodes = nrofDiscardedLevelSearchNodes + 1; }

			if (!boundnode) //branch and bound
			{				
				// SPIDER: FOR ALL NEXT PHASES ARE THE ARRIVALS THE SAME... 
				// DEPARTURES NEED TO BE CALCULATED ONLY FOR A SELECTION OF PHASES...
				// SUBTOTALS i.e. intersection delays need to be stored...
				
				int nextTimeInt = currentTimeInt + settings.controlIntLngth;
				int nextLevelIdx = nextTimeInt / settings.controlIntLngth;

				// greedy optimal solution
				double optJunctionCost = 999999;
				int optJunctionPhase = 0; //all-red

				for (int nextBasicPhasenr = 1; nextBasicPhasenr<=net->getNrofBasicPhases(); ++nextBasicPhasenr)
				{
					phasePtr nextBasicPhase = net->basicPhases(nextBasicPhasenr);
					
					//Store delay of basicDecisionNodes in a separate node structure: not in decision tree! (for all scenarios)				
					for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {

						decisionNodePtr nextbasicdn{};
						int nextbasicdnnr = nextBasicPhasenr; //separate list per scenario; ID of nodes in each list the same, 1, ..#basicphases!
						int submindnnr = mindn->subNodeIDs(scnr);
						decisionNodePtr submindn = decisionNodes[submindnnr]; //note for scnr==0, submindn=mindn!

						if (basicDecisionNodes[scnr].size() <= nextbasicdnnr) {
							nextbasicdn = std::make_shared<decisionNode>(nextbasicdnnr, nextBasicPhase, nextTimeInt, -1, -1, submindnnr);
							basicDecisionNodes[scnr][nextbasicdnnr] = nextbasicdn;
						}
						else {
							nextbasicdn = basicDecisionNodes[scnr][nextbasicdnnr];
							nextbasicdn->setPhase(nextBasicPhase);
							nextbasicdn->setTimeInt(nextTimeInt);
							nextbasicdn->setCost(-1); //not known yet
							nextbasicdn->setCumCost(-1); //not known yet
							nextbasicdn->setPrevNodeID(submindnnr);
						}
						
						//delay in the decision process
						//before calculating statistics, set disturbed parameters for model calculation! (NOT TO LOCAL SIGNAL GROUPS!!!)
						if (settings.nrofScenarios > 0) {
							for (auto [signalgrid, signalgr] : net->signalGroups()) {
								int sgnr = signalgr->getSgnr();
								if (settings.randomSaturationRates) {
									signalgr->setDisturbedSatrate(randomDecisionParams.getRandomSatrate(scnr, nextLevelIdx, sgnr));
								}
								if (settings.randomTravelTimes) {
									signalgr->setDisturbedNrofTravIntervals(randomDecisionParams.getRandomNrofTravInt(scnr, nextLevelIdx, sgnr));
								}
								if (settings.randomDemand) {
									if (!(signalgr->getInternalArr())) { signalgr->setDemandError(randomDecisionParams.getRandomDemandError(scnr, nextLevelIdx, sgnr)); }
								}
								if (settings.randomFractions) {
									for (auto [nextsgid, frac] : signalgr->nextSignalGroups()) {//all next signal groups with real fractions; //reading only!
										signalgr->disturbedNextSignalGroup(nextsgid, randomDecisionParams.getRandomNextSignalGroup(scnr, nextLevelIdx, sgnr, nextsgid));
									}
									for (auto [prevsgid, frac] : signalgr->prevSignalGroups()) {//all prev signal groups with real fractions (also from centroid!!!); //reading only!
										signalgr->disturbedPrevSignalGroup(prevsgid, randomDecisionParams.getRandomPrevSignalGroup(scnr, nextLevelIdx, sgnr, prevsgid));
									}
								}
							}
						}
						double nextCumCost = CalculateStatistics(nextbasicdn, submindn, settings, net, controlIntervals, localControlIntIdx, logstr, preDecisionNodes[scnr], decisionNodes, nrofDecisionNodes, 0, tmpvehicles, true);
						nrofNodeEvaluations = nrofNodeEvaluations + 1;
					}				
				}

				//costs for complete spider (all phases) based on combination of basic nodes (for all scenarios)
				for (auto [nextPhaseid, nextPhasenr] : currentPhase->nextPhases()) {
					phasePtr nextPhase = net->phases(nextPhasenr);

					double maxscnrcost = 0;
					int maxscnr = 0;
					double sumscnrcost = 0;
					double weightedsumscnrcost = 0;
					for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {

						double nextCumCost = 0.0;
						for (int jncnr = 1; jncnr <= net->junctions().size(); ++jncnr) {
							decisionNodePtr nextbasicdn = basicDecisionNodes[scnr][nextPhase->basicPhases()[jncnr]];
							nextCumCost = nextCumCost + nextbasicdn->cumjnccosts(jncnr);
						}
						//STORE
						spidercosts[scnr][nextPhasenr] = nextCumCost;
						sumscnrcost = sumscnrcost + nextCumCost; //remember sum for average cost calculation
						weightedsumscnrcost = weightedsumscnrcost + (randomDecisionParams.getScenarioWeight(scnr, nextLevelIdx) * nextCumCost);
						if (randomDecisionParams.getScenarioActive(scnr, nextLevelIdx)) {
							if (nextCumCost > maxscnrcost) { maxscnrcost = nextCumCost; maxscnr = scnr; } //remember max for worst-case
						}
					}
					//CALCULATE COSTS AVERAGE OVER SCENARIOS; OR MAX COSTS; 
					double nextCumCost = 0.0;
					nextCumCost = spidercosts[0][nextPhasenr]; //default non-robust control with average (disturbed) parameter settings.
					if (settings.objrobust == 1) { nextCumCost = spidercosts[0][nextPhasenr]; } //one of the scenario's, f.e. nominal scenario with average (disturbed) model parameters;
					if (settings.objrobust == 2) { nextCumCost = spidercosts[settings.scenarioLog][nextPhasenr]; } //one of the scenario's specified by scenarioLog; (note if nrofscenarios>0, scenarioLog==0 field)
					if (settings.objrobust == 3) { nextCumCost = maxscnrcost; } //worst-case cost
					if (settings.objrobust == 4) { nextCumCost = weightedsumscnrcost; } //average weighted cost, excluding nominal scenario (weight=0)
					if (settings.objrobust == 5) { nextCumCost = (sumscnrcost-spidercosts[0][nextPhasenr]) / (settings.nrofScenarios); } //average cost, excluding nominal scenario
					
					superspidercosts[nextPhasenr] = nextCumCost; //store avg / max scenario costs per phase; to use later!

					//DETERMINE OPTIMAL PHASE!!!
					if ((optJunctionCost - nextCumCost) > 0.000000001) {
						optJunctionCost = nextCumCost;
						optJunctionPhase = nextPhasenr;
					}
				}
				
				//Check if greedy phase has to be added to the tree;
				int nextdnnr = nrofDecisionNodes + 1; //if node is kept, this will be its number...+subnumbers!!!
				phasePtr nextPhase = net->phases(optJunctionPhase);
				double nextCumCost = optJunctionCost; 
				
				bool endbound = true;
				bool intermediatebound = true;
				bool rejectstate = true;
				bool storestate = false; 
				bool storestatetotal = false;
				
				Bound(settings, net, decisionNodes, nrofDecisionNodes, basicDecisionNodes[0], usedInMinDecisionNodes, usedInMinDecisionNodesAll, mindn, nextPhase, nextLevelIdx, nextdnnr, nextCumCost, optdelay, endbound, intermediatebound, rejectstate, storestate, storestatetotal);
				
				bool boundnode = true;
				if ((!endbound) && (!intermediatebound) && (!rejectstate)) { boundnode = false; }
				
				bool boundspider = true;
				if ((!endbound) && (!intermediatebound)) { boundspider = false; }

				//Add optimum phase == greedy phase to the tree
				if (!boundnode) //branch and bound
				{
					decisionNodePtr nextdn{}; //supernode!

					//For each scenario! add supernode & subnodes!
					for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
						int subnextdnnr = nrofDecisionNodes + 1; //this is the node number! for scnr==0, subnextdnnr=nextdnnr!
						decisionNodePtr subnextdn{};
						int submindnnr = mindn->subNodeIDs(scnr);
						decisionNodePtr submindn = decisionNodes[submindnnr]; //note for scnr==0, submindn=mindn!
						Branch(settings, net, decisionNodes, nrofDecisionNodes, basicDecisionNodes[scnr], submindn, nextPhase, subnextdnnr, subnextdn, controlIntervals, localControlIntIdx, logstr, preDecisionNodes[scnr]);
						
						nrofNodeEvaluations = nrofNodeEvaluations + 1;
						nrofDecisionNodes = nrofDecisionNodes + 1; 

						for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) { subnextdn->subNodeID(tmpscnr, -1); subnextdn->setSuperCumCost(0.0); } //empty subnode structure to be sure!
						if (scnr == 0) { nextdn = decisionNodes[nextdnnr]; nextdn->setSuperCumCost(nextCumCost); } // preserve supernode; and override cumcost
						if (scnr >= 0) { nextdn->subNodeID(scnr, subnextdnnr); }//add subnumbers to supernode;
					}

					nrofSearchNodes = nrofSearchNodes + 1;

					//add to search list
					levelSearchNodes[nextLevelIdx][nextCumCost][nextdnnr] = nextdn;

					// update used decision nodes in basic nodes 
					if (storestate) {
						if (settings.useTotalState) {
							if (storestatetotal) {
								usedInMinDecisionNodes[0][nextLevelIdx] = nextdnnr;
							}
						}
						else {
							usedInMinDecisionNodes[nextdn->phase()->oid()][nextLevelIdx] = nextdnnr;
							//check total optimum
							if (storestatetotal) { usedInMinDecisionNodes[0][nextLevelIdx] = nextdnnr; }
						}
					}
					//always store nodes per level and phase
					if (settings.useTotalState) { usedInMinDecisionNodesAll[0][nextLevelIdx][nextdnnr] = nextdnnr; }
					else {
						usedInMinDecisionNodesAll[0][nextLevelIdx][nextdnnr] = nextdnnr;
						usedInMinDecisionNodesAll[nextdn->phase()->oid()][nextLevelIdx][nextdnnr] = nextdnnr;
					}

					if (greedydnnr == -1)
					{
						greedydn = nextdn;
						greedydnnr = nextdn->oid();
					}
				}
				if (!boundspider) {

					// INCLUDE SPIDER: IF GREEDY NODE IS ADDED CONSIDER SPIDER; IF GREEDY NODE IS REJECTED: REJECT SPIDER AS WELL!
					// EXCEPTION: IF GREEDY NODE IS REJECTED BASED ON STATE, STILL CONSIDER SPIDER NODES

					//main loop relevant phases are added to tree; compared to minimum phase in spider; & CROSSREF IN TREE!
					for (auto [nextPhaseid, nextPhasenr] : currentPhase->nextPhases())
					{
						int nextdnnr = nrofDecisionNodes + 1; //if node is kept, this will be its number...+subnumbers!
						phasePtr nextPhase = net->phases(nextPhasenr);

						bool phaseAllowed = true; // always true; structure-free control no cycles;

						if (nextLevelIdx > settings.nrofDecisionInt) { phaseAllowed = false; } //GREEDY TAIL

						// if phase is the greedy phase, then its is already in the tree and can be skipped!
						if (greedydnnr > 0) { if (nextPhase->oid() == greedydn->phase()->oid()) { phaseAllowed = false; } }

						if (phaseAllowed) //constraints
						{
							double nextCumCost = 0.0;
							
							//Use stored values!!!!
							nextCumCost = superspidercosts[nextPhasenr];  

							bool endbound = true;
							bool intermediatebound = true;
							bool rejectstate = true;
							bool storestate = false;
							bool storestatetotal = false;

							Bound(settings, net, decisionNodes, nrofDecisionNodes, basicDecisionNodes[0], usedInMinDecisionNodes, usedInMinDecisionNodesAll, mindn, nextPhase, nextLevelIdx, nextdnnr, nextCumCost, optdelay, endbound, intermediatebound, rejectstate, storestate, storestatetotal);

							bool boundnode = true;
							if ((!endbound) && (!intermediatebound) && (!rejectstate)) { boundnode = false; }

							//Only continue if node need to be kept: branch!
							if (!boundnode) //branch and bound
							{
								decisionNodePtr nextdn{};

								//For each scenario! Add supernode & subnodes!
								for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
									int subnextdnnr = nrofDecisionNodes + 1; //this is the node number! for scnr==0, subnextdnnr=nextdnnr!
									decisionNodePtr subnextdn{};
									int submindnnr = mindn->subNodeIDs(scnr);
									decisionNodePtr submindn = decisionNodes[submindnnr]; //note for scnr==0, submindn=mindn!
									Branch(settings, net, decisionNodes, nrofDecisionNodes, basicDecisionNodes[scnr], submindn, nextPhase, subnextdnnr, subnextdn, controlIntervals, localControlIntIdx, logstr, preDecisionNodes[scnr]);

									nrofNodeEvaluations = nrofNodeEvaluations + 1;
									nrofDecisionNodes = nrofDecisionNodes + 1; 

									for (int tmpscnr = 0; tmpscnr <= settings.nrofScenarios; ++tmpscnr) { subnextdn->subNodeID(tmpscnr, -1); subnextdn->setSuperCumCost(0.0); } //empty subnode structure to be sure!
									if (scnr == 0) { nextdn = decisionNodes[nextdnnr]; nextdn->setSuperCumCost(nextCumCost); } // preserve supernode and override cumcost!
									if (scnr >= 0) { nextdn->subNodeID(scnr, subnextdnnr); }//add subnumbers to supernode;
								}

								nrofSearchNodes = nrofSearchNodes + 1;

								//add to search list
								levelSearchNodes[nextLevelIdx][nextCumCost][nextdnnr] = nextdn;

								// update used decision nodes in basic nodes 
								if (storestate) {
									if (settings.useTotalState) {
										if (storestatetotal) {
											usedInMinDecisionNodes[0][nextLevelIdx] = nextdnnr;
										}
									}
									else {
										usedInMinDecisionNodes[nextdn->phase()->oid()][nextLevelIdx] = nextdnnr;
										//check total optimum
										if (storestatetotal) { usedInMinDecisionNodes[0][nextLevelIdx] = nextdnnr; }
									}
								}
								//always store nodes per level and phase
								if (settings.useTotalState) { usedInMinDecisionNodesAll[0][nextLevelIdx][nextdnnr] = nextdnnr; }
								else {
									usedInMinDecisionNodesAll[0][nextLevelIdx][nextdnnr] = nextdnnr;
									usedInMinDecisionNodesAll[nextdn->phase()->oid()][nextLevelIdx][nextdnnr] = nextdnnr;
								}
							}
						}
					}
				}
			}
		}
	}

	if (threadswitchSIM->getAbortDecision()) {
		GetSystemTime(&testtime);
		tmplogfile << "BB:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;
		std::cout << logstr << "aborted decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		std::cout << logstr << "nrofSearchNodes " << nrofSearchNodes << " nrofDecisionNodes " << nrofDecisionNodes << " nrofNodeEvaluations " << nrofNodeEvaluations << std::endl;
		threadswitchSIM->setAbortDecision(false);
	}
	else { //search list is empty, optimum reached

		GetSystemTime(&testtime);
		tmplogfile << "BB:" << testtime.wHour << ":" << testtime.wMinute << ":" << testtime.wSecond << ":" << testtime.wMilliseconds << std::endl;
		std::cout << logstr << "end new decision for control " << (localControlIntIdx - 1 + settings.nrofDecisionInt + 1) << " to " << (localControlIntIdx - 1 + settings.nrofDecisionInt + settings.nrofPredictionInt) << std::endl;
		std::cout << logstr << "nrofSearchNodes " << nrofSearchNodes << " nrofDecisionNodes " << nrofDecisionNodes << " nrofNodeEvaluations " << nrofNodeEvaluations << std::endl;

		//nrofpaths in decision tree
		int nrofSubPaths = 0;
		for (int tmpdnnr = 1; tmpdnnr <= nrofDecisionNodes; ++tmpdnnr) {
			decisionNodePtr tmpdn = decisionNodes[tmpdnnr];
			int tmplevidx = tmpdn->timeint() / settings.controlIntLngth;
			if (tmplevidx == settings.nrofPredictionInt) { //end node
				nrofSubPaths = nrofSubPaths + 1;
			}
		}
		std::cout << logstr << "nrofPaths " << nrofPaths << " nrofSubPaths " << nrofSubPaths << std::endl;

		std::cout << logstr << "decision is ready!" << std::endl;
		threadswitchSIM->setDecision(false); //ready;)     
		threadswitchSIM->setControlAvail(true);

	}
}

int main(int argc, char** argv)
{
	using namespace std::chrono;
	auto retVal = 0;
	try {

		#pragma region InitializationGlobalVariables

		//INITIALIZATION: define global variables; anything that has nothing to do with the network;  
		optimizationSettings settings; //general settings
		
		threadswitchPtr thrdswitch = std::make_shared<threadswitch>(); //switches to stop/go the separate threads

		std::unordered_map<int, vehiclePtr> vehicles{}; //vehicles with detector passages (see main thread below)
		std::unordered_map<int, vehiclePtr> tmpvehicles{}; //hardcopy; for reading only

		ControlIntervals controlIntervals; //map of time intervals with (simulated) statistics (see statistics thread below)
		for (int k = 0; k <= settings.nrofSimInt; ++k) { // initialize control interval structure (also k=0 for initialization!)
			decisionNodePtr contrint = std::make_shared<decisionNode>(k, nullptr, 0, 0, 0, -1);
			controlIntervals.add(k, contrint);
		}

		std::unordered_map<int, decisionNodePtr> optimalNodes{}; // optimal decision sequence shared between decision and control threads (see below)		
		std::unordered_map<int, decisionNodePtr> decisionNodes{}; //re-used search structure (tree (B&B), (set of) path(s) (G&A)) of possible subsequent decicions (with predicted statistics) (see decision thread below)		
		int nrofDecisionNodes = 0;	
		std::unordered_map<int, std::unordered_map<int, decisionNodePtr>> preDecisionNodes{}; //re-evaluated current decision sequence (with re-evaluated predicted statistics) (see decision thread below)

		junctionPtr timerNode; //for synchronization;
		std::string trigger = "GRN";//for synchronization;
		int tmptimer = 0;//for synchronization;
		double tmptimecor = 0.0; //synchronisation; 0.0 now using semaphore!
		std::atomic<bool> tmpnewtimeint = false;
		std::atomic<bool> tmphalftimeint = false;
		double tmptimedetcor = -0.3;// shift to compensate for time delay in signal activation (-0.3 for time step 0.1; -0.6 for time step 0.2)
		double predtimeshift = -0.2; //shift to compensate for difference in actual and effective green (passages through orange/red); 
		int tmpstatControlIntIdx = 0;

		std::mutex optimalNodes_mutex;
		#pragma endregion

		#pragma region OpenFiles
		//OPEN FILES

		#ifdef _DEBUG
		std::ofstream ofsJSON;
		ofsJSON.open("json.log", std::ofstream::out);
		if (!ofsJSON.is_open()) { std::cout << "Error opening json.log!" << std::endl; auto dummy = getchar();; return 1;	}
		#endif
		
		std::ofstream timerfile; //open timer file for writing
		timerfile.open("timer.txt", std::ofstream::out);
		if (!timerfile.is_open()) {	std::cout << "Error opening timer.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream statfile; //open statistics file for writing
		statfile.open("statistics.txt", std::ofstream::out);
		if (!statfile.is_open()) { std::cout << "Error opening statistics.txt!" << std::endl; auto dummy = getchar();; return 1; }
		
		std::ofstream statpredfile; //open predicted statistics file for writing
		statpredfile.open("statisticspred.txt", std::ofstream::out);
		if (!statpredfile.is_open()) { std::cout << "Error opening statisticspred.txt!" << std::endl; auto dummy = getchar();; return 1; }
		
		std::ofstream ctrseqfile; //open control sequence file for writing
		ctrseqfile.open("controlseq.txt", std::ofstream::out);
		if (!ctrseqfile.is_open()) { std::cout << "Error opening controlseq.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream vehOUT; //open arrival pattern file for writing 
		vehOUT.open("vehiclesOUT.txt", std::ofstream::out);
		if (!vehOUT.is_open()) { std::cout << "Error opening vehiclesOUT.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream vehIN; //open arrival pattern file for reading
		vehIN.open("vehiclesIN.txt", std::ifstream::in);
		if (!vehIN.is_open()) { std::cout << "Error opening vehiclesIN.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream treefile; //open decision tree file for reading
		treefile.open("decisiontree.txt", std::ifstream::in);
		if (!treefile.is_open()) { std::cout << "Error opening decisiontree.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream treefileOUT; //open decision tree file for writing
		treefileOUT.open("decisiontreeOUT.txt", std::ofstream::out);
		if (!treefileOUT.is_open()) { std::cout << "Error opening decisiontreeOUT.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream basicphasefile; //open basic structure control file for reading
		basicphasefile.open("phasesbasic.txt", std::ifstream::in);
		if (!basicphasefile.is_open()) { std::cout << "Error opening phasesbasic.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream phasefile; //open structure control file for reading
		phasefile.open("phases.txt", std::ifstream::in);
		if (!phasefile.is_open()) { std::cout << "Error opening phases.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream phasefileOUT; //open structure control file for writing
		phasefileOUT.open("phasesOUT.txt", std::ofstream::out);
		if (!phasefileOUT.is_open()) { std::cout << "Error opening phasesOUT.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream signalfile; //open signal file for reading
		signalfile.open("signals.txt", std::ifstream::in);
		if (!signalfile.is_open()) { std::cout << "Error opening signals.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream demandfile; //open demand file for reading
		demandfile.open("demand.txt", std::ifstream::in);
		if (!demandfile.is_open()) { std::cout << "Error opening demand.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ifstream splitfile; //open split fraction file for reading
		splitfile.open("splitfractions.txt", std::ifstream::in);
		if (!splitfile.is_open()) { std::cout << "Error opening splitfractions.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream satflowoutfile; //open detailed vehicle passing file for writing
		satflowoutfile.open("satflowsout.txt", std::ofstream::out);
		if (!satflowoutfile.is_open()) { std::cout << "Error opening satflowsout.txt!" << std::endl; auto dummy = getchar();; return 1; }

		std::ofstream tmplogfile; //open log file for writing
		tmplogfile.open("tmplogfile.txt", std::ofstream::out);
		if (!tmplogfile.is_open()) { std::cout << "Error opening tmplogfile.txt!" << std::endl; auto dummy = getchar();; return 1; }
		
		std::ofstream settingsfile; //open log file for writing
		settingsfile.open("settingsfile.txt", std::ofstream::out);
		if (!settingsfile.is_open()) { std::cout << "Error opening settingsfile.txt!" << std::endl; auto dummy = getchar();; return 1; }

		#pragma endregion

		//read preprocessed vehicles with free-flow passages from vehiclesIN.txt file (see function readVehicles)
		#pragma region ReadVehicles			
		{
			int nrofSG = 0; //count nrof signal groups from passages in vehicle file!
			readVehicles(vehIN, controlIntervals, settings, vehicles, nrofSG);

			//copy vehicle list for reading only: for individual queue info			
			for (auto [vehid, veh] : vehicles) { 
				vehiclePtr tmpveh = std::make_shared<vehicle>(vehid);
				tmpvehicles[vehid] = tmpveh;
				for (auto [passidx, pass]:veh->passages()) {
					passagePtr tmppas = std::make_shared<passage>(pass->oid(), pass->detid(), pass->detname(), pass->signame(), pass->jncname(), pass->sgnr());
					// predtime, lagtime, realtime, deltim not relevant in copy, set to zero.
					tmpveh->passage(passidx, tmppas);
				}			
				tmpveh->setNrofPassages(veh->getNrofPassages());
				tmpveh->setNextVehID(veh->getNextVehID());
				tmpveh->setSeqNr(veh->getSeqNr());			
			}
		}
		#pragma endregion

		#pragma region initialize websocket connection to Simulator
		//auto endpointUrl = "ws://localhost:3333";	//moved to settings

		auto aimsunWSClientHandler = std::make_unique<AimsunWSClientHandler>();

		auto aimsunWebsocketClient = std::make_shared<WebSocketClient>();
	
		AddReponseFilterFunction AddReponseFilter = aimsunWSClientHandler->GetFunctionAddReponseFilter();
		CancelReponseFilterFunction CancelReponseFilter = aimsunWSClientHandler->GetFunctionCancelReponseFilter();
		GetMessagesFunction GetMessages = aimsunWSClientHandler->GetFunctionGetMessages();
		aimsunWebsocketClient->SetUrl(settings.endpointUrl);
		aimsunWebsocketClient->SetHandler(std::move(aimsunWSClientHandler));
		#pragma endregion

		#pragma region Synchronized connection call
		int retries = 0;
		const int maxRetries = 60; // sec	
		while (retries < maxRetries) {		
			std::cout << "AimsunWebSocketClient connecting to simulator... " << std::endl;
			if (aimsunWebsocketClient->Connect()){
				std::cout << "AimsunWebSocketClient websocket connected" << std::endl;
				break;
			}
			std::this_thread::sleep_for(1000ms);
			++retries;
		}

		if (!aimsunWebsocketClient->IsConnected()) {
			std::cout << "Websocket connect error!" << std::endl;
			std::cout << "\n\nPress enter..." << std::endl;
			//getchar();
			return 1;
		}
		#pragma endregion

		#pragma region Synchronized configuration call

		auto awaitResponse = [](const std::chrono::seconds& timeout, ResponseFuture& responseFuture, std::string& reponseMessage) {
				auto responseReady = std::future_status::ready == responseFuture.wait_for(timeout);
				if (responseReady)
				{
					reponseMessage = responseFuture.get();
				}
				return responseReady;
		};

		std::cout << "Configuring simulator..." << std::endl;
		ResponsePromise configResponsePromise;
		auto configResponseFuture = configResponsePromise.get_future();

		std::string configReponse;
		bool configReponseError{true};

		const std::string configResponseFilter = "CFG:";
		AddReponseFilter(configResponseFilter, std::move(configResponsePromise) );
		if (aimsunWebsocketClient->Send("INI:5213D983-A660-4EA3-8CC9-846FA6FBF94D" ))
		{
			if (awaitResponse(30s, configResponseFuture, configReponse))
			{
				if (configReponse.size() > 4)
				{
					std::cout << "Application configured" << std::endl;
					configReponseError = false;
				}
			}
		}
		CancelReponseFilter(configResponseFilter);

		if (configReponseError) 
		{
			std::cout << "Configuration error!" << std::endl;
			std::cout << "\n\nPress enter..." << std::endl;
			//getchar();
			return 1;
		}
		#pragma endregion

		#pragma region Start sending regular ping messages
		std::weak_ptr<WebSocketClient> simulatorWebsocketClientWeakPtr = aimsunWebsocketClient;
		std::thread threadPing([simulatorWebsocketClientWeakPtr] {
			for (;;)
			{
				std::this_thread::sleep_for(1000ms);
				if (auto simulatorWebsocketClientPtr = simulatorWebsocketClientWeakPtr.lock())
				{
					if (simulatorWebsocketClientPtr->IsConnected())
					{
						simulatorWebsocketClientPtr->Send("PNG:");
						continue;
					}
				}

				std::cout << "Could not send message to simulator Aimsun (websocket Ping)" << std::endl;
				break;
			}
		});
		#pragma endregion

		#pragma region Synchronized request call
		std::cout << "Retrieving layout..." << std::endl;
		ResponsePromise layoutResponsePromise;
		auto layoutResponseFuture = layoutResponsePromise.get_future();
		std::string layoutReponse;
		bool layoutReponseError{ true };

		const std::string layoutResponseFilter = "RES:";
		AddReponseFilter(layoutResponseFilter, std::move(layoutResponsePromise));
		if ( aimsunWebsocketClient->Send("REQ:{}") )
		{
			if (awaitResponse(30s, layoutResponseFuture, layoutReponse))
			{
				if (layoutReponse.size() > 4)
				{
					std::cout << "Layout retrieved" << std::endl;
					layoutReponseError = false;
				}
			}
		}
		CancelReponseFilter(layoutResponseFilter);

		if (layoutReponseError) {
			std::cout << "Layout error!" << std::endl;
			std::cout << "\n\nPress enter..." << std::endl;
			auto dummy = getchar();
			return 1;
		}
		#pragma endregion

		//read network configuration, i.e., intersections, signal groups, detectors, from Aimsun (json message at beginning of simulation)
		//store in maps junctions_ with junction objects, signalGroups_ with signalGroup objects, detectors_ with detector objects 
		#pragma region read layoutReponse
		simulatorPtr sim{ nullptr };
		networkPtr net{ nullptr };
		bool bOK = false;
		if (0 == layoutReponse.find("RES:")) {
			Json::Reader reader;
			Json::Value message;
			if (reader.parse(layoutReponse.substr(4), message)) {
				if (message.isObject()) {
					auto& sim_ = message["SIM"];
					if (sim_.isObject()) {
						#pragma region SIM
						auto& obj_ = sim_["OBJ"];
						auto& oid_ = sim_["OID"];
						auto& nam_ = sim_["NAM"];
						auto& sss_ = sim_["SSS"];
						if (obj_.isString() && 0 == obj_.asString().compare("SIM") && oid_.isInt() && sss_.isDouble()) {
							auto sss = sss_.asDouble();
							if (sss != 0.1)
							{
								std::cout << "Error - Simulation step size is not 0.1s! Please set it to 0.1s in the simulation and restart." << std::endl;
								std::cout << "\n\nPress enter..." << std::endl;
								//getchar();
								return 1;
							}
							sim = std::make_shared<simulator>(sss);
						}
						#pragma endregion
						auto& net_ = message["NET"];
						if (sim && net_.isObject()) {
							#pragma region NET
							auto& obj_ = net_["OBJ"];
							auto& oid_ = net_["OID"];
							auto& nam_ = net_["NAM"];
							if (obj_.isString() && 0 == obj_.asString().compare("NET") && oid_.isInt()) {
								sim->network(std::make_shared<network>());
							}
							net = sim->network();
							#pragma endregion
							auto& jnc_ = message["JNC"];
							if (net && jnc_.isArray()) {
								#pragma region JNC
								// store JNC data if needed
								#pragma endregion
								auto& typ_ = message["TYP"];
								if (net && typ_.isArray()) {
									#pragma region TYP
									//store TYP data if needed
									#pragma endregion
									auto& lnk_ = message["LNK"];
									if (lnk_.isArray()) {
										#pragma region LNK
										//store LNK data if needed
										#pragma endregion
										auto& nde_ = message["NDE"];
										if (nde_.isArray()) {
											#pragma region NDE
											//store NDE data if needed
											
											for (auto& nde : nde_) {
												auto& obj_ = nde["OBJ"];
												auto& oid_ = nde["OID"];
												auto& nam_ = nde["NAM"];
												if (obj_.isString() && 0 == obj_.asString().compare("NDE") && oid_.isInt()) {
													auto ndeOid = oid_.asInt();
													std::string ndeNam;
													if (nam_.isString()) {
														ndeNam = nam_.asString();
														if (ndeNam != "") {
															if (ndeNam=="timer") {
																timerNode = std::make_shared<junction>(ndeOid, ndeNam, 0);
																std::cout << "timer node processed " << ndeOid << "\t" << ndeNam << "\t" << 0 << std::endl;
															}
															else {
																int ndeNr = (int)net->junctions().size() + 1;
																net->junction(ndeOid, std::make_shared<junction>(ndeOid, ndeNam, ndeNr));
																std::cout << "node processed " << ndeOid << "\t" << ndeNam << "\t" << ndeNr << std::endl;
															}
														}
													}
												}
											}
											#pragma endregion
											auto& mov_ = message["MOV"];
											if (mov_.isArray()) {
												#pragma region MOV
												//store MOV data if needed
												#pragma endregion
												auto& dtc_ = message["DTC"];
												if (dtc_.isArray()) {
													#pragma region DTC
													for (auto& dtc : dtc_) {
														auto& obj_ = dtc["OBJ"];
														auto& oid_ = dtc["OID"];
														auto& nam_ = dtc["NAM"];
														if (obj_.isString() && 0 == obj_.asString().compare("DTC") && oid_.isInt() ) {
															auto dtcOid = oid_.asInt();
															
															std::string dtcNam;
															std::string sigNam;
															std::string jncNam;
															if (nam_.isString()) {
																dtcNam = nam_.asString();
																auto idx = nam_.asString().find("_0");
																if (idx != std::string::npos) {
																	sigNam = nam_.asString().substr(idx - 2, 2);
																	idx = nam_.asString().find("/");
																	if (idx != std::string::npos) {
																		jncNam = nam_.asString().substr(0, idx);

																		int sgnr = std::stoi(sigNam, nullptr);
																		for (auto [tmpjncoid, tmpjnc] : net->junctions()) {
																			if (jncNam == tmpjnc->name()) {
																				// node in the list of junctions
																				int jncnr = tmpjnc->jncnnr();
																				sgnr = 12 * (jncnr - 1) + sgnr;
																			}
																		}

																		net->detector(dtcOid, std::make_shared<detector>(dtcOid, dtcNam, sigNam, jncNam, sgnr));
																		std::cout << "detector processed " << dtcOid << "\t" << dtcNam  << "\t" << sigNam << "\t" << jncNam << "\t" << sgnr << std::endl;

																	}
																}	
															}															
														}
														else throw nullptr;
													}
													#pragma endregion
													auto& zne_ = message["ZNE"];
													if (zne_.isArray()) {
														#pragma region ZNE
														//store ZNE data if needed
														#pragma endregion
														auto& bus_ = message["BUS"];
														if (bus_.isArray()) {
															//store BUS data if needed
															auto& vhc_ = message["VHC"];
															if (vhc_.isArray()) {
																#pragma region VHC
																//store VHC data if needed
																#pragma endregion
																auto& sig_ = message["SIG"];
																if (sig_.isArray()) {
																	#pragma region SIG
																	for (auto& sig : sig_) {
																		auto& obj_ = sig["OBJ"];
																		auto& oid_ = sig["OID"];
																		auto& nam_ = sig["NAM"];
																		auto& nde_ = sig["NDE"];
																		auto& ste_ = sig["STE"];
																		if (obj_.isString() && 0 == obj_.asString().compare("SIG") && oid_.isInt() && ste_.isString()) {
																			auto sigOid = oid_.asInt();
																			auto nde = nde_.asInt();
																			auto state = ste_.asString();
																			std::string sigNam;
																			if (nam_.isString()) {
																				sigNam = nam_.asString();
																			}
																			// select node from junction list
																			junctionPtr jnc = net->junctions()[nde];
																			if (jnc != nullptr){
																				std::string jncNam;
																				jncNam = jnc->name();
																				
																				signalGroupPtr sg = std::make_shared<signalGroup>(sigOid, sigNam, jncNam);
																				int sgnr = std::stoi(sigNam, nullptr);
																				int jncnr = jnc->jncnnr();
																				sgnr = 12 * (jncnr - 1) + sgnr; //internal signal numbers are subsequently numbered...
																				sg->setSgnr(sgnr);
																				net->signalGroup(sigOid, sg);
																				std::cout << "signal group processed " << sigOid << "\t" << sigNam << "\t" << jncNam << "\t" << sgnr << std::endl;
																				
																			}
																		}
																		else throw nullptr;
																	}
																	#pragma endregion
																	auto& mtr_ = message["MTR"];
																	if (mtr_.isArray()) {
																		#pragma region MTR
																		//store MTR data if needed
																		#pragma endregion
																		auto& vms_ = message["VMS"];
																		if (vms_.isArray()) {
																			#pragma region VMS
																			//store VMS data if needed
																			#pragma endregion
																			bOK = true;
																		}
																	}
																	#pragma region SIG
																	//store SIG data if needed
																	#pragma endregion
																}
															}
														}
														#pragma region ZNE
														//store ZNE data if needed
														#pragma endregion
													}
													#pragma region DTC
													//store DTC data if needed
													#pragma endregion
												}
												#pragma region MOV
												//store MOV data if needed
												#pragma endregion
											}
											#pragma region NDE
											//store NDE data if needed
											#pragma endregion
										}
										#pragma region LNK
										//store LNK data if needed
										#pragma endregion
									}
								}
							}
						}
					}
				}
			}
		}

		#ifdef _DEBUG
		ofsJSON << layoutReponse << std::endl;
		#endif

		if (!bOK) {
			throw nullptr;
		}
	
		std::cout << "layoutReponse processed" << std::endl;
		auto signalGroups = net->signalGroups();
		auto junctions = net->junctions();
		auto detectors = net->detectors();
		#pragma endregion

		//read user-specified control information from additional input files: 
		//read prediction model parameters from signals.txt (see function readSignals), splitfractions.txt and demand.txt (see function readSplitFractions)
		//read pre-defined control structure from phasesbasic.txt (see function readBasicPhases), phases.txt (see function readAndBuildPhases), decisiontree.txt (see function readAndBuildPhaseTree)
		#pragma region ReadControlStructure
		{
			readSignals(signalfile, net, settings);
			readSplitFractions(demandfile, splitfile, net, settings);
			readBasicPhases(basicphasefile, net, settings);
			bool buildPH = false; //from file if available; (becomes true if file is empty and phases are build from basic phases);
			readAndBuildPhases(phasefile, phasefileOUT, net, settings, buildPH);
			readAndBuildPhaseTree(treefile, treefileOUT, net, settings, buildPH);
		}
		#pragma endregion

		//write user-specified settings to log file settings.txt (see function WriteSettings)
		#pragma region CheckAndWriteSettings
		{
			writeSettings(settingsfile, settings);
		}
		#pragma endregion

		//calculate/aggregate simulated state statistics based on vehicle passages (in map vehicle of vehicle objects)
		//store statistics as attributes of a decisionNode object (for the current time interval) in map controlIntervals
		//write statistics for current time interval to statistics.txt (see function writeStatistics)
		#pragma region Separate Thread for statistics
			std::thread threadUpdateStatistics([&] {
				while (aimsunWebsocketClient->IsConnected())
				{
					if ((thrdswitch->getStatistics()) && (!thrdswitch->getStalledProcess())) { //separate thread, otherwise decision in 6 seconds to be ready...
						std::this_thread::sleep_for(2000ms); //wait 2 seconds till detector statistics are corrected: passages through red... (2sec less for optimization...)
						
						int localControlIntIdx = settings.controlIntIdx - 1; //controlIntIdx is updated before statistics are calculated!!!
						tmpstatControlIntIdx = localControlIntIdx; //used to check passages through red;
						
						// update statistics for delay calculation for previous control interval;
						auto thisControlInterval = controlIntervals.get(localControlIntIdx);
						auto prevControlInterval = controlIntervals.get(localControlIntIdx - 1);
						if (localControlIntIdx > 0) {

							//ini
							if (settings.individualqueueSTAT) { //initialize individual vehicle positions to previous node; 
								for (auto [signalgrid, signalgr] : signalGroups) {
									int sgnr = signalgr->getSgnr();
									int internalsgnr = signalgr->getInternalSgnr();
									thisControlInterval->leadvehid(sgnr, prevControlInterval->leadvehids(sgnr));
									if (signalgr->getInternalArr()) {
										int maxpos = thisControlInterval->getSizeVehids(internalsgnr) - 1; //20;
										for (int pos = 0; pos <= maxpos; ++pos) {
											 thisControlInterval->vehid(internalsgnr, pos, prevControlInterval->vehids(internalsgnr, pos)); 
										}
									}
								}
							}

							for (auto [signalgrid, signalgr] : signalGroups) {
								int sgnr = signalgr->getSgnr();

								double arr = thisControlInterval->arrivals(sgnr); // updated in preprocessing (external) or updated detector passage (internal)
								double arrLNK = thisControlInterval->arrivalsLNK(sgnr); // updated in preprocessing (external) or updated detector passage (internal)
								double cumarr = prevControlInterval->cumarrivals(sgnr) + arr; //recalculated: preprocessed (external only)
								double cumarrLNK = prevControlInterval->cumarrivalsLNK(sgnr) + arrLNK; //recalculated: preprocessed (external only)
								double dep = thisControlInterval->departures(sgnr); // updated detector passage
								double cumdep = prevControlInterval->cumdepartures(sgnr) + dep;
								double cumqueue = cumarr - cumdep; // actually the queue at that moment
								double cumdelay = prevControlInterval->cumdelays(sgnr) + cumqueue * settings.controlIntLngth;
								double cumveh = cumarrLNK - cumdep;  // number of vehicles on the link at that moment
								double cumtime = prevControlInterval->cumtimes(sgnr) + cumveh * settings.controlIntLngth;
								thisControlInterval->cumarrival(sgnr, cumarr);
								thisControlInterval->cumarrivalLNK(sgnr, cumarrLNK);
								thisControlInterval->cumdeparture(sgnr, cumdep);
								thisControlInterval->cumqueue(sgnr, cumqueue);
								thisControlInterval->cumdelay(sgnr, cumdelay);
								thisControlInterval->cumvehicle(sgnr, cumveh);
								thisControlInterval->cumtime(sgnr, cumtime);

								if (settings.individualqueueSTAT) {
									std::unordered_map<int, int> tmpnextSignalGroups; //temporary next signal group "fractions" with number of vehicles (part of deltaDep) <nextsgid,#>
									int n = CalculateIndividuals(thisControlInterval, prevControlInterval, settings, net, signalgr, dep, tmpvehicles, false, tmpnextSignalGroups, false);
								}
							}

							double totcumarr = 0.0;
							double totcumarrLNK = 0.0;
							double totcumdep = 0.0;
							double totcumqueue = 0.0;
							double totcumdelay = 0.0;
							double totcumveh = 0.0;
							double totcumtime = 0.0;
							for (auto [signalgrid, signalgr] : signalGroups) {
								int sgnr = signalgr->getSgnr();

								//Update headtail of queue
								thisControlInterval->head(sgnr, 0.0);
								thisControlInterval->tail(sgnr, 0.0);
								thisControlInterval->tmphead(sgnr, 0.0);
								thisControlInterval->tmptail(sgnr, 0.0);
								if ((settings.headtailSTAT) && (signalgr->getInternalArr())) { //internal signal groups only
									CalculateHeadTail(thisControlInterval, prevControlInterval, settings, signalgr, false);
								}

								totcumarr = totcumarr + thisControlInterval->cumarrivals(sgnr);
								totcumarrLNK = totcumarrLNK + thisControlInterval->cumarrivalsLNK(sgnr);
								totcumdep = totcumdep + thisControlInterval->cumdepartures(sgnr);
								totcumqueue = totcumqueue + thisControlInterval->cumqueues(sgnr);
								totcumdelay = totcumdelay + thisControlInterval->cumdelays(sgnr);
								totcumveh = totcumveh + thisControlInterval->cumvehicles(sgnr);
								totcumtime = totcumtime + thisControlInterval->cumtimes(sgnr);
							}
							thisControlInterval->cumarrival(0, totcumarr);
							thisControlInterval->cumarrivalLNK(0, totcumarrLNK);
							thisControlInterval->cumdeparture(0, totcumdep);
							thisControlInterval->cumqueue(0, totcumqueue);
							thisControlInterval->cumdelay(0, totcumdelay);
							thisControlInterval->cumvehicle(0, totcumveh);
							thisControlInterval->cumtime(0, totcumtime);
							thisControlInterval->head(0, 0.0);
							thisControlInterval->tail(0, 0.0);
							thisControlInterval->tmphead(0, 0.0);
							thisControlInterval->tmptail(0, 0.0);

							writeStatistics(statfile, net, localControlIntIdx, thisControlInterval, 1, tmpvehicles, settings);
						}
						else { //localControlIntIdx == 0: initialization
							for (auto[signalgrid, signalgr] : signalGroups) {
								int sgnr = signalgr->getSgnr();
								thisControlInterval->arrivalLNK(sgnr, 0.0);
								thisControlInterval->cumarrivalLNK(sgnr, 0.0);
								thisControlInterval->arrival(sgnr, 0.0);
								thisControlInterval->cumarrival(sgnr, 0.0);
								thisControlInterval->departure(sgnr, 0.0);
								thisControlInterval->cumdeparture(sgnr, 0.0);
								thisControlInterval->cumqueue(sgnr, 0.0);
								thisControlInterval->cumdelay(sgnr, 0.0);
								thisControlInterval->cumvehicle(sgnr, 0.0);
								thisControlInterval->cumtime(sgnr, 0.0);
								thisControlInterval->head(sgnr, 0.0);
								thisControlInterval->tail(sgnr, 0.0);
								thisControlInterval->tmphead(sgnr, 0.0);
								thisControlInterval->tmptail(sgnr, 0.0);

								if (settings.individualqueueSTAT) {
									if (signalgr->getInternalArr()) {
										int internalsgnr = signalgr->getInternalSgnr();
										thisControlInterval->leadvehid(sgnr, 0);
										int maxpos = thisControlInterval->getSizeVehids(internalsgnr) - 1; 
										for (int pos = 0; pos <= maxpos; ++pos) {
											thisControlInterval->vehid(internalsgnr, pos, 0);
										}
									}
									else {//external signal group
										for (auto [tmpvehid, tmpveh] : tmpvehicles) {
											if ((tmpveh->getSeqNr() == 1) && (tmpveh->passages()[1]->sgnr() == sgnr)) {
												thisControlInterval->leadvehid(sgnr, tmpvehid);
											}
										}
									}
								}
							}
							thisControlInterval->arrivalLNK(0, 0.0);
							thisControlInterval->cumarrivalLNK(0, 0.0);
							thisControlInterval->arrival(0, 0.0);
							thisControlInterval->cumarrival(0, 0.0);
							thisControlInterval->departure(0, 0.0);
							thisControlInterval->cumdeparture(0, 0.0);
							thisControlInterval->cumqueue(0, 0.0);
							thisControlInterval->cumdelay(0, 0.0);
							thisControlInterval->cumvehicle(0, 0.0);
							thisControlInterval->cumtime(0, 0.0);
							thisControlInterval->head(0, 0.0);
							thisControlInterval->tail(0, 0.0);
							thisControlInterval->tmphead(0, 0.0);
							thisControlInterval->tmptail(0, 0.0);

							writeStatistics(statfile, net, localControlIntIdx, thisControlInterval, 1, tmpvehicles, settings);
						}

						
						thrdswitch->setStatistics(false); //ready;)
						
					}
					std::this_thread::sleep_for(100ms);
				}
			});
		#pragma endregion

		//optimize new decision sequence for the upcoming prediction horizon (starting in the (shifted!) initial state in map controlIntervals) (see function makeNewDecision(GA))
		//store new decision sequence in the map optimalNodes
		#pragma region Separate Thread for control decision
			std::thread threadControlDecision([&] {
				
				while (aimsunWebsocketClient->IsConnected())
				{
					// start optimization loop: for interval 1,..., N  (control horizon) (intervals of f.e. 6 seconds)
					// start with (shifted) current state as the first decision node; ->MakeNewDecision
					
					if (thrdswitch->getDecision()) { //if newDecision is true; && statistics false == ready ;)
						
						thrdswitch->setAbortDecision(false);
						
						thrdswitch->setControlAvail(false);
						
						if (!thrdswitch->getStatistics()) { // most recent statistics need to be ready!
							
							//make copy because statistics changes settings.controlIntIdx during decision making!
							optimizationSettings localsettings;
							int localControlIntIdx = settings.controlIntIdx; //hardcopy!;
							localsettings.controlIntIdx = localControlIntIdx; //hardcopy!

							//use changes made in localsettings from previous decision! (needed for robust heuristic)
							for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
								localsettings.scenarioActive[scnr] = settings.scenarioActive[scnr];
							}
							localsettings.scenarioContainer.clear();
							for (int i = 0; i < settings.scenarioContainer.size(); ++i) {
								localsettings.scenarioContainer.push_back(settings.scenarioContainer[i]);
							}

							//check if end of simulation horizon is reached
							int lastInt = localsettings.controlIntIdx - 1 + localsettings.nrofDecisionInt + localsettings.nrofPredictionInt;
							int firstInt = localsettings.controlIntIdx - 1 + localsettings.nrofDecisionInt;
							if (lastInt > localsettings.nrofSimInt) {
								std::cout << "no new decision for control possible: end of simulation horizon reached " << lastInt << std::endl;
								thrdswitch->setControlAvail(false);
								{
									std::lock_guard lock(optimalNodes_mutex);									
									optimalNodes.clear();
									for (int l = 0; l <= localsettings.nrofPredictionInt; ++l) {
										decisionNodePtr tmpdn = std::make_shared<decisionNode>(0, net->phases(0), 0, 0, 0, -1);
										optimalNodes[l] = tmpdn;
									}									
								}
								thrdswitch->setControlAvail(true);

								std::cout << "end new decision for control: all red " << (firstInt + 1) << " to " << (lastInt) << std::endl;
								thrdswitch->setDecision(false); //ready;)
							}
							else {
								if (localsettings.activateMethod == 1) {
									MakeNewDecision(localsettings, optimalNodes, net, controlIntervals, optimalNodes_mutex, preDecisionNodes, decisionNodes, nrofDecisionNodes, tmplogfile, statpredfile, thrdswitch, "", true, tmpvehicles);
								}
								if (localsettings.activateMethod == 2) {
									MakeNewDecisionGA(localsettings, optimalNodes, net, controlIntervals, optimalNodes_mutex, preDecisionNodes, decisionNodes, nrofDecisionNodes, tmplogfile, statpredfile, thrdswitch, "", true, tmpvehicles);
								}
								//save changes made in localsettings! (needed for robust heuristic)
								for (int scnr = 0; scnr <= settings.nrofScenarios; ++scnr) {
									settings.scenarioActive[scnr] = localsettings.scenarioActive[scnr];
								}
								settings.scenarioContainer.clear();
								for (int i = 0; i < localsettings.scenarioContainer.size(); ++i) {
									settings.scenarioContainer.push_back(localsettings.scenarioContainer[i]);
								}

							}
						}
					}
					
					std::this_thread::sleep_for(100ms);
				}
			});
		#pragma endregion

		//process the current optimal decision sequence (in map optimalNodes), and send (json) messages to switch signals in Aimsun 
		//write activated control decision to controlseq.txt (in format [intervalidx, phaseid])
		#pragma region Separate Thread for controller (sending messages)
			std::thread threadController([&] {
				// initialize to all red;
				std::cout << "initialize all signals to red " << std::endl;
				net->setActivePhase(net->phases(0));//All RED
				for (auto[sgID, sg] : signalGroups)
				{
					// send RED to all signalgroups initially 
					auto aimsunid = std::to_string(sg->oid());
					auto initState = aimsunSGState::RED;
					sendSGUpdateToAimsun(simulatorWebsocketClientWeakPtr, aimsunid, SGStateMap.at(initState));
					int sgnr = sg->getSgnr();
					for (int intidx = 0; intidx <= settings.nrofDecisionInt; ++intidx) {
						controlIntervals.get(intidx)->state(sgnr, "RED");
						controlIntervals.get(intidx)->stateDuration(sgnr, intidx * settings.controlIntLngth);
						controlIntervals.get(intidx)->setPhase(net->phases(0));
					}
				}

				if (settings.preprocessing) {

					// NOTE: PREPROCESSING TO STORE UNDELAYED ARRIVALS!!! 
					std::cout << "set signals to green to store undelayed arrivals " << std::endl;
					for (auto [sgID, sg] : signalGroups)
					{// each signalgroup
						auto aimsunid = std::to_string(sg->oid());
						int sgnr = sg->getSgnr();
						int sgnam = std::stoi(sg->name(), nullptr);
						int jncnam = std::stoi(sg->jncname(), nullptr);

						if (sgnam <= 12) {
							//change free flow path in here (one free flow path at a time)!
							if ((jncnam == 1 && ((sgnam == 1) || (sgnam == 2) || (sgnam == 3))) ||
								(jncnam == 2 && ((sgnam == 1) || (sgnam == 2) || (sgnam == 3))) || (jncnam == 2 && ((sgnam == 7) || (sgnam == 8) || (sgnam == 9))) ||
								(jncnam == 3 && ((sgnam == 1) || (sgnam == 2) || (sgnam == 3))) || (jncnam == 3 && ((sgnam == 7) || (sgnam == 8) || (sgnam == 9))) ||
								(jncnam == 4 && ((sgnam == 7) || (sgnam == 8) || (sgnam == 9))) ||
								(jncnam == 1 && ((sgnam == 7) || (sgnam == 8) || (sgnam == 9)))) {

								auto initState = aimsunSGState::GRN;
								sendSGUpdateToAimsun(simulatorWebsocketClientWeakPtr, aimsunid, SGStateMap.at(initState));
							}
						}
					}

					//run simulation without feedback of optimum signals
					while (aimsunWebsocketClient->IsConnected())
					{
						while (!tmphalftimeint) {
							std::this_thread::sleep_for(std::chrono::milliseconds(100));
						}
						if (tmphalftimeint) {
							tmphalftimeint = false;
								
							//release semaphore (use subfunction!)
							int retryCount = 0;
							int releasedSema = 0;
							while ((retryCount++ < 10) && (!releasedSema)) {
								releasedSema = ReleaseSemaphore();
								std::this_thread::sleep_for(1000ms);
							}
						}
						while ((!tmpnewtimeint)) {
							std::this_thread::sleep_for(std::chrono::milliseconds(100));
						}
						if (tmpnewtimeint) {
							tmpnewtimeint = false;
								
							int retryCount = 0;
							int releasedSema = 0;
							while ((retryCount++ < 10) && (!releasedSema)) {
								releasedSema = ReleaseSemaphore();
								std::this_thread::sleep_for(1000ms);
							}
						}
					}					
				}
				else {
					
					// Complete simulation if settings.controlIntIdx starts at 0
					// Warm start if settings.controlIntIdx starts at userdefined value > 0 (not supported yet!)

					std::cout << "Complete simulation!!!" << std::endl;

					// initialize statistics all zero at beginning of simulation;
					settings.controlIntIdx++; // ==1 always one int ahead from tmpcontrolintidx
					thrdswitch->setStatistics(true); 

					// determine the first control decision sequence...
					thrdswitch->setDecision(true);

					for (int l = 1; l <= settings.nrofDecisionInt; ++l) {
						while (!tmphalftimeint) {
							std::this_thread::sleep_for(std::chrono::milliseconds(100));
						}
						if (tmphalftimeint) {
							thrdswitch->setStalledProcess(true);

							tmphalftimeint = false;

							std::cout << "passed timeint" << (l - 0.5) * settings.controlIntLngth << std::endl;

							//release semaphore (use subfunction!)
							int retryCount = 0;
							int releasedSema = 0;
							while ((retryCount++ < 10) && (!releasedSema)) {
								releasedSema = ReleaseSemaphore();
								std::this_thread::sleep_for(1000ms);
							}
							if (releasedSema) { thrdswitch->setStalledProcess(false); }
						}

						while ((!tmpnewtimeint)) {
							std::this_thread::sleep_for(std::chrono::milliseconds(100));
						}
						if (tmpnewtimeint) {
							thrdswitch->setStalledProcess(true);

							tmpnewtimeint = false;

							ctrseqfile << settings.controlIntIdx << "\t" << net->getActivePhase()->oid() << std::endl;

							settings.controlIntIdx++; //interval that has to start;
							thrdswitch->setStatistics(true);

							std::cout << "passed timeint" << l * settings.controlIntLngth << std::endl;

							//if full time interval has elapsed release semaphore (except last interval!)
							if (l < settings.nrofDecisionInt) {
								int retryCount = 0;
								int releasedSema = 0;
								while ((retryCount++ < 10) && (!releasedSema)) {
									releasedSema = ReleaseSemaphore();
									std::this_thread::sleep_for(1000ms);
								}
								if (releasedSema) { thrdswitch->setStalledProcess(false); }
							}
						}
					}
					
					while (aimsunWebsocketClient->IsConnected())
					{					
						//check if decision is  ready

						if (!settings.suboptimal) { //wait till optimal solution is available (optional);
							while (thrdswitch->getDecision()) {
								std::this_thread::sleep_for(std::chrono::milliseconds(100));
							}
						}

						if (!thrdswitch->getDecision()) { // if newDecision is false == ready;)
							std::cout << "optimal control is ready" << std::endl;
						}
						else {
							std::cout << "optimal control not ready yet: suboptimal control is used" << std::endl;
						}
						
						std::unordered_map<int, decisionNodePtr> tmpOptimalNodes{};					
						if (thrdswitch->getControlAvail()) {
							std::cout << "control available " << std::endl;
							// copy control action to tmpoptcontrol list; when done, new decision can be made and control list can be overwritten;
							{
								std::lock_guard<std::mutex> lock(optimalNodes_mutex);
								for (auto [tmpidx, tmpdn] : optimalNodes) {
									tmpOptimalNodes[tmpidx] = tmpdn;
								}
							}
						}
						else { std::cout << "warning: new phase for control not available yet" << std::endl; }

						thrdswitch->setAbortDecision(true); //abort current decision	
						thrdswitch->setDecision(true); //start new decision
							
						//activate next phase in Aimsun 
						for (int l = 1; l <= settings.nrofDecisionInt; ++l) {

							//To be sure Aimsun is stalled for a minimum number of seconds (before sending signals!)
							std::this_thread::sleep_for(std::chrono::milliseconds(500));

							std::unordered_map<int, signalGroupPtr> tmpSignalGroupsOnHold{}; //AimsunID						
							if (tmpOptimalNodes.size() >= 2) {

								//update signal states & durations for micro simulation statistics and activate in Aimsun!
								decisionNodePtr dn = tmpOptimalNodes[l];
								phasePtr ph = dn->phase();
									
								// terminate (set to RED) signal groups in active phase that are not in next phase
								// keep signal groups green that are in current and next phase (= do nothing, adapt duration only)
								// set signal groups to GRN if signal group in next phase is not in active phase + 3 sec loss time (all red)
								// keep signal groups red that are not in current and not in next phase (=do nothing, adapt duration only...)
								for (auto [signalgrid, signalgr] : signalGroups) {
									int sgnr = signalgr->getSgnr();

									bool current = false;									
									if (controlIntervals.get(settings.controlIntIdx - 1)->states(sgnr) == "GRN") { current = true; }

									bool next = false;
									if (dn->states(sgnr) == "GRN") { next = true; }

									//default no change in state; state duration increased by intlngth; GRN->GRN or RED->RED
									controlIntervals.get(settings.controlIntIdx)->state(sgnr, controlIntervals.get(settings.controlIntIdx - 1)->states(sgnr));
									controlIntervals.get(settings.controlIntIdx)->stateDuration(sgnr, controlIntervals.get(settings.controlIntIdx - 1)->stateDurations(sgnr) + settings.controlIntLngth);
									if (current && !next) { //GRN->RED
										auto aimsunid = std::to_string(signalgr->oid());
										auto nextState = aimsunSGState::RED;
										sendSGUpdateToAimsun(simulatorWebsocketClientWeakPtr, aimsunid, SGStateMap.at(nextState));
										//update state
										controlIntervals.get(settings.controlIntIdx)->state(sgnr, "RED");
										controlIntervals.get(settings.controlIntIdx)->stateDuration(sgnr, settings.controlIntLngth);
									}
									if (!current && next) { //RED->GRN										
										tmpSignalGroupsOnHold[signalgrid] = signalgr;
									}
								}
							}
							else { std::cout << "error: new phase for control not generated" << std::endl; }
	
							//signal states are send to Aimsun; release semaphore;
							{
								int retryCount = 0;
								int releasedSema = 0;
								while ((retryCount++ < 10) && (!releasedSema)) {
									releasedSema = ReleaseSemaphore();
									std::this_thread::sleep_for(1000ms);
								}
								if (releasedSema) { thrdswitch->setStalledProcess(false); } 
							}

							//sleep while Aimsun runs to next semaphore; == half way interval;
							while (!tmphalftimeint) {
								std::this_thread::sleep_for(std::chrono::milliseconds(100));
							}
							if (tmphalftimeint) {
								thrdswitch->setStalledProcess(true); 
								tmphalftimeint = false;
								std::cout << "passed timeint" << (l - 0.5) * settings.controlIntLngth << std::endl;
							}

							//To be sure Aimsun is stalled for a minimum number of seconds (before sending signals!)
							std::this_thread::sleep_for(std::chrono::milliseconds(500));

							if (tmpOptimalNodes.size() >= 2) {

								// switch to green aftersettings.settings.lossTime... half way interval!
								for (auto [signalgrid, signalgr] : tmpSignalGroupsOnHold) {
									auto aimsunid = std::to_string(signalgr->oid());
									auto nextState = aimsunSGState::GRN;
									sendSGUpdateToAimsun(simulatorWebsocketClientWeakPtr, aimsunid, SGStateMap.at(nextState));
									//update state
									int sgnr = signalgr->getSgnr();
									controlIntervals.get(settings.controlIntIdx)->state(sgnr, "GRN");
									controlIntervals.get(settings.controlIntIdx)->stateDuration(sgnr, settings.controlIntLngth - settings.lossTime);
								}
								net->setActivePhase(tmpOptimalNodes[l]->phase());
								controlIntervals.get(settings.controlIntIdx)->setPhase(tmpOptimalNodes[l]->phase());
									
							}
							else { std::cout << "error: new phase for control not generated" << std::endl; }
		
							//signal states are send to Aimsun; release semaphore;
							{
								int retryCount = 0;
								int releasedSema = 0;
								while ((retryCount++ < 10) && (!releasedSema)) {
									releasedSema = ReleaseSemaphore();
									std::this_thread::sleep_for(1000ms);
								}
								if (releasedSema) { thrdswitch->setStalledProcess(false); } 
							}

							//sleep while Aimsun runs to next semaphore; == next interval;
							while (!tmpnewtimeint) {
								std::this_thread::sleep_for(std::chrono::milliseconds(100));
							}
							if (tmpnewtimeint) {
								thrdswitch->setStalledProcess(true);
								tmpnewtimeint = false;
								std::cout << "passed timeint" << l * settings.controlIntLngth << std::endl;

								ctrseqfile << settings.controlIntIdx << "\t" << net->getActivePhase()->oid() << std::endl;

								settings.controlIntIdx++; //interval that has to start
								thrdswitch->setStatistics(true);
							}
						}
					}
				}
				std::cout << "Controller thread exit" << std::endl;
			});
		#pragma endregion

		//process vehicle detector passages from Aimsun (json messages)
		//store and update passage times in vehicle objects of the vehicle map
		//write vehicle passages to output file vehiclesOUT.txt (same format as vehiclesIN.txt, see function readVehicles), and log file satflowsout.txt
		#pragma region Main Thread for receiving messages
			{
				const double simulationStep = sim->sss();
				std::cout << "Simulation step size = " << simulationStep << std::endl;
				std::unordered_map<std::string /*detector*/, std::string /*vehId*/> wasHereBefore{};
				double timStart{ DBL_MAX };
				double timeSync = 0.;
				bool errorMainProcess = false;

				std::chrono::system_clock::time_point oneSecondTLCTime = std::chrono::system_clock::now();

				while ((aimsunWebsocketClient->IsConnected())&(!errorMainProcess))
				{
					auto aimsunMessages = GetMessages();
					auto lastAimsunMessagesSize = aimsunMessages.size();
					while (!aimsunMessages.empty()) {
						auto aimsunMessage = aimsunMessages.front();
						#ifdef _DEBUG
						ofsJSON << aimsunMessage << std::endl;
						#endif
						if (0 == aimsunMessage.find("MSG:")) {
							Json::Reader reader;
							Json::Value message;
							if (reader.parse(aimsunMessage.substr(4), message)) {
								if (message.isObject()) {
									#pragma region base
									auto& tim_ = message["TIM"];
									auto& obj_ = message["OBJ"];
									auto& oid_ = message["OID"];
									auto tim = tim_.asDouble();
									if (timStart == DBL_MAX) {
										timStart = tim;
									}
									#pragma endregion
									if (obj_.isString() && oid_.isInt()) {
										auto obj = obj_.asString();
										auto oid = oid_.asInt();
										#pragma region DTC
										if (0 == obj.compare("DTC")) {
											auto& ste_ = message["STE"];
											auto& spd_ = message["SPD"];
											auto& len_ = message["LEN"];
											auto& vhc_ = message["VHC"];

											if (ste_.isString()) {
												auto ste = ste_.asString();
												// do something here for DTC data
											}
										}
										#pragma endregion
										#pragma region SIG
										else if (0 == obj.compare("SIG")) {
											auto& ste_ = message["STE"];
											auto& nde_ = message["NDE"];

											if (ste_.isString()) {
												auto ste = ste_.asString();
												auto nde = nde_.asInt();
												// Do something here for SIG data											
												std::string nam;
												std::unordered_map<int, signalGroupPtr>::iterator ittmp1 = signalGroups.find(oid);
												if (ittmp1 != signalGroups.end()) { nam = ittmp1->second->name(); }
												
												if (nde == timerNode->oid()) {//trigger
													if (trigger != ste) {													
														tmptimer += settings.controlIntLngth/2;
														trigger = ste;
														timerfile << settings.controlIntLngth/2 << " simulation seconds have passed " << tmptimer << " real time " << tim << std::endl;
														tmptimecor = tmptimer - tim;
														//check if tmptimecor gets larger than 0.0! Processes are not in sync and need to be corrected!
														if (tmptimecor >= 0.05) { 
															std::cout << "ERROR PROCESS: PROCESSES ARE NOT IN SYNC AND NEED TO BE CORRECTED! " << std::endl;
															errorMainProcess = true;
														}
														if (ste == "GRN") { tmpnewtimeint = true; } 
														if (ste == "RED") { tmphalftimeint = true; }
													}													
													satflowoutfile << tim << "\t" << (tim + tmptimecor) << "\t" << (tim + tmptimecor) << "\t" << "TLC" << "\t" << nde << "\t" << oid << "\t" << "timer" << "\t" << ste << std::endl;
												}
												std::unordered_map<int, junctionPtr>::iterator ittmp2 = junctions.find(nde);
												if (ittmp2 != junctions.end()) {
													std::string jncnam = ittmp2->second->name();

													//check if signals switch as expected (with tmptimedetcor lag)!
													if (tmptimer >= settings.controlIntLngth * settings.nrofDecisionInt) {
														if ((double)tmptimer - (tim + tmptimecor) >= tmptimedetcor + 0.05) { 
															std::cout << "ERROR PROCESS: SIGNALS SWITCH TOO SOON! " << std::endl; 
															errorMainProcess = true;
														}
														if ((double)tmptimer - (tim + tmptimecor) <= tmptimedetcor - 0.05) { 
															std::cout << "ERROR PROCESS: SIGNALS SWITCH TOO LATE! " << std::endl;
															errorMainProcess = true;
														}
													}
											
													satflowoutfile << tim << "\t" << (tim + tmptimecor) << "\t" << (tim + tmptimecor + tmptimedetcor) << "\t" << "TLC" << "\t" << nde << "\t" << oid << "\t" << jncnam << "_" << nam << "\t" << ste << std::endl;
												}
											}
										}
										#pragma endregion
										#pragma region CTU
										else if (0 == obj.compare("JNC")) {
											auto& jnc = message["JNC"];
											auto& jncid_ = message["OID"];
											{
												if (jncid_.isInt())
												{
													auto jncid = jncid_.asInt();
													// Do something here for JNC data (floating car) data
												}
											}
										}
										#pragma endregion
										#pragma region VHC
										else if (0 == obj.compare("VHC")) {
											auto& ste_ = message["STE"]; // in case of entrance of vehicle in network
											auto& dtcid_ = message["DTC"]; // in case of detection of vehicle in network
											auto& oid_ = message["OID"];
											if (ste_.isString()) {
												auto ste = ste_.asString();
												auto oid = oid_.asInt();
												// Do something here with VHC (entrance) data 
											}
											if (dtcid_.isInt()) {
												auto dtcid = dtcid_.asInt();
												auto oid = oid_.asInt();
												// Do something here with VHC (detection) data 

												for (auto[id, dt] : detectors)
												{// each detector
													if (dtcid == dt->oid())
													{// detector in the message is in the list of detectors

														double realtime = tim + tmptimecor + tmptimedetcor;//synchronize & correct time lag
														double predtime = realtime + predtimeshift;//synchronize
														double lagtime = realtime + predtimeshift;//synchronize
														double delay = realtime - lagtime; //initial delay of predtimeshift
														vehiclePtr veh = vehicles[oid];
														decisionNodePtr thisControlInterval = nullptr;
														decisionNodePtr detectedControlInterval = nullptr;			
														if (veh != nullptr) {

															//loop over passages list, to find right detector, and idx on the path
															int curpasidx = -1;
															for (auto [pasidx, pas] : veh->passages()) {
																if (dtcid == pas->detid()) { //current passage
																	curpasidx = pasidx;

																	predtime = pas->predtime();//is already corrected
																	lagtime = pas->lagtime();//is already corrected

																	//realtime cannot be smaller than lagtime! delay cannot be negative!
																	//if (realtime < lagtime) { realtime = lagtime; } // do not adapt realtime; realtime is leading! adapt arrivals instead!
																	delay = realtime - lagtime;
																	if (delay < 0) { delay = 0; }

																	// update current passage::realtime															
																	pas->setRealTime(realtime);
																	pas->setDelay(delay);

																	//update departures
																	//find the corresponding signal number
																	int sgnr = pas->sgnr(); // int sgnr = dt->sgnr;
																	
																	if (sgnr > 0) {

																		//lookup signalgroup;
																		signalGroupPtr sggr = nullptr;
																		for (auto [tmpsggrid, tmpsgggr] : signalGroups) {
																			if (tmpsgggr->getSgnr() == sgnr) {
																				sggr = tmpsgggr;
																			}
																		}
																		
																		int k = (int)floor(realtime / settings.controlIntLngth + 1); //rounded downwards
																		thisControlInterval = controlIntervals.get(k);
		
																		//check passages through red...
																		if (thisControlInterval->states(sgnr) == "RED") {
																			satflowoutfile << "passage through red!!!" << std::endl;

																			//always correct passages through red; if necessary correct arrivals as well
																			k = k - 1;
																			thisControlInterval = controlIntervals.get(k);
																			satflowoutfile << "departures corrected!!!" << std::endl;
											
																			//check time after red light passage
																			if (tmpstatControlIntIdx >= (k)) {
																				satflowoutfile << "PASSAGE THROUGH RED CANNOT BE CORRECTED: STATISTICS ARE ALREADY CALCULATED! " << std::endl;
																				std::cout << "ERROR PROCESS: PASSAGE THROUGH RED CANNOT BE CORRECTED: STATISTICS ARE ALREADY CALCULATED! " << std::endl;
																				//errorMainProcess = true;
																				k = k + 1; //accept passage through red
																				thisControlInterval = controlIntervals.get(k);
																			}
																			
																		}
																		thisControlInterval->departure(sgnr, thisControlInterval->departures(sgnr) + 1);
																		thisControlInterval->departure(0, thisControlInterval->departures(0) + 1);

																		//if necessary, correct arrivals if lagtime interval is larger then >> thisk 
																		int prevk = (int)floor(lagtime / settings.controlIntLngth + 1);
																		detectedControlInterval = controlIntervals.get(prevk);
																		if (k < prevk) {
																			//arrivals need to be corrected
																			detectedControlInterval->arrival(sgnr, detectedControlInterval->arrivals(sgnr) - 1);
																			thisControlInterval->arrival(sgnr, thisControlInterval->arrivals(sgnr) + 1);
																			detectedControlInterval->arrival(0, detectedControlInterval->arrivals(0) - 1);
																			thisControlInterval->arrival(0, thisControlInterval->arrivals(0) + 1);
																			if (!(sggr->getInternalArr())) { //external signal groups also the arrivalsLNK!!!
																				detectedControlInterval->arrivalLNK(sgnr, detectedControlInterval->arrivalsLNK(sgnr) - 1);
																				thisControlInterval->arrivalLNK(sgnr, thisControlInterval->arrivalsLNK(sgnr) + 1);
																				detectedControlInterval->arrivalLNK(0, detectedControlInterval->arrivalsLNK(0) - 1);
																				thisControlInterval->arrivalLNK(0, thisControlInterval->arrivalsLNK(0) + 1);
																			}
																			satflowoutfile << "arrivals corrected!!!" << std::endl;
																		}
																	}
																}
																
															}
															if (curpasidx < 0) { std::cout << "passage not found: " << dtcid << " of vehicle " << oid << std::endl; }

															if ((curpasidx > 0) && (curpasidx < veh->getNrofPassages())) { //there is a next passage
																
																passagePtr nextpas = veh->passages()[curpasidx + 1];
																// update next passage::lagtime; //initialized to predtime!
																double nextpredarrtime = realtime + (nextpas->predtime() - predtime) + predtimeshift;
																nextpas->setLagTime(nextpredarrtime);

																//update internal arrivals;
																int nextsgnr = nextpas->sgnr();
																if (nextsgnr > 0) {
																	auto nextk = (int)floor(nextpredarrtime / settings.controlIntLngth + 1); //rounded downwards
																	auto nextControlInterval = controlIntervals.get(nextk);
																	
																	nextControlInterval->arrival(nextsgnr, nextControlInterval->arrivals(nextsgnr) + 1);
																	nextControlInterval->arrival(0, nextControlInterval->arrivals(0) + 1);

																	//store link arrivals as well (not only queue arrivals)		
																	thisControlInterval->arrivalLNK(nextsgnr, thisControlInterval->arrivalsLNK(nextsgnr) + 1);
																	thisControlInterval->arrivalLNK(0, thisControlInterval->arrivalsLNK(0) + 1);
																}
															}
														}
														else {std::cout << "vehicle not found: " << oid << std::endl;}
														vehOUT << oid << "\t" << dtcid << "\t" << dt->name() << "\t" << dt->jncname() << "\t" << dt->signame() << "\t" << dt->sgnr() << "\t" << predtime << "\t" << lagtime << "\t" << realtime << "\t" << delay << std::endl;
														satflowoutfile << tim << "\t" << (tim + tmptimecor) << "\t" << (tim + tmptimecor + tmptimedetcor) << "\t" << "VEH" << "\t" << oid << "\t" << dtcid << "\t" << dt->name() << "\t" << dt->jncname() << "\t" << dt->signame() << "\t" << dt->sgnr() << std::endl;
													}
												}
											}
										}
										#pragma endregion
									}
									else
									{
										timeSync = tim;
									}
								}
							}
						}
						else if (0 == aimsunMessage.find("RES:")) {
						}
						else if (0 == aimsunMessage.compare("exit")) {
							aimsunWebsocketClient->Disconnect();
							break;
						}
						else {
							throw nullptr;
						}
						aimsunMessages.pop();
					}

					const auto checkStateFrequency = 1s;

					if ((std::chrono::system_clock::now() - oneSecondTLCTime) >= checkStateFrequency)		// run only once every second
					{
						if (lastAimsunMessagesSize > 0)
						{
							std::cout << "Message queue size: " << lastAimsunMessagesSize << std::endl;
						}
						oneSecondTLCTime += checkStateFrequency;
					}
					std::this_thread::sleep_for(1ms);
				}

				if (errorMainProcess) {
					std::cout << "main process error! " << std::endl;
					aimsunWebsocketClient->Disconnect();
					return 1; 
				}
		#pragma endregion

		#pragma region Terminate Connection and Threads

				std::cout << "aimsunWebsocketClient disconnected " << std::endl;

				if (threadPing.joinable())
				{
					threadPing.join();
				}

				if (threadController.joinable())
				{
					threadController.join();
				}

				if (threadControlDecision.joinable())
				{
					threadControlDecision.join();
				}

				if (threadUpdateStatistics.joinable())
				{
					threadUpdateStatistics.join();
				}
			};
		#pragma endregion
		
		#pragma region CloseFiles

		timerfile.close();
		statfile.close();
		statpredfile.close();
		ctrseqfile.close();
		vehOUT.close();
		vehIN.close();
		treefile.close();
		treefileOUT.close();
		basicphasefile.close();
		phasefile.close();
		phasefileOUT.close();
		signalfile.close();
		demandfile.close();
		splitfile.close();
		satflowoutfile.close();
		tmplogfile.close();
		settingsfile.close();

		#pragma endregion
	}
	catch (std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		retVal = 1;
	}
	return retVal;
}

bool sendSGUpdateToAimsun(std::weak_ptr<WebSocketClient> clientPtr, std::string sgAimsunID, std::string nextState)
{
	std::string message = "UPD:{\"OBJ\":\"SIG\",\"OID\":" + sgAimsunID + ",\"STE\":\"" + nextState + "\"}";
	if (auto simulatorWebsocketClientPtr = clientPtr.lock()) {
		simulatorWebsocketClientPtr->Send(message);
		return true;
	}
	else {
		std::cout << "Could not send message to simulator Aimsun" << std::endl;
	}
	return false;
}
