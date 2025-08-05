#include "mapf_instances_logger.h"
#include <unistd.h>


MAPFInstancesLogger::MAPFInstancesLogger(std::string pathTempl) {
	fileID = 0;
	pathTemplate = pathTempl;
	
	// Create logs directory structure
	CreateLogDirectory();
}

void MAPFInstancesLogger::CreateLogDirectory() {
	// Extract experiment name from path template
	std::string experimentName = ExtractExperimentName(pathTemplate);
	
	// Debug output
	std::cout << "DEBUG: Original pathTemplate: " << pathTemplate << std::endl;
	std::cout << "DEBUG: Extracted experimentName: " << experimentName << std::endl;
	
	// Create logs directory path
	std::string logsDir = "task_examples/logs/" + experimentName;
	
	// Get current working directory and adjust to project root
	char cwd[1024];
	if (getcwd(cwd, sizeof(cwd)) != NULL) {
		std::cout << "DEBUG: Current working directory: " << cwd << std::endl;
		// If we're in build directory, go up one level to project root
		std::string cwdStr(cwd);
		if (cwdStr.find("/build") != std::string::npos) {
			cwdStr = cwdStr.substr(0, cwdStr.find("/build"));
			std::cout << "DEBUG: Adjusted to project root: " << cwdStr << std::endl;
		}
		logsDir = cwdStr + "/" + logsDir;
	} else {
		std::cout << "ERROR: Failed to get current working directory" << std::endl;
	}
	
	// Debug output
	std::cout << "DEBUG: Creating directory: " << logsDir << std::endl;
	
	// Create directory if it doesn't exist
	std::string mkdirCmd = "mkdir -p " + logsDir;
	std::cout << "DEBUG: Executing command: " << mkdirCmd << std::endl;
	int result = system(mkdirCmd.c_str());
	if (result != 0) {
		std::cout << "ERROR: Failed to create directory: " << logsDir << std::endl;
		std::cout << "mkdir command: " << mkdirCmd << std::endl;
		std::cout << "system() returned: " << result << std::endl;
		// Try alternative approach
		std::string altCmd = "mkdir -p ./" + logsDir;
		std::cout << "DEBUG: Trying alternative command: " << altCmd << std::endl;
		result = system(altCmd.c_str());
		if (result != 0) {
			std::cout << "ERROR: Alternative command also failed" << std::endl;
		} else {
			std::cout << "SUCCESS: Created directory with alternative command" << std::endl;
		}
	} else {
		std::cout << "SUCCESS: Created directory: " << logsDir << std::endl;
	}
	
	// Update path template to use the new directory
	pathTemplate = logsDir + "/" + ExtractFileName(pathTemplate);
	
	// Debug output
	std::cout << "DEBUG: Updated pathTemplate: " << pathTemplate << std::endl;
}

std::string MAPFInstancesLogger::ExtractExperimentName(const std::string& path) {
	// Extract experiment name from path like "0_task_orca_par_15_15" or "0_task_50"
	std::string fileName = ExtractFileName(path);
	
	// Remove file extension if present
	size_t dotPos = fileName.find_last_of('.');
	if (dotPos != std::string::npos) {
		fileName = fileName.substr(0, dotPos);
	}
	
	// For now, just use the base name without any suffix
	// This will create folders like "0_task", "empty_task_new", etc.
	return fileName;
}

std::string MAPFInstancesLogger::ExtractFileName(const std::string& path) {
	size_t lastSlash = path.find_last_of("/\\");
	if (lastSlash != std::string::npos) {
		return path.substr(lastSlash + 1);
	}
	return path;
}


bool MAPFInstancesLogger::SaveInstance(MAPFActorSet &agents, SubMap &map, MAPFConfig &conf, const std::vector<std::vector<Point>>& globalWaypoints) {

	XMLDocument *docMainXML, *docAgentsXML;
	
	// Use the updated pathTemplate that includes the logs directory
	std::string tmp1 = pathTemplate;
	std::string tmp2 = pathTemplate;
	
	// Add submap global position information
	std::cout << "DEBUG: Saving MAPF instance with enhanced logging" << std::endl;
	std::cout << "DEBUG: Submap origin in global coordinates: (" << map.GetPoint(Node(0,0)).X() << ", " << map.GetPoint(Node(0,0)).Y() << ")" << std::endl;
	std::cout << "DEBUG: Submap size: " << map.GetHeight() << "x" << map.GetWidth() << std::endl;
	std::cout << "DEBUG: Submap cell size: " << map.GetCellSize() << std::endl;


	size_t found = tmp1.find_last_of("/");
	std::string piece = "/MAP_" + std::to_string(fileID) + "_";
	tmp1.erase(found, 1);
	tmp1.insert(found, piece);

	found = tmp2.find_last_of("/");
	piece = "/AGENT_" + std::to_string(fileID) + "_";
	tmp2.erase(found, 1);
	tmp2.insert(found, piece);
	std::string agentSmall = tmp2;
	agentSmall.erase(0, found + 1);


	std::string mainFile = tmp1 + ".xml";
	std::string agentFile = tmp2 + "-1.xml";
	/* MAIN ROOT */
	docMainXML = new XMLDocument();
	auto rootXML = docMainXML->NewElement(CNS_TAG_ROOT);
	docMainXML->InsertFirstChild(rootXML);

	/* MAP */
	auto mapXML = docMainXML->NewElement(CNS_TAG_MAP);
	
	// Add global coordinate information
	auto globalInfoXML = docMainXML->NewElement("global_coordinates");
	Point originPoint = map.GetPoint(Node(0, 0));
	Point topRightPoint = map.GetPoint(Node(0, map.GetWidth() - 1));
	Point bottomLeftPoint = map.GetPoint(Node(map.GetHeight() - 1, 0));
	Point bottomRightPoint = map.GetPoint(Node(map.GetHeight() - 1, map.GetWidth() - 1));
	
	globalInfoXML->SetAttribute("origin_x", originPoint.X());
	globalInfoXML->SetAttribute("origin_y", originPoint.Y());
	globalInfoXML->SetAttribute("top_right_x", topRightPoint.X());
	globalInfoXML->SetAttribute("top_right_y", topRightPoint.Y());
	globalInfoXML->SetAttribute("bottom_left_x", bottomLeftPoint.X());
	globalInfoXML->SetAttribute("bottom_left_y", bottomLeftPoint.Y());
	globalInfoXML->SetAttribute("bottom_right_x", bottomRightPoint.X());
	globalInfoXML->SetAttribute("bottom_right_y", bottomRightPoint.Y());
	globalInfoXML->SetAttribute("cell_size", map.GetCellSize());
	
	mapXML->InsertEndChild(globalInfoXML);
	
	auto gridXML = docMainXML->NewElement(CNS_TAG_GRID);
	gridXML->SetAttribute(CNS_TAG_WIDTH, map.GetWidth());
	gridXML->SetAttribute(CNS_TAG_HEIGHT, map.GetHeight());

	for (size_t i = 0; i < map.GetHeight(); i++) {
		auto rowXML = docMainXML->NewElement(CNS_TAG_ROW);

		std::string rowStr = "";
		for (size_t j = 0; j < map.GetWidth(); j++) {
			if (j != 0) {
				rowStr += " ";
			}

			if (map.CellIsObstacle(i, j)) {
				rowStr += "1";
			}
			else {
				rowStr += "0";
			}
		}

		rowXML->SetText(rowStr.c_str());
		gridXML->InsertEndChild(rowXML);
	}
	mapXML->InsertEndChild(gridXML);
	rootXML->InsertFirstChild(mapXML);
	/* ALGORITHM */
	auto algorithmXML = docMainXML->NewElement(CNS_TAG_ALG);

	auto boolToString = [](bool arg) {
		return (arg) ? "true" : "false";
	};

	std::string algTagName[] =
			{
					"planner",
					"low_level",
					"with_cat",
					"with_perfect_h",
					"with_card_conf",
					"with_bypassing",
					"with_matching_h",
					"with_disjoint_splitting",
					"focal_w",
					"pp_order",
					"parallelize_paths_1",
					"parallelize_paths_2"
			};
	vector<string> algTagText =
			{
					conf.planner,
					conf.lowLevel,
					boolToString(conf.withCAT),
					boolToString(conf.withPerfectHeuristic),
					boolToString(conf.withCardinalConflicts),
					boolToString(conf.withBypassing),
					boolToString(conf.withMatchingHeuristic),
					boolToString(conf.withDisjointSplitting),
					std::to_string(conf.focalW),
					"0",
					boolToString(conf.parallelizePaths1),
					boolToString(conf.parallelizePaths2),

			};

	for (size_t tag = 0; tag < 12; tag++) {
		auto algTagXML = docMainXML->NewElement(algTagName[tag].c_str());
		algTagXML->SetText(algTagText[tag].c_str());
		algorithmXML->InsertEndChild(algTagXML);
	}

	rootXML->InsertEndChild(algorithmXML);

	/* OPTIONS */
	auto optionsXML = docMainXML->NewElement(CNS_TAG_OPT);

	std::string optTagNames[] = {

			"agents_file",
			"tasks_count",
			"maxtime",
			"single_execution"
	};

	std::string optTagText[] = {
			"",
			"1",
			"",
			"false"
	};

	optTagText[0] = agentSmall;
	optTagText[2] = std::to_string(conf.maxTime);

	for (size_t tag = 0; tag < 4; tag++) {
		auto optTagXML = docMainXML->NewElement(optTagNames[tag].c_str());
		optTagXML->SetText(optTagText[tag].c_str());
		optionsXML->InsertEndChild(optTagXML);
	}

	auto rangeXML = docMainXML->NewElement("agents_range");
	rangeXML->SetAttribute("min", agents.getActorCount());
	rangeXML->SetAttribute("max", agents.getActorCount());

	optionsXML->InsertEndChild(rangeXML);

	auto logpathXML = docMainXML->NewElement("logpath");
	auto logfilenameXML = docMainXML->NewElement("logfilename");
	optionsXML->InsertEndChild(logpathXML);
	optionsXML->InsertEndChild(logfilenameXML);
	rootXML->InsertEndChild(optionsXML);


	/* SAVE MAIN*/
	bool resMain = (docMainXML->SaveFile(mainFile.c_str()) == XMLError::XML_SUCCESS);
	docAgentsXML = new XMLDocument();
	rootXML = docAgentsXML->NewElement(CNS_TAG_ROOT);
	docAgentsXML->InsertFirstChild(rootXML);
	for (size_t agent = 0; agent < agents.getActorCount(); agent++) {
		auto agentXML = docAgentsXML->NewElement("agent");
		agentXML->SetAttribute("id", (int) agent);
		agentXML->SetAttribute("start_i", agents.getActor(agent).getStart_i());
		agentXML->SetAttribute("start_j", agents.getActor(agent).getStart_j());
		agentXML->SetAttribute("goal_i", agents.getActor(agent).getGoal_i());
		agentXML->SetAttribute("goal_j", agents.getActor(agent).getGoal_j());
		
		// Add global coordinate information
		Point startGlobal = map.GetPoint(Node(agents.getActor(agent).getStart_i(), agents.getActor(agent).getStart_j()));
		Point goalGlobal = map.GetPoint(Node(agents.getActor(agent).getGoal_i(), agents.getActor(agent).getGoal_j()));
		agentXML->SetAttribute("start_global_x", startGlobal.X());
		agentXML->SetAttribute("start_global_y", startGlobal.Y());
		agentXML->SetAttribute("par_goal_global_x", goalGlobal.X());
		agentXML->SetAttribute("par_goal_global_y", goalGlobal.Y());
		
		// Add global path waypoints information if available
		if (!globalWaypoints.empty() && agent < globalWaypoints.size()) {
			auto waypointsXML = docAgentsXML->NewElement("global_waypoints");
			std::string waypointsStr = "";
			for (size_t i = 0; i < globalWaypoints[agent].size(); ++i) {
				if (i > 0) waypointsStr += " ";
				waypointsStr += "(" + std::to_string(globalWaypoints[agent][i].X()) + "," + std::to_string(globalWaypoints[agent][i].Y()) + ")";
			}
			waypointsXML->SetText(waypointsStr.c_str());
			agentXML->InsertEndChild(waypointsXML);
			
			// Add final goal information
			if (!globalWaypoints[agent].empty()) {
				Point finalGoal = globalWaypoints[agent].back();
				agentXML->SetAttribute("final_goal_global_x", finalGoal.X());
				agentXML->SetAttribute("final_goal_global_y", finalGoal.Y());
			}
		}

		rootXML->InsertEndChild(agentXML);
	}


	bool resAgents = (docAgentsXML->SaveFile(agentFile.c_str()) == XMLError::XML_SUCCESS);

	fileID++;
	return resMain && resAgents;

}

//void MAPFInstancesLogger::AddResults(const MAPFSearchResult &result)
//{
//
//    size_t makespan = 0, timeflow = 0;
//    if(result.pathfound)
//        for (int i = 0; i < result.agentsPaths->size(); i++)
//        {
//            makespan = std::max(makespan, result.agentsPaths->at(i).size() - 1);
//            int lastMove;
//            for (lastMove = result.agentsPaths->at(i).size() - 1; lastMove > 1 && result.agentsPaths->at(i)[lastMove] == result.agentsPaths->at(i)[lastMove - 1]; --lastMove);
//            timeflow += lastMove;
//        }
//    pre_log << result.pathfound << "\t" << result.time << "\t" << makespan << "\t" << timeflow << "\t" << result.HLExpansions << "\t" <<result.HLNodes << "\n";
//
//    return ;
//}

MAPFInstancesLogger::~MAPFInstancesLogger() {
//    pre_log.close();
}

MAPFInstancesLogger::MAPFInstancesLogger(const MAPFInstancesLogger &obj) {
	this->fileID = obj.fileID;
	this->pathTemplate = obj.pathTemplate;
//    this->resPath = obj.resPath;
//    this->pre_log.close();
//    this->pre_log.open(this->resPath);
}

MAPFInstancesLogger &MAPFInstancesLogger::operator=(const MAPFInstancesLogger &obj) {
	if (this != &obj) {
		this->fileID = obj.fileID;
		this->pathTemplate = obj.pathTemplate;
//        this->resPath = obj.resPath;
//        this->pre_log.close();
//        this->pre_log.open(this->resPath);
	}
	return *this;
}


size_t MAPFInstancesLogger::GetFileID() const {
	return fileID;
}