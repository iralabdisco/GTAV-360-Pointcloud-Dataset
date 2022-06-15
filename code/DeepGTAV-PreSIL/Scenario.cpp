#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "Rewarders\GeneralRewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\SpeedRewarder.h"
#include "defaults.h"
#include <time.h>
#include <fstream>
#include <string>
#include <sstream>
#include "Functions.h"
#include "Constants.h"
#include <Eigen/Core>
#include <sstream>
#include "AreaRoaming.h"
#include <chrono>
#include <thread>

extern "C" {
    __declspec(dllimport) int export_get_depth_buffer(void** buf);
}

const float VERT_CAM_FOV = 59; //In degrees
//Need to input the vertical FOV with GTA functions.
//90 degrees horizontal (KITTI) corresponds to 59 degrees vertical (https://www.gtaall.com/info/fov-calculator.html).
const float HOR_CAM_FOV = 90; //In degrees

const int PEDESTRIAN_CLASS_ID = 10;

char* Scenario::weatherList[14] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
char* Scenario::vehicleList[3] = { "blista", "blista", "blista" };//voltic, packer

void Scenario::parseScenarioConfig(const Value& sc, bool setDefaults) {
	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

    //log(std::to_string(setDefaults));

    //log(std::to_string(location.IsArray()));
    //log(std::to_string(location[0].GetFloat()));
    //log(std::to_string(location[1].GetFloat()));
    //log(std::to_string(location[2].GetFloat()));

	if (location.IsArray()) {
		if (!location[0].IsNull()) x = location[0].GetFloat();
		else if (setDefaults) x = 5000 * ((float)rand() / RAND_MAX) - 2500;

		if (!location[1].IsNull()) y = location[1].GetFloat(); 
		else if (setDefaults) y = 8000 * ((float)rand() / RAND_MAX) - 2000;

        if (!location[2].IsNull()) z = location[2].GetFloat();
        else if (setDefaults) z = 0;

        if (!location[3].IsNull()) {
            //log("Location 2 is not null");
            startHeading = location[3].GetFloat();
        }
        else if (setDefaults) {
            //log("Location 3 is NULL");
            startHeading = 0;
        }
	}
	else if (setDefaults) {
		x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}
    
    //log(std::to_string(x));
    //log(std::to_string(y));
    //log(std::to_string(z));
    //log(std::to_string(startHeading));
    

	if (time.IsArray()) {
		if (!time[0].IsNull()) hour = time[0].GetInt();
		else if (setDefaults) hour = rand() % 24;

		if (!time[1].IsNull()) minute = time[1].GetInt();
		else if (setDefaults) minute = rand() % 60;
	}
	else if (setDefaults) {
        hour = 16;//TODO Do we want random times? rand() % 24;
		minute = rand() % 60;
	}

    //log(std::to_string(hour));
    //log(std::to_string(minute));

	if (!weather.IsNull()) _weather = weather.GetString();
    //TODO: Do we want other weather?
    else if (setDefaults) _weather = "CLEAR";// weatherList[rand() % 14];

	if (!vehicle.IsNull()) _vehicle = vehicle.GetString();
    else if (setDefaults) _vehicle = "blazer";// vehicleList[rand() % 3]; //ingot //blazer

	if (drivingMode.IsArray()) {
		if (!drivingMode[0].IsNull()) _drivingMode = drivingMode[0].GetInt();
		else if (setDefaults)  _drivingMode = rand() % 4294967296;
		if (drivingMode[1].IsNull()) _setSpeed = drivingMode[1].GetFloat(); 
		else if (setDefaults) _setSpeed = 1.0*(rand() % 10) + 10;
	}
	else if (setDefaults) {
		_drivingMode = -1;
	}

    //log("speed: " + std::to_string(_setSpeed));
}

void Scenario::parseDatasetConfig(const Value& dc, bool setDefaults) {
	if (!dc["rate"].IsNull()) rate = dc["rate"].GetFloat();
	else if (setDefaults) rate = _RATE_;

    if (!dc["startIndex"].IsNull()) {
        instance_index = dc["startIndex"].GetInt();
        baseTrackingIndex = instance_index;
        series_index = instance_index;
    }

    char temp[] = "%06d";
    char strComp[sizeof temp + 100];
    sprintf(strComp, temp, instance_index);
    instance_string = strComp;
	
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) s_camParams.width = dc["frame"][0].GetInt();
		else if (setDefaults) s_camParams.width = _WIDTH_;

		if (!dc["frame"][1].IsNull()) s_camParams.height = dc["frame"][1].GetInt();
		else if (setDefaults) s_camParams.height = _HEIGHT_;
	}
	else if (setDefaults) {
		s_camParams.width = _WIDTH_;
		s_camParams.height = _HEIGHT_;
	}

    //Need to reset camera params when dataset config is received
    s_camParams.init = false;

	if (!dc["vehicles"].IsNull()) vehicles = dc["vehicles"].GetBool();
	else if (setDefaults) vehicles = _VEHICLES_;

	if (!dc["peds"].IsNull()) peds = dc["peds"].GetBool();
	else if (setDefaults) peds = _PEDS_;

	if (!dc["trafficSigns"].IsNull()) trafficSigns = dc["trafficSigns"].GetBool();
	else if (setDefaults) trafficSigns = _TRAFFIC_SIGNS_;

	if (!dc["direction"].IsNull()) {
		direction = true;
		if (!dc["direction"][0].IsNull()) dir.x = dc["direction"][0].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][1].IsNull()) dir.y = dc["direction"][1].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][2].IsNull()) dir.z = dc["direction"][2].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;
	}
	else if (setDefaults) direction = _DIRECTION_;

	if (dc["reward"].IsArray()) {
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
			reward = true;
		}
		else if (setDefaults) reward = _REWARD_;
	}
	else if (setDefaults) reward = _REWARD_;

	if (!dc["throttle"].IsNull()) throttle = dc["throttle"].GetBool();
	else if (setDefaults) throttle = _THROTTLE_;
	if (!dc["brake"].IsNull()) brake = dc["brake"].GetBool();
	else if (setDefaults) brake = _BRAKE_;
	if (!dc["steering"].IsNull()) steering = dc["steering"].GetBool();
	else if (setDefaults) steering = _STEERING_;
	if (!dc["speed"].IsNull()) speed = dc["speed"].GetBool();
	else if (setDefaults) speed = _SPEED_;
	if (!dc["yawRate"].IsNull()) yawRate = dc["yawRate"].GetBool();
	else if (setDefaults) yawRate = _YAW_RATE_;
	if (!dc["drivingMode"].IsNull()) drivingMode = dc["drivingMode"].GetBool();
	else if (setDefaults) drivingMode = _DRIVING_MODE_;
	if (!dc["location"].IsNull()) location = dc["location"].GetBool();
	else if (setDefaults) location = _LOCATION_;
	if (!dc["time"].IsNull()) time = dc["time"].GetBool();
	else if (setDefaults) time = _TIME_;
    if (!dc["offscreen"].IsNull()) offscreen = dc["offscreen"].GetBool();
    else if (setDefaults) offscreen = _OFFSCREEN_;
    if (!dc["showBoxes"].IsNull()) showBoxes = dc["showBoxes"].GetBool();
    else if (setDefaults) showBoxes = _SHOWBOXES_;
    if (!dc["pointclouds"].IsNull()) pointclouds = dc["pointclouds"].GetBool();
    else if (setDefaults) pointclouds = _POINTCLOUDS_;
    if (!dc["stationaryScene"].IsNull()) stationaryScene = dc["stationaryScene"].GetBool();
    else if (setDefaults) stationaryScene = _STATIONARY_SCENE_;
    if (!dc["collectTracking"].IsNull()) collectTracking = dc["collectTracking"].GetBool();
    else if (setDefaults) collectTracking = _COLLECT_TRACKING_;
    if (!dc["recordScenario"].IsNull()) m_recordScenario = dc["recordScenario"].GetBool();
    else if (setDefaults) m_recordScenario = _RECORD_SCENARIO_;
    if (!dc["positionScenario"].IsNull()) m_positionScenario = dc["positionScenario"].GetBool();
    else if (setDefaults) m_positionScenario = _POSITION_SCENARIO_;

    /*if (DRIVE_SPEC_AREA && !stationaryScene) {
        int startArea = 0;
        dir.x = s_locationBounds[3][0][3];
        dir.y = s_locationBounds[3][1][3];
        dir.z = 0.f;
        x = s_locationBounds[3][0][3];//1,2,3,4,5,6,7,8 are all good
        y = s_locationBounds[3][1][3];//1-0 was last one used for 'good' data
        z = 64.0f;
    }*/

    //log("dir x: "+std::to_string(dir.x));
    //log("dir y: "+std::to_string(dir.y));
    //log("dir z: " + std::to_string(dir.z));

    if (stationaryScene || TRUPERCEPT_SCENARIO) {
        vehiclesToCreate.clear();
        //log("About to get vehicles");
        if (!dc["vehiclesToCreate"].IsNull()) {
            //log("Vehicles non-null");
            const rapidjson::Value& jsonVehicles = dc["vehiclesToCreate"];
            for (rapidjson::SizeType i = 0; i < jsonVehicles.Size(); i++) {
                //log("At least one");
                bool noHit = false;
                VehicleToCreate vehicleToCreate;
                const rapidjson::Value& jVeh = jsonVehicles[i];

                if (!jVeh[0].IsNull()) vehicleToCreate.model = jVeh[0].GetString();
                if (!jVeh[1].IsNull()) vehicleToCreate.forward = jVeh[1].GetFloat();
                if (!jVeh[2].IsNull()) vehicleToCreate.right = jVeh[2].GetFloat();
                if (!jVeh[3].IsNull()) vehicleToCreate.heading = jVeh[3].GetFloat();
                if (!jVeh[4].IsNull()) vehicleToCreate.color = jVeh[4].GetInt();
                if (!jVeh[5].IsNull()) vehicleToCreate.color2 = jVeh[5].GetInt();
                else noHit = true;

                if (!noHit) {
                    //log("Pushing back vehicle");
                    vehiclesToCreate.push_back(vehicleToCreate);
                }
            }
        }
        pedsToCreate.clear();
        //log("About to get ped");
        if (!dc["pedsToCreate"].IsNull()) {
            //log("ped non-null");
            const rapidjson::Value& jsonPeds = dc["pedsToCreate"];
            for (rapidjson::SizeType i = 0; i < jsonPeds.Size(); i++) {
                //log("At least one");
                bool noHit = false;
                PedToCreate pedToCreate;
                const rapidjson::Value& jPed = jsonPeds[i];

                if (!jPed[0].IsNull()) pedToCreate.model = jPed[0].GetInt();
                if (!jPed[1].IsNull()) pedToCreate.forward = jPed[1].GetFloat();
                if (!jPed[2].IsNull()) pedToCreate.right = jPed[2].GetFloat();
                if (!jPed[3].IsNull()) pedToCreate.heading = jPed[3].GetFloat();
                else noHit = true;

                if (!noHit) {
                    //log("Pushing back ped");
                    pedsToCreate.push_back(pedToCreate);
                }
            }
        }
        vehicles_created = false;
    }

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

	if (vehicles) d.AddMember("vehicles", a, allocator);
	if (peds) d.AddMember("peds", a, allocator);
	if (trafficSigns) d.AddMember("trafficSigns", a, allocator);
	if (direction) d.AddMember("direction", a, allocator);
	if (reward) d.AddMember("reward", 0.0, allocator);
	if (throttle) d.AddMember("throttle", 0.0, allocator);
	if (brake) d.AddMember("brake", 0.0, allocator);
	if (steering) d.AddMember("steering", 0.0, allocator);
	if (speed) d.AddMember("speed", 0.0, allocator);
	if (yawRate) d.AddMember("yawRate", 0.0, allocator);
	if (drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (location) d.AddMember("location", a, allocator);
	if (time) d.AddMember("time", 0, allocator);
    d.AddMember("index", 0, allocator);
    d.AddMember("focalLen", 0.0, allocator);
    d.AddMember("curPosition", a, allocator);
    d.AddMember("seriesIndex", a, allocator);

	screenCapturer = new ScreenCapturer(s_camParams.width, s_camParams.height);
}

void Scenario::buildScenario() {
    Vector3 pos, rotation;
    Hash vehicleHash;
    float heading;
    GAMEPLAY::SET_GRAVITY_LEVEL(3);
	ENTITY::DELETE_ENTITY(&m_ownVehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    if (stationaryScene) {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        heading = startHeading;
        std::ostringstream oss;
        oss << "Start heading: " << startHeading;
        std::string str = oss.str();
        vehicles_created = false;
    }


    pos.x = x;
    pos.y = y;
    pos.z = z;
    heading = startHeading;
	//m_ownVehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
	//VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(m_ownVehicle);

	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
        WAIT(0);
	}

	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	//PED::SET_PED_INTO_VEHICLE(ped, m_ownVehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	rotation = ENTITY::GET_ENTITY_ROTATION(ped, 0);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	//if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	//else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, CAM_OFFSET_FORWARD, CAM_OFFSET_UP, TRUE);
    CAM::ATTACH_CAM_TO_ENTITY(camera, ped, 0, 0, 0.8, true);
	CAM::SET_CAM_FOV(camera, VERT_CAM_FOV);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);

    if (stationaryScene) {
        _setSpeed = 0;
    }

    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	AI::CLEAR_PED_TASKS(ped);
    /*
	if (_drivingMode >= 0 && !stationaryScene && !m_positionScenario) {
        if (DRIVE_SPEC_AREA) {
            AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f), vehicleHash, _drivingMode, 0.f, true);

        }
        else {
            AI::TASK_VEHICLE_DRIVE_WANDER(ped, m_ownVehicle, _setSpeed, _drivingMode);
        }
        
    }
    */
    if (m_recordScenario) {
        UNK1::_SET_RECORDING_MODE(1);
    }
    baseFolder = std::string(getenv("DEEPGTAV_EXPORT_DIR")) + "\\" + "object\\";
    CreateDirectory(baseFolder.c_str(), NULL);

    veloFolder = baseFolder + "velodyne\\";
    depthFolder = baseFolder + "depth\\";
    imageFolder = baseFolder + "image\\";
    poseFolder = baseFolder + "pose\\";
    CreateDirectory(veloFolder.c_str(), NULL);
    CreateDirectory(depthFolder.c_str(), NULL);
    CreateDirectory(imageFolder.c_str(), NULL);
    CreateDirectory(poseFolder.c_str(), NULL);

    

    if (!lidar_init) {
        //log("entro nel if");
        setupLiDAR();
        depth_map = new float[s_camParams.width * s_camParams.height];
        m_depth = new float[s_camParams.width * s_camParams.height];
        m_startIndex = instance_index;
    }
    
}

void Scenario::start(const Value& sc, const Value& dc) {
	if (running) return;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, true);
	parseDatasetConfig(dc, true);

	//Build scenario
	buildScenario();


	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;
	running = false;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, false);
	parseDatasetConfig(dc, false);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::run() {
	if (running) {
        if (m_recordScenario) {
            Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(player, 0);
            CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
        }
        //Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(ped, 0);
        //CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
		std::clock_t now = std::clock();

        if (SAME_TIME_OF_DAY) {
            TIME::SET_CLOCK_TIME(hour, minute, 0);
        }

        if (DRIVE_SPEC_AREA && !START_SPEC_AREA) {
            if (pow(currentPos.x - dir.x, 2) + pow(currentPos.y - dir.y, 2) < pow(50, 2))
            {
                std::vector<std::pair<float, float>> new_points = generate_n_random_points(
                    m_startArea, m_polyGrid, 1, 100, { { currentPos.x , currentPos.y } });
                dir.x = new_points[0].first;
                dir.y = new_points[0].second;

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
                    GAMEPLAY::GET_HASH_KEY((char*)_vehicle), _drivingMode, 1.f, true);
            }
            else if (!in_bounds(currentPos.x, currentPos.y, m_startArea, m_polyGrid))
            {
                std::vector<std::pair<float, float>> new_points = generate_n_random_points(
                    m_startArea, m_polyGrid, 1, 100, { { currentPos.x , currentPos.y } });
                dir.x = new_points[0].first;
                dir.y = new_points[0].second;

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
                    GAMEPLAY::GET_HASH_KEY((char*)_vehicle), _drivingMode, 1.f, true);
            }
        }
		if (_drivingMode < 0) {
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		
		float delay = ((float)(now - lastSafetyCheck)) / CLOCKS_PER_SEC;
		if (delay > 10) {
            //Need to delay first camera parameters being set so native functions return correct values
            if (!s_camParams.firstInit) {
                s_camParams.init = false;
                setCamParams();
                s_camParams.firstInit = true;
            }

			lastSafetyCheck = std::clock();
			//Avoid bad things such as getting killed by the police, robbed, dying in car accidents or other horrible stuff
			PLAYER::SET_EVERYONE_IGNORE_PLAYER(player, TRUE);
			PLAYER::SET_POLICE_IGNORE_PLAYER(player, TRUE);
			PLAYER::CLEAR_PLAYER_WANTED_LEVEL(player); // Never wanted

			// Put on seat belt
			PED::SET_PED_CONFIG_FLAG(ped, 32, FALSE);

			// Invincible vehicle
			//VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(m_ownVehicle, FALSE);
			//VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(m_ownVehicle, FALSE);
			//VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(m_ownVehicle, TRUE);

			//VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(m_ownVehicle, FALSE);
			//ENTITY::SET_ENTITY_INVINCIBLE(m_ownVehicle, TRUE);
			//ENTITY::SET_ENTITY_PROOFS(m_ownVehicle, 1, 1, 1, 1, 1, 1, 1, 1);

			// Player invincible
			PLAYER::SET_PLAYER_INVINCIBLE(player, TRUE);

			// Driving characteristics
			PED::SET_DRIVER_AGGRESSIVENESS(ped, 0.0);
			PED::SET_DRIVER_ABILITY(ped, 100.0);
		}
	}
	scriptWait(0);
}

void Scenario::stop() {
	if (!running) return;
	running = false;
	CAM::DESTROY_ALL_CAMS(TRUE);
	CAM::RENDER_SCRIPT_CAMS(FALSE, TRUE, 500, FALSE, FALSE);
	AI::CLEAR_PED_TASKS(ped);
	setCommands(0.0, 0.0, 0.0);
}

void Scenario::setCommands(float throttle, float brake, float steering) {
	currentThrottle = throttle;
	currentBrake = brake;
	currentSteering = steering;
}

StringBuffer Scenario::generateMessage() {
	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);
	
    std::string line, data;
    std::vector<float> row;
    log(std::to_string(series_index));
    log("About to pause game");
    GAMEPLAY::SET_GAME_PAUSED(true);
    GAMEPLAY::SET_TIME_SCALE(0.0f);


    /*
    Vector3 vehicle_position = ENTITY::GET_ENTITY_COORDS(m_ownVehicle, true);
    Vector3 vehicle_rotation = ENTITY::GET_ENTITY_ROTATION(m_ownVehicle, 0);
    std::string baseFolder1 = std::string(getenv("DEEPGTAV_EXPORT_DIR")) + "\\";
    std::string file1 = baseFolder1 + "object\\" + "vehicle_pose.txt";
    FILE* f1 = fopen(file1.c_str(), "a");
    std::ostringstream oss1;
    oss1 << vehicle_position.x << ", " << vehicle_position.y << ", " << vehicle_position.z << ", " << vehicle_rotation.x << ", " << vehicle_rotation.y << ", " << vehicle_rotation.z;
    std::string str1 = oss1.str();
    fprintf(f1, str1.c_str());
    fprintf(f1, "\n");
    fclose(f1);
    */

    
    if (!pose_initialized) {
        std::string baseFolder_t = std::string(getenv("DEEPGTAV_EXPORT_DIR")) + "\\";
        std::string file_location = baseFolder_t + "object\\" + "location.txt";
        file.open(file_location.c_str(), std::ios::in);
        pose_initialized = true;
    }
    
    
    if (file.is_open()) {
        while (std::getline(file, line)) {
            row.clear();
            std::stringstream str(line);
            while (std::getline(str, data, ','))
                row.push_back(atof(data.c_str()));
            // get vehicle position to teleport
            vehicle_position.x = row[1];
            vehicle_position.y = row[2];
            vehicle_position.z = row[3];
            // get vehicle rotation
            vehicle_rotation.x = 0.0f;
            vehicle_rotation.y = 0.0f;
            vehicle_rotation.z = 0.0f;
            default_rotation.x = row[4];
            default_rotation.y = row[5];
            default_rotation.z = row[6];
            
            real_rotation.x = -1; real_rotation.y = -1, real_rotation.z = -1;
            GAMEPLAY::SET_GAME_PAUSED(false);
            

            
            //while (std::abs(real_rotation.x) >= 0.005 && std::abs(real_rotation.y) >= 0.005 && std::abs(real_rotation.z) >= 0.005) {
            ENTITY::SET_ENTITY_COORDS(ped, vehicle_position.x, vehicle_position.y, vehicle_position.z, 1, 0, 0, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            scriptWait(0);
            ENTITY::SET_ENTITY_ROTATION(ped, vehicle_rotation.y, vehicle_rotation.x, vehicle_rotation.z, 0, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            scriptWait(0);
            GAMEPLAY::CLEAR_AREA(vehicle_position.x, vehicle_position.y, vehicle_position.z, 200.0, true, false, false, false);
            //VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(m_ownVehicle);
            //scriptWait(0);
            real_rotation = ENTITY::GET_ENTITY_ROTATION(ped, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            vehicle_position.z += 0.1f;

            std::ostringstream l;
            l << "Initial Position Setting: \n";
            l << "position: " << vehicle_position.x << " | " << vehicle_position.y << " | " << vehicle_position.z << "\n";
            l << "real_rotation: " << real_rotation.x << " | " << real_rotation.y << " | " << real_rotation.z << "\n";
            std::string l1 = l.str();
            log(l1);
            //}
            
            /*
            setFilenames(0);
            ENTITY::SET_ENTITY_COORDS(m_ownVehicle, vehicle_position.x, vehicle_position.y, vehicle_position.z, 1, 0, 0, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            scriptWait(0);
            ENTITY::SET_ENTITY_ROTATION(m_ownVehicle, default_rotation.y, default_rotation.x, default_rotation.z, 0, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            scriptWait(0);
            Any notification = UI::_GET_ACTIVE_NOTIFICATION_HANDLE();
            log(notification);
            UI::_REMOVE_NOTIFICATION(notification);
            GAMEPLAY::CLEAR_AREA(vehicle_position.x, vehicle_position.y, vehicle_position.z, 200.0, true, false, false, false);
            scriptWait(0);
            setPosition();
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));
            GAMEPLAY::SET_GAME_PAUSED(true);
            setRenderingCam(m_ownVehicle, CAM_OFFSET_UP, CAM_OFFSET_FORWARD, 0);
            setCamParams();
            setPosition();
            capture();
            exportImage(screenCapturer->pixels);
            series_index++;
            char temp[] = "%06d";
            char strComp[sizeof temp + 100];
            sprintf(strComp, temp, series_index);
            instance_string = strComp;
            */
            /*
            std::string file1 = baseFolder + "location.txt";
            FILE* f1 = fopen(file1.c_str(), "a");
            std::ostringstream oss1;
            log(series_index);
            oss1 << series_index << ", " << vehicle_position.x << ", " << vehicle_position.y << ", " << vehicle_position.z << ", " << default_rotation.x << ", " << default_rotation.y << ", " << default_rotation.z;
            std::string str1 = oss1.str();
            fprintf(f1, str1.c_str());
            fprintf(f1, "\n");
            fclose(f1);
            */
            
            int depthSize = -1;

            for (int i = 1; i <= 4; i++) {

                setFilenames(i);
                log(std::string("SETTORE: " + std::to_string(i)));
                changeVehicleOriantation(ped, i);
                scriptWait(0);
                setRenderingCam(ped, CAM_OFFSET_UP, CAM_OFFSET_FORWARD, i);
                scriptWait(0);
                setCamParams();
                scriptWait(0);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                setPosition();
                //exportPose();

                capture();
                //exportImage(screenCapturer->pixels);

                depthSize = setDepthBuffer();

                if (direction) setDirection();
                if (reward) setReward();
                if (throttle) setThrottle();
                if (brake) setBrake();
                if (steering) setSteering();

                if (depthSize != -1 && lidar_init) {
                    saveAllData(depth_map);
                }
                else {
                    log("ERROR: Depth buffer could not be properly set!!!!!!!!!!!!!!!!!!!!!!", true);
                    log(std::to_string(lidar_init));
                }

                if (i == 4) {
                    //CAM::SET_CAM_ROT(camera, 0.0f, 0.0f, default_rotation.z, 0);
                    //scriptWait(0);
                    //capture();
                    //exportImage(screenCapturer->pixels);
                    series_index++;
                    char temp[] = "%06d";
                    char strComp[sizeof temp + 100];
                    sprintf(strComp, temp, series_index);
                    instance_string = strComp;
                }

            }
            
        
        log("End Frame\n\n");
        }
    }
    else {
        log("Could not open the file");
    }
    
    GAMEPLAY::SET_GAME_PAUSED(false);
    GAMEPLAY::SET_TIME_SCALE(1.0f);

	d.Accept(writer);

	return buffer;
}

void Scenario::saveAllData(float* depth) {
    m_depth = depth;
    //exportDepth(m_depth);
    //exportImage(screenCapturer->pixels);
    exportPose();
    collectLiDAR(m_depth);
}

void Scenario::setCameraRotation(int sector) {
    Vector3 rot; rot.x = 0.0f; rot.y = 0.0f;
    //Vector3 rot = old_cam_rotation;
    switch (sector)
    {
    case 1: {
        rot.z = 0.0f;
        break;
    }
    case 2: {
        rot.z = 90.0f;
        break;
    }
    case 3: {
        rot.z = -180.0f;
        break;
    }
    case 4: {
        rot.z = -90.0f;
        break;
    }
    default:
        break;
    }
    CAM::SET_CAM_ROT(camera, rot.y, rot.x, rot.z, 0);
    //old_cam_rotation = CAM::GET_CAM_ROT(camera, 0);
    scriptWait(0);
}

void Scenario::changeVehicleOriantation(Ped v, int sector) {
    Vector3 rot; rot.x = 0.0f; rot.y = 0.0f;
    GAMEPLAY::SET_GAME_PAUSED(false);
    switch (sector)
    {
    case 1: {
        rot.z = 0.0f;
        break;
    }
    case 2: {
        rot.z =  90.0f;
        break;
    }
    case 3: {
        rot.z = - 180.0f;
        break;
    }
    case 4: {
        rot.z = -90.0f;
        break;
    }
    default:
        break;
    }
    ENTITY::SET_ENTITY_COORDS(v, vehicle_position.x, vehicle_position.y, (vehicle_position.z), 1, 0, 0, 1);
    scriptWait(0);
    ENTITY::SET_ENTITY_ROTATION(v, rot.x, rot.y, rot.z, 0, true);
    scriptWait(0);
    UI::_REMOVE_NOTIFICATION(UI::_GET_ACTIVE_NOTIFICATION_HANDLE());
    scriptWait(0);
    GAMEPLAY::SET_GAME_PAUSED(true);
}

void Scenario::setRenderingCam(Ped v, int height, int length, int sector) {
    Vector3 position;
    Vector3 fVec, rVec, uVec;
    Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(v, 0);
    ENTITY::GET_ENTITY_MATRIX(v, &fVec, &rVec, &uVec, &position);

    Vector3 offset;
    offset.x = 0;
    offset.y = length / 2;
    offset.z = height;
    Vector3 offsetWorld = camToWorld(offset, fVec, rVec, uVec);
    //Since it's offset need to subtract the cam position
    offsetWorld.x -= s_camParams.pos.x;
    offsetWorld.y -= s_camParams.pos.y;
    offsetWorld.z -= s_camParams.pos.z;

    GAMEPLAY::SET_TIME_SCALE(0.0f);
    GAMEPLAY::SET_GAME_PAUSED(false);
    GAMEPLAY::SET_TIME_SCALE(0.0f);
    CAM::SET_CAM_COORD(camera, position.x + offsetWorld.x, position.y + offsetWorld.y, position.z + offsetWorld.z + 0.2f);
    //CAM::SET_CAM_COORD(camera, position.x, position.y, (position.z + 1));
    //setCameraRotation(sector);
    CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
    scriptWait(0);
    GAMEPLAY::SET_GAME_PAUSED(true);

    GRAPHICS::DRAW_LINE(position.x, position.y, position.z, position.x + 2, position.y, position.z, 255, 0, 0, 255);
    GRAPHICS::DRAW_LINE(position.x, position.y, position.z, position.x, position.y + 2, position.z, 0, 255, 0, 255);
    GRAPHICS::DRAW_LINE(position.x, position.y, position.z, position.x, position.y, position.z + 2, 0, 0, 255, 255);

    std::ostringstream oss;
    oss << "EntityID/position/rotation: " << v << "\n" <<
        position.x << ", " << position.y << ", " << position.z <<
        "\n" << rotation.x << ", " << rotation.y << ", " << rotation.z <<
        "\nOffset: " << offset.x << ", " << offset.y << ", " << offset.z <<
        "\nOffsetworld: " << offsetWorld.x << ", " << offsetWorld.y << ", " << offsetWorld.z;
    log(oss.str(), true);
}

void Scenario::capture() {
    //Time synchronization seems to be correct with 2 render calls
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);
    screenCapturer->capture();
    scriptWait(0);
}

void Scenario::setThrottle(){
	d["throttle"] = getFloatValue(m_ownVehicle, 0x92C);
}

void Scenario::setBrake(){
	d["brake"] = getFloatValue(m_ownVehicle, 0x930);
}

void Scenario::setSteering(){
	d["steering"] = -getFloatValue(m_ownVehicle, 0x924) / 0.6981317008;
}

void Scenario::setDirection(){
	int direction;
	float distance;
	Vehicle temp_vehicle;
	Document::AllocatorType& allocator = d.GetAllocator();
	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &temp_vehicle, &distance);
	Value _direction(kArrayType);
	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
	d["direction"] = _direction;
}

void Scenario::setReward() {
	d["reward"] = rewarder->computeReward(m_ownVehicle);
}

static int bike_num = 0;

void Scenario::createVehicle(const char* model, float relativeForward, float relativeRight, float heading, int color, int color2) {
    Hash vehicleHash = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(model));
    Vector3 pos;
    pos.x = currentPos.x + currentForwardVector.x * relativeForward + currentRightVector.x * relativeRight;
    pos.y = currentPos.y + currentForwardVector.y * relativeForward + currentRightVector.y * relativeRight;
    pos.z = currentPos.z + currentForwardVector.z * relativeForward + currentRightVector.z * relativeRight;
    STREAMING::REQUEST_MODEL(vehicleHash);
    while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    Vehicle tempV = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
    WAIT(0);
    if (color != -1) {
        VEHICLE::SET_VEHICLE_COLOURS(tempV, color, color2);
    }
    //VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(tempV);

    ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&tempV);
}

void Scenario::createPed(int model, float relativeForward, float relativeRight, float heading, int task) {
    //Ped hashes found at: https://www.se7ensins.com/forums/threads/request-pc-ped-hashes.1317848/
    Hash hash = 0x505603B9;// GAMEPLAY::GET_HASH_KEY(const_cast<char*>(model));
    Vector3 pos;
    pos.x = currentPos.x + currentForwardVector.x * relativeForward + currentRightVector.x * relativeRight;
    pos.y = currentPos.y + currentForwardVector.y * relativeForward + currentRightVector.y * relativeRight;
    pos.z = currentPos.z + currentForwardVector.z * relativeForward + currentRightVector.z * relativeRight;
    STREAMING::REQUEST_MODEL(hash);
    while (!STREAMING::HAS_MODEL_LOADED(hash)) WAIT(0);
    Ped temp = PED::CREATE_PED(4, hash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
    WAIT(0);
    AI::TASK_WANDER_STANDARD(ped, 10.0f, 10);
    ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&temp);
}

void Scenario::createVehicles() {
    setPosition();
    if ((stationaryScene || TRUPERCEPT_SCENARIO) && !vehicles_created) {
        log("Creating peds");
        for (int i = 0; i < pedsToCreate.size(); i++) {
            PedToCreate p = pedsToCreate[i];
            createPed(p.model, p.forward, p.right, p.heading, i);
        }
        log("Creating vehicles");
        for (int i = 0; i < vehiclesToCreate.size(); i++) {
            VehicleToCreate v = vehiclesToCreate[i];
            createVehicle(v.model.c_str(), v.forward, v.right, v.heading, v.color, v.color2);
        }
        vehicles_created = true;
    }
}

//Saves the position and vectors of the capture vehicle
void Scenario::setPosition() {
    //NOTE: The forward and right vectors are swapped (compared to native function labels) to keep consistency with coordinate system
    ENTITY::GET_ENTITY_MATRIX(ped, &m_camForwardVector, &m_camRightVector, &m_camUpVector, &currentPos);
    ENTITY::GET_ENTITY_MATRIX(ped, &currentForwardVector, &currentRightVector, &currentUpVector, &currentPos); //Blue or red pill
}

int Scenario::setDepthBuffer(bool prevDepth) {
    //log("About to get depth buffer");
    int size = export_get_depth_buffer((void**)&depth_map);
    scriptWait(0);
    //std::ostringstream oss;
    //oss << "Depth buffer size: " << size <<std::endl;
    //oss << *(depth_map) << std::endl;
    //log(oss.str(), true);

    log("get depth buffer");
    return size;
}

void Scenario::exportDepth(float* depth) {
    int size_t = s_camParams.width * s_camParams.height;
    FILE* ofile = fopen(depthFile.c_str(), "wb");
    fwrite(depth, sizeof(float), size_t, ofile);
    fclose(ofile);
    scriptWait(0);
}

void Scenario::drawBoxes(Vector3 BLL, Vector3 FUR, Vector3 dim, Vector3 upVector, Vector3 rightVector, Vector3 forwardVector, Vector3 position, int colour) {
    //log("Inside draw boxes");
    if (showBoxes) {
        log("Inside show boxes");
        Vector3 edge1 = BLL;
        Vector3 edge2;
        Vector3 edge3;
        Vector3 edge4;
        Vector3 edge5 = FUR;
        Vector3 edge6;
        Vector3 edge7;
        Vector3 edge8;

        int green = colour * 255;
        int blue = abs(colour - 1) * 255;

        edge2.x = edge1.x + 2 * dim.y*rightVector.x;
        edge2.y = edge1.y + 2 * dim.y*rightVector.y;
        edge2.z = edge1.z + 2 * dim.y*rightVector.z;

        edge3.x = edge2.x + 2 * dim.z*upVector.x;
        edge3.y = edge2.y + 2 * dim.z*upVector.y;
        edge3.z = edge2.z + 2 * dim.z*upVector.z;

        edge4.x = edge1.x + 2 * dim.z*upVector.x;
        edge4.y = edge1.y + 2 * dim.z*upVector.y;
        edge4.z = edge1.z + 2 * dim.z*upVector.z;

        edge6.x = edge5.x - 2 * dim.y*rightVector.x;
        edge6.y = edge5.y - 2 * dim.y*rightVector.y;
        edge6.z = edge5.z - 2 * dim.y*rightVector.z;

        edge7.x = edge6.x - 2 * dim.z*upVector.x;
        edge7.y = edge6.y - 2 * dim.z*upVector.y;
        edge7.z = edge6.z - 2 * dim.z*upVector.z;

        edge8.x = edge5.x - 2 * dim.z*upVector.x;
        edge8.y = edge5.y - 2 * dim.z*upVector.y;
        edge8.z = edge5.z - 2 * dim.z*upVector.z;

        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 0, green, blue, 200);

        GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);

        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 0, green, blue, 200);
        WAIT(0);
    }
}

void Scenario::setCamParams() {
    //These values stay the same throughout a collection period
    if (!s_camParams.init) {
        s_camParams.nearClip = CAM::GET_CAM_NEAR_CLIP(camera);
        s_camParams.farClip = CAM::GET_CAM_FAR_CLIP(camera);
        s_camParams.fov = CAM::GET_CAM_FOV(camera);
        s_camParams.ncHeight = 2 * s_camParams.nearClip * tan(s_camParams.fov / 2. * (PI / 180.)); // field of view is returned vertically
        s_camParams.ncWidth = s_camParams.ncHeight * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
        s_camParams.init = true;

        if (m_recordScenario) {
            float gameFC = CAM::GET_CAM_FAR_CLIP(camera);
            std::ostringstream oss;
            oss << "NC, FC (gameFC), FOV: " << s_camParams.nearClip << ", " << s_camParams.farClip << " (" << gameFC << "), " << s_camParams.fov;
            std::string str = oss.str();
            log(str, true);
        }
    }

    //These values change frame to frame
    s_camParams.theta = CAM::GET_CAM_ROT(camera, 0);
    s_camParams.pos = CAM::GET_CAM_COORD(camera);

    std::ostringstream oss1;
    oss1 << "\ns_camParams.pos X: " << s_camParams.pos.x << " Y: " << s_camParams.pos.y << " Z: " << s_camParams.pos.z <<
        "\nvehicle.pos X: " << currentPos.x << " Y: " << currentPos.y << " Z: " << currentPos.z <<
        "\nfar: " << s_camParams.farClip << " nearClip: " << s_camParams.nearClip << " fov: " << s_camParams.fov <<
        "\nrotation gameplay: " << s_camParams.theta.x << " Y: " << s_camParams.theta.y << " Z: " << s_camParams.theta.z <<
        "\n AspectRatio: " << GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
    std::string str1 = oss1.str();
    log(str1);

    //For optimizing 3d to 2d and unit vector to 2d calculations
    s_camParams.eigenPos = Eigen::Vector3f(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z);
    s_camParams.eigenRot = Eigen::Vector3f(s_camParams.theta.x, s_camParams.theta.y, s_camParams.theta.z);
    s_camParams.eigenTheta = (PI / 180.0) * s_camParams.eigenRot;
    s_camParams.eigenCamDir = rotate(WORLD_NORTH, s_camParams.eigenTheta);
    s_camParams.eigenCamUp = rotate(WORLD_UP, s_camParams.eigenTheta);
    s_camParams.eigenCamEast = rotate(WORLD_EAST, s_camParams.eigenTheta);
    s_camParams.eigenClipPlaneCenter = s_camParams.eigenPos + s_camParams.nearClip * s_camParams.eigenCamDir;
    s_camParams.eigenCameraCenter = -s_camParams.nearClip * s_camParams.eigenCamDir;
    
    s_camParams.pos.x = s_camParams.pos.x + CAM_OFFSET_FORWARD * currentForwardVector.x + CAM_OFFSET_UP * currentUpVector.x;
    s_camParams.pos.y = s_camParams.pos.y + CAM_OFFSET_FORWARD * currentForwardVector.y + CAM_OFFSET_UP * currentUpVector.y;
    s_camParams.pos.z = s_camParams.pos.z + CAM_OFFSET_FORWARD * currentForwardVector.z + CAM_OFFSET_UP * currentUpVector.z;

    //For measuring height of camera (LiDAR) to ground plane
    /*float groundZ;
    GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, &(groundZ), 0);
    
    std::ostringstream oss;
    oss << "LiDAR height: " << s_camParams.pos.z - groundZ;
    std::string str = oss.str();
    log(str);*/
}

void Scenario::setupLiDAR() {
    if (pointclouds && !lidar_init) //flag if activate the LiDAR
    {
        //Specs on Velodyne HDL-64E
        //0.09f azimuth resolution
        //26.8 vertical fov (+2 degrees up to -24.8 degrees down)
        //0.420 vertical resolution
        lidar.Init3DLiDAR_FOV(MAX_LIDAR_DIST, 90.0f, 0.09f, 26.9f, 0.420f, 8.0f);
        //lidar.Init3DLiDAR_FOV(MAX_LIDAR_DIST, 90.0f, 0.4f, 30.0f, 2.0f, 15.0f);
        lidar.AttachLiDAR2Camera(camera, ped);
        lidar_init = true;
        log("init lidar");
    }
}

void Scenario::collectLiDAR(float* depth) {

    log("Starting collecting LiDAR data.");

    m_entitiesHit.clear();
    lidar.updateCurrentPosition(m_camForwardVector, m_camRightVector, m_camUpVector);
    float* pointCloud = lidar.GetPointClouds(pointCloudSize, &m_entitiesHit, lidar_param, depth, ped);
    scriptWait(0);
    //float* pointCloud = lidar.GetPointClouds(pointCloudSize, &m_entitiesHit, lidar_param, m_pDepth, m_pInstanceSeg, camera);
    //log("Saving LiDAR data.");
    std::ofstream ofile(veloFile, std::ios::binary);
    ofile.write((char*)pointCloud, FLOATS_PER_POINT * sizeof(float) * pointCloudSize);
    ofile.close();

    log("Finishing collecting LiDAR data.");

}

std::string Scenario::getStandardFilename(std::string subDir, std::string extension, int type) {
    std::string filename;
    //Type: 0(null), 1(velodyne), 2(depth), 3(images)
    switch (type)
    {
    case 0: {
        filename = baseFolder;
        CreateDirectory(filename.c_str(), NULL);
        filename.append(subDir);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        break;
    }
    case 1: {
        filename = veloFolder;
        CreateDirectory(filename.c_str(), NULL);
        filename.append(subDir);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        break;
    }
    case 2: {
        filename = depthFolder;
        CreateDirectory(filename.c_str(), NULL);
        filename.append(subDir);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        break;
    }
    case 3: {
        filename = imageFolder;
        CreateDirectory(filename.c_str(), NULL);
        filename.append(subDir);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        break;
    }
    case 4: {
        filename = poseFolder;
        CreateDirectory(filename.c_str(), NULL);
        filename.append(subDir);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        break;
    }
    default:
        break;
    }

    filename.append("\\");
    filename.append(instance_string);
    filename.append(extension);
    return filename;
}

void Scenario::setFilenames(int sector) {
    //These are standard files
    switch (sector)
    {
    case 1: {
        //imageFile = getStandardFilename("image_1", ".png", 3);
        veloFile = getStandardFilename("velodyne_1", ".bin", 1);
        //depthFile = getStandardFilename("depth_1", ".bin", 2);
        poseFile = getStandardFilename("pose_1", ".txt", 4);
        break;
    }
    case 2: {
        //imageFile = getStandardFilename("image_2", ".png", 3);
        veloFile = getStandardFilename("velodyne_2", ".bin", 1);
        //depthFile = getStandardFilename("depth_2", ".bin", 2);
        poseFile = getStandardFilename("pose_2", ".txt", 4);
        break;
    }
    case 3: {
        //imageFile = getStandardFilename("image_3", ".png", 3);
        veloFile = getStandardFilename("velodyne_3", ".bin", 1);
        //depthFile = getStandardFilename("depth_3", ".bin", 2);
        poseFile = getStandardFilename("pose_3", ".txt", 4);
        break;
    }
    case 4: {
        //imageFile = getStandardFilename("image_4", ".png", 3);
        veloFile = getStandardFilename("velodyne_4", ".bin", 1);
        //depthFile = getStandardFilename("depth_4", ".bin", 2);
        poseFile = getStandardFilename("pose_4", ".txt", 4);
        break;
    }

    default:
        break;
    }

    imageFile = getStandardFilename("image", ".png", 0);
    //trashFile = getStandardFilename("trash", ".bin", 0);
    scriptWait(0);
}

void Scenario::exportImage(BYTE* data, std::string filename) {
    cv::Mat tempMat(cv::Size(s_camParams.width, s_camParams.height), CV_8UC3, data);
    if (filename.empty()) {
        filename = imageFile;
    }
    cv::imwrite(filename, tempMat);
    tempMat.release();
}

void Scenario::exportPose() {
    Vector3 posEntity, rightVec, forwardVec, upVec;
    Vector3 camPos, rCam, fCam, upCam;
    Vector3 vRot, cRot;
    //ENTITY::GET_ENTITY_MATRIX(m_ownVehicle, &forwardVec, &rightVec, &upVec, &posEntity);
    //ENTITY::GET_ENTITY_MATRIX(camera, &fCam, &rCam, &upCam, &camPos);
    posEntity = ENTITY::GET_ENTITY_COORDS(player, true);
    vRot = ENTITY::GET_ENTITY_ROTATION(player, 0);
    cRot = CAM::GET_CAM_ROT(camera, 0);
    camPos = CAM::GET_CAM_COORD(camera);

    FILE* f_pose = fopen(poseFile.c_str(), "w");
    std::ostringstream oss;

    oss << posEntity.x << "," << posEntity.y << "," << posEntity.z << "\n";
    oss << vRot.x << "," << vRot.y << "," << vRot.z << "\n";
    oss << camPos.x << "," << camPos.y << "," << camPos.z << "\n";
    oss << cRot.x << "," << cRot.y << "," << cRot.z << "\n";
    oss << "0.0,0.0," << default_rotation.z << "\n";
    //oss << camPos.x << "," << camPos.y << "," << camPos.z << "\n";
    //oss << fCam.x << "," << fCam.y << "," << fCam.z << "\n";
    //oss << rCam.x << "," << rCam.y << "," << rCam.z << "\n";
    //oss << upCam.x << "," << upCam.y << "," << upCam.z << "\n";
    //oss << cRot.x << "," << cRot.y << "," << cRot.z << "\n";
    
    std::string str = oss.str();
    fprintf(f_pose, str.c_str());
    fclose(f_pose);
}