#pragma once

#include <stdlib.h>
#include <ctime>

#include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"

#include "ScreenCapturer.h"
#include "Rewarders\Rewarder.h"
#include "LiDAR.h"
#include "Functions.h"
#include "CamParams.h"
#include <memory>
#include "ObjectDetection.h"
#include "Constants.h"

using namespace rapidjson;

//#define DEBUG 1

class Scenario {
private:
    //std::unique_ptr<ObjectDetection> m_pObjDet = NULL;

	static char* weatherList[14];
	static char* vehicleList[3];

	Vehicle m_ownVehicle = NULL;
	Player player = NULL;
	Ped ped = NULL;
	Cam camera = NULL;
	Vector3 dir;

	float x, y, z;
    float startHeading;
	int hour, minute;
	const char* _weather;
	const char* _vehicle;

	bool vehicles;
	bool peds;
	bool trafficSigns; //TODO
	bool direction;
	bool reward;
	bool throttle;
	bool brake;
	bool steering;
	bool speed;
	bool yawRate;
	bool drivingMode; //TODO
	bool location;
	bool time;
    bool offscreen;
    bool showBoxes;
    bool pointclouds;
    bool stationaryScene;

	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

    Vector3 currentPos;
    Vector3 currentForwardVector;
    Vector3 currentUpVector;
    Vector3 currentRightVector;

	Rewarder* rewarder;
	std::clock_t lastSafetyCheck;
	int _drivingMode;
	float _setSpeed;

	bool running = false;
	Document d;

    int m_startArea = 3; //Downtown (see s_locationBounds)
    std::vector<std::vector<char>> m_polyGrid;

    //Depth Map variables
    float* m_depth = nullptr;
    float* depth_map = nullptr;
    uint8_t* m_stencilBuffer = nullptr;
    unsigned char* color_buf = NULL;

    bool vehicles_created = false;
    std::vector<VehicleToCreate> vehiclesToCreate;
    std::vector<PedToCreate> pedsToCreate;

    std::string depthFile;
    std::string poseFile;
    std::string veloFile;
    std::string imageFile;
    std::string calibFile;
    std::string trashFile;

    std::string baseFolder;
    std::string veloFolder;
    std::string depthFolder;
    std::string imageFolder;
    std::string poseFolder;
    std::string trashFolder;

    LiDAR lidar;
    bool lidar_init = false;
    int pointCloudSize = 0;
    int lidar_param = 7;
    std::unordered_map<int, HitLidarEntity*> m_entitiesHit;

    Vector3 m_camForwardVector;
    Vector3 m_camRightVector;
    Vector3 m_camUpVector;

    bool first_buffer = true;

public:
	float rate;

	void start(const Value& sc, const Value& dc);
	void stop();
	void config(const Value& sc, const Value& dc);
	void setCommands(float throttle, float brake, float steering);
	void run();

    //Depth buffer fn/var needs to be accessed by server
    int setDepthBuffer(bool prevDepth = false);
    bool m_prevDepth = false;

	ScreenCapturer* screenCapturer;
	StringBuffer generateMessage();

    void setRenderingCam(Ped v, int height, int length, int sector);
    void generateSecondaryPerspectives();
    void generateSecondaryPerspective(ObjEntity vInfo);
    void capture();

    int instance_index;
    int series_index;
    std::string series_string = "0000";
    std::string instance_string;
    int baseTrackingIndex = instance_index;
    int m_startIndex;

    //Tracking variables
    bool collectTracking;
    //# of instances in one series
    const int trSeriesLength = 500;
    //# of seconds between series
    const int trSeriesGapTime = 30;
    //Used for keeing track of when to add the gap
    bool trSeriesGap = false;

    //Mode for recording clips (data not generated)
    bool m_recordScenario;
    //Mode for outputting current position/heading (data not generated)
    bool m_positionScenario;

    Vector3 vehicle_position;
    Vector3 vehicle_rotation;
    Vector3 default_rotation;
    Vector3 real_rotation;
    bool pose_initialized = false;
    std::fstream file;

    void setFilenames(int sector);


private:
	void parseScenarioConfig(const Value& sc, bool setDefaults);
	void parseDatasetConfig(const Value& dc, bool setDefaults);
	void buildScenario();
    void exportCalib(std::string calibFile);
	void setDirection();
	void setReward();
	void setThrottle();
	void setBrake();
	void setSteering();
    void drawBoxes(Vector3 BLL, Vector3 FUR, Vector3 dim, Vector3 upVector, Vector3 rightVector, Vector3 forwardVector, Vector3 position, int colour);
    void createVehicles();
    void createVehicle(const char* model, float relativeForward, float relativeRight, float heading, int color, int color2);
    void createPed(int model, float relativeForward, float relativeRight, float heading, int task);
    void setCamParams();
    void setPosition();
    void setStencilBuffer();
    void changeVehicleOriantation(Ped v, int sector);
    void setCameraRotation(int sector);

    void exportPose();

    void setScanPose(Vehicle v, Vector3 position, Vector3 rotation);

    //Do not use this function. Causes GTA to crash - need to figure out why
    void setColorBuffer();

    void setupLiDAR();
    void collectLiDAR(float* depth);
    void exportImage(BYTE* data, std::string filename = "");
    void increaseIndex();
    std::string getStandardFilename(std::string subDir, std::string extension, int type);
    void exportDepth(float* depth);
    void saveAllData(float* depth);

};