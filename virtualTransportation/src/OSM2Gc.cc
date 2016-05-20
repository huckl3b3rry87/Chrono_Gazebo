#include "OSM2Gc.hh"

#include <gazebo/math/Vector2d.hh>
#include <osmscout/Area.h>
#include <osmscout/AreaWayIndex.h>
#include <osmscout/DataFile.h>
#include <osmscout/GeoCoord.h>
#include <osmscout/Point.h>
#include <osmscout/TypeConfig.h>
#include <osmscout/TypeFeatures.h>
#include <osmscout/Types.h>
#include <osmscout/Way.h>
#include <sdf/Element.hh>
#include <sdf/parser.hh>
#include <cmath>
#include <cstdbool>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <utility>
#include <vector>

// simple road model. ROAD_VIS with different geometries will be added to the link
#define ROAD_MODEL "\
	<sdf version='1.5'>\
		<model name='roads'>\
			<static>true</static>\
			<link name='roadslink'>\
				<must_be_base_link>true</must_be_base_link>\
			</link>\
		</model>\
	</sdf>"

// road segment visualization. the geometry will be changed for different road segments
#define ROAD_VIS "\
	<sdf version='1.5'>\
		<visual name='roadblock'>\
			<pose>0 0 0 0 0 0</pose>\
			<cast_shadows>false</cast_shadows>\
			<material>\
				<script>\
					<uri>file:/\
/../data/gazono.material</uri>\
					<name>Gazebo/Lanes_2</name>\
				</script>\
			</material>\
			<geometry>\
				<plane></plane>\
			</geometry>\
		</visual>\
	</sdf>"

// simple building model. building geometry is constructed by extruding a polygon
#define BUILDING_MODEL "\
	<sdf version='1.5'>\
		<model name='building'>\
			<static>true</static>\
			<link name='buildingslink'>\
				<must_be_base_link>true</must_be_base_link>\
				<visual name='buildingvis'>\
					<pose>0 0 0 0 0 0</pose>\
					<material>\
						<script>\
							<uri>file:/\
/../data/gazono.material</uri>\
							<name>Gazebo/PaintedWall</name>\
						</script>\
					</material>\
					<geometry>\
						<polyline></polyline>\
					</geometry>\
				</visual>\
			</link>\
		</model>\
	</sdf>"

// road configuration
struct RoadConf {
	std::string material;
	double width;
};

// building configuration
struct BuildingConf {
	std::string material;
	double height;
};

// mapping of different types of road to configurations
const static std::map<std::string, struct RoadConf> RoadConfMap = {
		{ "motorway", { "Gazebo/Motorway", 16 } },
		{ "trunk", { "Gazebo/Motorway", 16 } },
		{ "primary", { "Gazebo/Trunk", 12 } },
		{ "primary_link", { "Gazebo/Trunk", 12 } },
		{ "secondary", { "Gazebo/Primary", 9 } },
		{ "secondary_link", { "Gazebo/Primary", 9 } },
		{ "tertiary", { "Gazebo/Primary", 6 } },
		{ "residential", { "Gazebo/Primary", 6 } },
		{ "road", { "Gazebo/Road", 6 } },
		{ "service", { "Gazebo/Residential", 4 } },
		{ "footway", { "Gazebo/Residential", 3 } },
		{ "path", { "Gazebo/Residential", 3 } },
		{ "pedestrian", { "Gazebo/Residential", 3 } },
		{ "cycleway", { "Gazebo/Residential", 3 } },
		{ "step", { "Gazebo/Steps", 2 } } };

/*
 * Get the cartesian coordinate given a latitude/longitude pair and a bounding box.
 * The effect of changing latitude radius is taken into account.
 */
std::pair<double, double> GetCartesian(osmscout::GeoCoord coord,
		osmscout::GeoBox box) {
	osmscout::GeoCoord center = box.GetCenter();
	double width = cos(center.lat / 180.0 * 3.1415926)
			* (box.maxCoord.lon - box.minCoord.lon) * 110567;
	double height = (box.maxCoord.lat - box.minCoord.lat) * 111000;
	double x = ((coord.lon - box.minCoord.lon)
			/ (box.maxCoord.lon - box.minCoord.lon) - 0.855) * width;
	double y = ((coord.lat - box.minCoord.lat)
			/ (box.maxCoord.lat - box.minCoord.lat) - 0.33) * height;
	return std::make_pair(x, y);
}

/* Get distance between two points */
double GetDist(std::pair<double, double> p1, std::pair<double, double> p2) {
	return std::sqrt(
			(p1.first - p2.first) * (p1.first - p2.first)
					+ (p1.second - p2.second) * (p1.second - p2.second));
}

/* Create the OSM2Gc object with the given path to the osm preprocessed data */
OSM2Gc::OSM2Gc(const std::string& path) {
	osmscout::DatabaseParameter param;
	database = std::make_shared<osmscout::Database>(param);
	database->Open(path);
	database->GetBoundingBox(box);
	double lonSpan = box.maxCoord.lon - box.minCoord.lon;
	double latSpan = box.maxCoord.lat - box.minCoord.lat;
	box.maxCoord.lon -= lonSpan * 0.2;
	box.maxCoord.lat -= latSpan * 0.2;
	box.minCoord.lon += lonSpan * 0.2;
	box.minCoord.lat += latSpan * 0.2;
}

/* libosmscout processed the osm data such that types and its sub-types are separated by '_' */
void ParseType(const std::string &type, std::string parsed[2]) {
	int underscore = type.find('_');
	if (underscore == type.npos) {
		parsed[0] = type;
	} else {
		parsed[0] = type.substr(0, underscore);
		parsed[1] = type.substr(underscore + 1);
	}
}

/* Construct the road model from the preprocessed osm data */
sdf::SDFPtr OSM2Gc::GetRoadModel() {
	// get the way references from the database
	osmscout::AreaWayIndexRef areaWayIndex = database->GetAreaWayIndex();
	osmscout::TypeInfoSet wayTypeInfos(database->GetTypeConfig()->GetWayTypes());
	osmscout::TypeInfoSet loadedTypes;
	std::vector<osmscout::FileOffset> offsets;
	areaWayIndex->GetOffsets(box, wayTypeInfos, offsets, loadedTypes);
	std::vector<osmscout::WayRef> ways;
	database->GetWaysByOffset(offsets, ways);

	// initialize the road model
	sdf::SDFPtr roadsSDF(new sdf::SDF);
	sdf::initFile("model.sdf", roadsSDF);
	sdf::readString(ROAD_MODEL, roadsSDF->Root());
	sdf::SDFPtr roadBlocksSDF(new sdf::SDF);
	sdf::initFile("visual.sdf", roadBlocksSDF);
	sdf::readString(ROAD_VIS, roadBlocksSDF->Root());

	// write the way data to the road sdf
	sdf::ElementPtr linkPtr = roadsSDF->Root()->GetElement("link");
	sdf::ElementPtr visPtr = roadBlocksSDF->Root();
	char name[256];
	for (int i = 0; i < ways.size(); i++) {
		auto way = ways[i];

		std::string type[2];
		// get the type of the road
		ParseType(way->GetType()->GetName(), type);
		RoadConf conf;
		std::string materialName;
		// currently only work with highway
		if (type[0] == "highway") {
			std::map<std::string, RoadConf>::const_iterator itr;
			if ((itr = RoadConfMap.find(type[1])) != RoadConfMap.end()) {
				conf = itr->second;
			} else {
				conf = {"Gazebo/Residential", 3};
			}
		} else
		continue;

		std::pair<double, double> from;
		bool hasFrom = false;
		for (int j = 0; j < way->nodes.size(); j++) {
			auto to = GetCartesian(way->nodes[j].GetCoord(), box);
			// find the starting point of the road. can add constraints here so that
			// the first point is inside some bounding box
			if (!hasFrom) {
				from = to;
				hasFrom = true;
				continue;
			}
			sprintf(name, "roadblock_%d_%d", i, j);
			// make sure each road segment has a different name
			visPtr->GetAttribute("name")->SetFromString(name);
			// use the material defined in the configuration
			visPtr->GetElement("material")->GetElement("script")->GetElement("name")->Set(
					conf.material);
			// the length of the road semgnt is the distance between the from and to points
			double length = GetDist(from, to);
			visPtr->GetElement("geometry")->GetElement("plane")->GetElement("size")->Set(
					gazebo::math::Vector2d(conf.width, length + conf.width / 4));
			double x = (from.first + to.first) / 2.0;
			double y = (from.second + to.second) / 2.0;
			// lift each road segment a slightly different height to avoid depth fighting
			double z = 0.01 * conf.width + 0.0005 * (j + i);
			double yaw =
					to.second == from.second ?
							acos(0) :
							-atan(((to.first - from.first) / (to.second - from.second)));
			char buf[256];
			sprintf(buf, "%f %f %f %f %f %f", x, y, z, 0.0, 0.0, yaw);
			visPtr->GetElement("pose")->Set(buf);
			from = to;
			linkPtr->InsertElement(visPtr->Clone());
		}
	}
//	printf("%s\n", roadsSDF->Root()->ToString("").c_str());
	return roadsSDF;
}

/* Construct a list of building models from the preprocessed osm data */
std::vector<sdf::SDFPtr> OSM2Gc::GetBuildingModels() {
	// get the building references from the database
	auto areaAreaIndex = database->GetAreaAreaIndex();
	osmscout::TypeInfoSet areaTypeInfos(
			database->GetTypeConfig()->GetAreaTypes());
	osmscout::TypeInfoSet loadedAreaTypes;
	std::vector<osmscout::DataBlockSpan> areaSpans;
	areaAreaIndex->GetAreasInArea(*database->GetTypeConfig(), box, 1000,
			areaTypeInfos, areaSpans, loadedAreaTypes);
	std::vector<osmscout::AreaRef> areas;

	// initialize building models
	std::vector<sdf::SDFPtr> buildingModels;
	sdf::SDFPtr buildingSDF(new sdf::SDF);
	sdf::initFile("model.sdf", buildingSDF);
	sdf::readString(BUILDING_MODEL, buildingSDF->Root());

// write the area data to the building sdf
	sdf::ElementPtr visPtr = buildingSDF->Root()->GetElement("link")->GetElement(
			"visual");
	sdf::ElementPtr polylinePtr = visPtr->GetElement("geometry")->GetElement(
			"polyline");
	sdf::ElementPtr materialNamePtr = visPtr->GetElement("material")->GetElement(
			"script")->GetElement("name");
	char name[256];
	database->GetAreasByBlockSpans(areaSpans, areas);
	for (int i = 0; i < areas.size(); i++) {
		osmscout::GeoBox areaBox;
		areas[i]->GetBoundingBox(areaBox);
		if (!areaBox.Intersects(box))
			continue;
		std::string type[2];
		BuildingConf conf;
		ParseType(areas[i]->GetType()->GetName(), type);
		// it seems that boundary in osm usually should not be visualized
		if (type[0] == "boundary" && type[1] == "administrative")
			continue;
		// it seems that landuse in osm usually should not be visualized
		if (type[0] == "landuse" && type[1] != "grass")
			continue;
		if (type[1] == "grass") {
			conf.material = "Gazebo/Grass6";
			conf.height = 0.2;
		} else if (type[1] == "parking") {
			conf.material = "Gazebo/Grey";
			conf.height = 0.1;
		} else {
			conf.material = "Gazebo/Building" + std::to_string(std::rand() % 4);
			conf.height = 5 + (std::rand() % 5) * (std::rand() % 5);
		}

		polylinePtr->ClearElements();
		materialNamePtr->Set(conf.material);
		for (int j = 0; j < areas[i]->rings.size(); j++) {
			auto nodes = areas[i]->rings[j].nodes;
			if (nodes.size() < 3)
				continue;
			sprintf(name, "building_%d_%d", i, j);
			buildingSDF->Root()->GetAttribute("name")->SetFromString(name);
			polylinePtr->GetElement("height")->Set(conf.height);
			for (int k = 0; k < nodes.size(); k++) {
				auto loc = GetCartesian(nodes[k].GetCoord(), box);
				polylinePtr->AddElement("point")->Set(
						gazebo::math::Vector2d(loc.first, loc.second));
			}
			sdf::SDFPtr newSDF(new sdf::SDF);
			newSDF->Root(buildingSDF->Root()->Clone());
			buildingModels.push_back(newSDF);
		}
	}
	return buildingModels;
}
