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

#define ROAD_MODEL "\
	<sdf version='1.5'>\
		<model name='roads'>\
			<static>true</static>\
			<link name='roadslink'>\
				<must_be_base_link>true</must_be_base_link>\
			</link>\
		</model>\
	</sdf>"

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

struct RoadConf {
	std::string material;
	double width;
};

struct BuildingConf {
	std::string material;
	double height;
};

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

double GetDist(std::pair<double, double> p1, std::pair<double, double> p2) {
	return std::sqrt(
			(p1.first - p2.first) * (p1.first - p2.first)
					+ (p1.second - p2.second) * (p1.second - p2.second));
}

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

void ParseType(const std::string &type, std::string parsed[2]) {
	int underscore = type.find('_');
	if (underscore == type.npos) {
		parsed[0] = type;
	} else {
		parsed[0] = type.substr(0, underscore);
		parsed[1] = type.substr(underscore + 1);
	}
}

sdf::SDFPtr OSM2Gc::GetRoadModel() {
	// get the way references from the database
	osmscout::AreaWayIndexRef areaWayIndex = database->GetAreaWayIndex();
	osmscout::TypeInfoSet wayTypeInfos(database->GetTypeConfig()->GetWayTypes());
	osmscout::TypeInfoSet loadedTypes;
	std::vector<osmscout::FileOffset> offsets;
	areaWayIndex->GetOffsets(box, wayTypeInfos, offsets, loadedTypes);
	std::vector<osmscout::WayRef> ways;
	database->GetWaysByOffset(offsets, ways);

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
		ParseType(way->GetType()->GetName(), type);
		RoadConf conf;
		std::string materialName;
		if (type[0] == "highway") {
//			printf("highway_%s\n", type[1].c_str());
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
		for (int j = 1; j < way->nodes.size(); j++) {
			auto to = GetCartesian(way->nodes[j].GetCoord(), box);
			if (!hasFrom) {
				from = to;
				hasFrom = true;
				continue;
			}
			sprintf(name, "roadblock_%d_%d", i, j);
			visPtr->GetAttribute("name")->SetFromString(name);
			visPtr->GetElement("material")->GetElement("script")->GetElement("name")->Set(
					conf.material);
			double length = GetDist(from, to);
			visPtr->GetElement("geometry")->GetElement("plane")->GetElement("size")->Set(
					gazebo::math::Vector2d(conf.width, length + conf.width / 4));
			double x = (from.first + to.first) / 2.0;
			double y = (from.second + to.second) / 2.0;
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

std::vector<sdf::SDFPtr> OSM2Gc::GetBuildingModels() {
	auto areaAreaIndex = database->GetAreaAreaIndex();
	osmscout::TypeInfoSet areaTypeInfos(
			database->GetTypeConfig()->GetAreaTypes());
	osmscout::TypeInfoSet loadedAreaTypes;
	std::vector<osmscout::DataBlockSpan> areaSpans;
	areaAreaIndex->GetAreasInArea(*database->GetTypeConfig(), box, 1000,
			areaTypeInfos, areaSpans, loadedAreaTypes);
	std::vector<osmscout::AreaRef> areas;

	std::vector<sdf::SDFPtr> buildingModels;
//	return buildingModels;
	sdf::SDFPtr buildingSDF(new sdf::SDF);
	sdf::initFile("model.sdf", buildingSDF);
	sdf::readString(BUILDING_MODEL, buildingSDF->Root());

// write the area data to the road sdf
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
//		if (areaBox.maxCoord.lon + areaBox.maxCoord.lat - areaBox.minCoord.lon
//				- areaBox.minCoord.lat > 0.0001) {
//			printf("idx: %d, type: %s, size: %lu\n", i,
//					areas[i]->GetType()->GetName().c_str(),
//					areas[i]->rings[0].nodes.size());
//			printf("lon diff: %.8f, lat diff: %.8f\n",
//					areaBox.maxCoord.lon - areaBox.minCoord.lon,
//					areaBox.maxCoord.lat - areaBox.minCoord.lat);
//		}
//
//		printf("\n");
		std::string type[2];
		BuildingConf conf;
		ParseType(areas[i]->GetType()->GetName(), type);
		if (type[0] == "boundary" && type[1] == "administrative")
			continue;
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
//			printf("feature of area %d ring %d\n", i, j);
//			for (auto f : areas[i]->rings[j].GetType()->GetFeatures()) {
//				printf("%s: ", f.GetFeature()->GetName().c_str());
//				if (!f.GetFeature()->HasValue()) {
//					printf("no value;    ");
//					continue;
//				}
//				char buf[1024];
//				auto val = f.GetFeature()->AllocateValue(buf);
//				if (dynamic_cast<osmscout::NameFeatureValue*>(val) == NULL) {
//					printf("not name;    ");
//					continue;
//				}
//				auto nval = dynamic_cast<osmscout::NameFeatureValue*>(val);
//				printf("%s;    ", nval->GetName().c_str());
//			}
//			printf("\n");
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
//	printf("%s\n", buildingsSDF->Root()->ToString("").c_str());
	return buildingModels;
}
