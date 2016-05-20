#include <osmscout/AreaWayIndex.h>
#include <osmscout/Database.h>
#include <osmscout/GeoCoord.h>
#include <osmscout/Point.h>
#include <osmscout/TypeConfig.h>
#include <osmscout/Types.h>
#include <osmscout/util/GeoBox.h>
#include <osmscout/Way.h>
#include <cmath>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

std::pair<double, double> GetCartesian(osmscout::GeoCoord coord,
		osmscout::GeoBox box) {
	osmscout::GeoCoord center = box.GetCenter();
	double width = cos(center.lat / 180.0 * 3.1415926)
			* (box.maxCoord.lon - box.minCoord.lon) * 110567;
	double height = (box.maxCoord.lat - box.minCoord.lat) * 111000;
	double x = ((coord.lon - box.minCoord.lon)
			/ (box.maxCoord.lon - box.minCoord.lon) - 0.7) * width;
	double y = ((coord.lat - box.minCoord.lat)
			/ (box.maxCoord.lat - box.minCoord.lat) - 0.4) * height;
	return std::make_pair(x, y);
}

int main() {
	osmscout::DatabaseParameter param;
	osmscout::DatabaseRef database = std::make_shared<osmscout::Database>(param);
	database->Open("../data/madison");
	auto areaWayIndex = database->GetAreaWayIndex();
	auto areaAreaIndex = database->GetAreaAreaIndex();
	osmscout::GeoBox box, originalBox;
	database->GetBoundingBox(box);
	database->GetBoundingBox(originalBox);

	double scaleFactor = 0;
	auto center = box.GetCenter();
	box.minCoord.lat += scaleFactor * (center.lat - box.minCoord.lat);
	box.minCoord.lon += scaleFactor * (center.lon - box.minCoord.lon);
	box.maxCoord.lat += scaleFactor * (center.lat - box.maxCoord.lat);
	box.maxCoord.lon += scaleFactor * (center.lon - box.maxCoord.lon);

	double width = cos(center.lat / 180.0 * 3.1415926)
			* (box.maxCoord.lon - box.minCoord.lon) * 110567;
	double height = (box.maxCoord.lat - box.minCoord.lat) * 111000;
	osmscout::GeoBox cartBox;
	cartBox.minCoord.lon = -width / 2;
	cartBox.minCoord.lat = -height / 2;
	cartBox.maxCoord.lon = width / 2;
	cartBox.maxCoord.lat = height / 2;

	int c = 0;
	if (box.minCoord != box.minCoord)
		c++;
	printf("c=%d\n", c);

	osmscout::TypeInfoSet wayTypeInfos(database->GetTypeConfig()->GetWayTypes());
	osmscout::TypeInfoSet areaTypeInfos(
			database->GetTypeConfig()->GetAreaTypes());
	osmscout::TypeInfoSet loadedWayTypes;
	osmscout::TypeInfoSet loadedAreaTypes;
	std::vector<osmscout::FileOffset> wayOffsets;
	std::vector<osmscout::DataBlockSpan> areaSpans;
	areaWayIndex->GetOffsets(box, wayTypeInfos, wayOffsets, loadedWayTypes);
	areaAreaIndex->GetAreasInArea(*database->GetTypeConfig(), originalBox, 1000,
			areaTypeInfos, areaSpans, loadedAreaTypes);
	std::vector<osmscout::WayRef> ways;
	std::vector<osmscout::AreaRef> areas;
	database->GetWaysByOffset(wayOffsets, ways);
	database->GetAreasByBlockSpans(areaSpans, areas);
//	for (auto iter = loadedWayTypes.begin(); iter != loadedWayTypes.end(); iter++) {
//		printf("Way Type: %s\n", (*iter)->GetName().c_str());
//	}
//	for (auto iter = loadedAreaTypes.begin(); iter != loadedAreaTypes.end(); iter++) {
//		printf("Area Type: %s\n", (*iter)->GetName().c_str());
//	}
//	FILE *way_out = std::fopen("way_out.txt", "w");
//	FILE *area_out = std::fopen("area_out.txt", "w");
//	for (int i = 0; i < ways.size(); i++) {
//		printf("Way %d: ", i);
//		fprintf(way_out, "%ld\n", ways[i]->nodes.size());
//		printf("type: %s\n", ways[i]->GetType()->GetName().c_str());
//		for (int j = 0; j < ways[i]->nodes.size(); j++) {
//			auto coord = getCartesian(ways[i]->nodes[j].GetCoord(), box);
//			osmscout::GeoCoord cartCoord;
//			cartCoord.lon = coord.first;
//			cartCoord.lat = coord.second;
////			printf("[%.1f, %.1f], ", coord.first, coord.second);
//			fprintf(way_out, "%.4f\t%.4f\n", coord.first, coord.second);
//		}
//		printf("\n");
//	}
//	for (int i = 0; i < areas.size(); i++) {
////		printf("Area %d: ", i);
////		printf("%ld, %ld\n", (long int) areas[i]->rings.size());
////		printf("type: %s\n", areas[i]->GetType()->GetName().c_str());
//		for (int j = 0; j < areas[i]->rings.size(); j++) {
//			fprintf(area_out, "%d\n", areas[i]->rings[j].nodes.size());
//			for (int k = 0; k < areas[i]->rings[j].nodes.size(); k++) {
//				auto coord = getCartesian(areas[i]->rings[j].nodes[k].GetCoord(), box);
//				fprintf(area_out, "%.7f\t%.7f\n", coord.first, coord.second);
//			}
//		}
//	}
//	fclose(way_out);
//	fclose(area_out);
	return 0;
}
