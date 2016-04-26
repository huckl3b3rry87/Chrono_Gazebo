#ifndef SRC_OSM2GC_HH_
#define SRC_OSM2GC_HH_

#include <osmscout/Database.h>
#include <osmscout/util/GeoBox.h>
#include <sdf/SDFImpl.hh>
#include <string>

class OSM2Gc {
public:
	OSM2Gc(const std::string& path);

	sdf::SDFPtr GetRoadModel();

	std::vector<sdf::SDFPtr> GetBuildingModels();

private:
	osmscout::DatabaseRef database;
	osmscout::GeoBox box;
};
#endif /* SRC_OSM2GC_HH */
