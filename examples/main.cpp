#include <iostream>

#include "concord/types_polygon.hpp"
#include "geoson/parser.hpp"
#include "geoson/writter.hpp"
#include "rerun/recording_stream.hpp"
#include "spdlog/spdlog.h"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");

        concord::Polygon poly;
        for (auto &f : fc.features) {
            if (std::get_if<concord::Polygon>(&f.geometry)) {
                poly = std::get<concord::Polygon>(f.geometry);
                spdlog::info("Found polygon");
                break;
            }
        }

        fc.datum.lat -= 0.1;
        geoson::WriteFeatureCollection(fc, "misc/field4.geojson");
        spdlog::info("Saved modified GeoJSON to misc/field4_modified.geojson");
    } catch (std::exception &e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
