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

    concord::Polygon poly;
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
        for (auto &f : fc.features) {
            if (std::get_if<concord::Polygon>(&f.geometry)) {
                poly = std::get<concord::Polygon>(f.geometry);
                spdlog::info("Found polygon");
                break;
            }
        }
    } catch (std::exception &e) {
        spdlog::error("Failed to parse geojson: {}", e.what());
        return 1;
    }

    for (auto &p : poly.getPoints()) {
        spdlog::info("x: {}, y: {}, z: {}\n", p.enu.x, p.enu.y, p.enu.z);
    }

    return 0;
}
