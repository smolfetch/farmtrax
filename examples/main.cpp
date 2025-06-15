#include <iomanip>
#include <iostream>
#include <thread>

#include "concord/concord.hpp"
#include "geoson/geoson.hpp"
#include "geotiv/geotiv.hpp"

#include "rerun/recording_stream.hpp"

#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "rerun.hpp"
#include "thread"

#include "farmtrax/utils/visualize.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    concord::Polygon poly;
    concord::Datum datum; // Will be set from GeoJSON
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
        datum = fc.datum; // Use the datum from the GeoJSON file
        std::cout << "Using datum from GeoJSON: lat=" << datum.lat << ", lon=" << datum.lon << ", alt=" << datum.alt
                  << std::endl;

        for (auto &f : fc.features) {
            if (std::get_if<concord::Polygon>(&f.geometry)) {
                poly = std::get<concord::Polygon>(f.geometry);
                break;
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Failed to parse geojson: " << e.what() << "\n";
        return 1;
    }

    // Calculate field bounds first
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : poly.getPoints()) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
    }

    std::cout << "Field bounds: x[" << min_x << ", " << max_x << "], y[" << min_y << ", " << max_y << "]\n";

    farmtrax::Field field(poly, 0.1, datum, true, 0.7, 50000.0);

    field.add_noise();

    farmtrax::visualize::show_field(field, rec);

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 2;

    std::this_thread::sleep_for(std::chrono::seconds(5));
    farmtrax::visualize::show_field(field, rec);

    return 0;
}
