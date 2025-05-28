#include <iostream>

#include "concord/types_polygon.hpp"
#include "geoson/parser.hpp"
#include "geoson/writter.hpp"
#include "geotiv/parse.hpp"
#include "geotiv/writter.hpp"

#include "rerun/recording_stream.hpp"

#include "farmtrax/field.hpp"
#include "rerun.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.80182312011719};

    concord::Polygon poly;
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
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

    for (auto &p : poly.getPoints()) {
        std::cout << "x: " << p.enu.x << ", y: " << p.enu.y << ", z: " << p.enu.z << "\n";
    }

    farmtrax::Field field(poly, 0.1, datum, true, 0.5);
    field.add_noise();

    geotiv::Layer layer;
    layer.grid = field.get_grid(0);
    layer.samplesPerPixel = 1;
    layer.planarConfig = 1;
    geotiv::RasterCollection rc;
    rc.crs = concord::CRS::WGS;
    rc.datum = concord::Datum();
    rc.heading = concord::Euler{0.0, 0.0, 0.0};
    rc.resolution = 0.1;
    rc.layers.push_back(layer);

    std::filesystem::path outPath = "output.tif";
    geotiv::WriteRasterCollection(rc, outPath);

    field.gen_field(4.0, 0.0, 3);

    auto part_cnt = field.get_parts().size();
    std::cout << "Part count: " << part_cnt << "\n";

    std::vector<std::vector<std::array<float, 3>>> borders;
    for (auto const &part : field.get_parts()) {
        std::vector<std::array<float, 3>> border_one;
        auto poy = part.border.polygon;
        for (auto p : poy.getPoints()) {
            std::array<float, 3> v;
            v[0] = p.enu.x;
            v[1] = p.enu.y;
            v[2] = 0.0f;
            border_one.push_back(v);
        }
        borders.push_back(border_one);
    }

    for (uint i = 0; i < borders.size(); ++i) {
        auto colorz = rerun::Color(255.0f, 0.0f, 0.0f);
        auto border__ = rerun::components::LineStrip3D(borders[i]);
        rec->log_static("/border" + std::to_string(i),
                        rerun::LineStrips3D(border__).with_colors({{colorz}}).with_radii({{0.2f}}));
    }

    for (uint i = 0; i < field.get_parts().size(); ++i) {
        std::vector<std::vector<std::array<float, 3>>> headlands;
        for (auto const &headland : field.get_parts()[i].headlands) {
            std::vector<std::array<float, 3>> headland_one;
            auto poy = headland.polygon;
            for (auto p : poy.getPoints()) {
                std::array<float, 3> v;
                v[0] = p.enu.x;
                v[1] = p.enu.y;
                v[2] = 0.0f;
                headland_one.push_back(v);
            }
            headlands.push_back(headland_one);
        }

        for (uint j = 1; j < headlands.size(); ++j) {
            auto colorz = rerun::Color(0.0f, 255.0f, 0.0f);
            auto headland__ = rerun::components::LineStrip3D(headlands[j]);
            rec->log_static("/headland" + std::to_string(i) + "_" + std::to_string(j),
                            rerun::LineStrips3D(headland__).with_colors({{colorz}}).with_radii({{0.2f}}));
        }
    }

    return 0;
}
