#pragma once

#include "concord/concord.hpp"
#include "farmtrax/utils/utils.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace farmtrax {

    using BPoint = boost::geometry::model::d2::point_xy<double>;
    using BLineString = boost::geometry::model::linestring<BPoint>;
    using BPolygon = boost::geometry::model::polygon<BPoint>;
    using BBox = boost::geometry::model::box<BPoint>;
    using BMultiPolygon = boost::geometry::model::multi_polygon<BPolygon>;

    class Partitioner {
    public:
        struct PartitionCriteria {
            double max_area;                      // Maximum area in sq meters (5 hectares)
            double min_convexity;                 // Minimum convexity ratio (60%)
            double max_aspect_ratio;              // Maximum length/width ratio
            double min_bridge_width;              // Minimum bridge width in meters
            double tooth_threshold;               // Tooth size relative to main body
            double simplify_tolerance;            // Polygon simplification tolerance
            int max_recursion_depth;              // Maximum recursion levels
            bool enable_bridge_detection;         // Enable narrow bridge splitting
            bool enable_tooth_detection;          // Enable tooth/extension splitting
            bool enable_aspect_splitting;         // Enable elongated field splitting
            
            // Default constructor
            PartitionCriteria() 
                : max_area(50000.0)
                , min_convexity(0.6)
                , max_aspect_ratio(4.0)
                , min_bridge_width(20.0)
                , tooth_threshold(0.3)
                , simplify_tolerance(2.0)
                , max_recursion_depth(5)
                , enable_bridge_detection(true)
                , enable_tooth_detection(true)
                , enable_aspect_splitting(true) {}
        };

    private:
        concord::Polygon border_;
        concord::Datum datum_;
        PartitionCriteria criteria_;

    public:
        std::vector<concord::Polygon> polygons_;

        Partitioner() = default;
        
        Partitioner(const concord::Polygon &poly, const concord::Datum &datum = concord::Datum{}) 
            : border_(poly), datum_(datum) {}

        // Main partitioning function with intelligent multi-criteria splitting
        std::vector<concord::Polygon> partition(double area_threshold, 
                                               const PartitionCriteria &criteria = PartitionCriteria{}) {
            criteria_ = criteria;
            criteria_.max_area = area_threshold; // Override with provided threshold
            
            polygons_.clear();
            std::vector<concord::Polygon> result;
            
            // Start recursive partitioning
            partition_recursive(border_, result, 0);
            
            polygons_ = result;
            return result;
        }

        // Get current partitioning criteria
        const PartitionCriteria& getCriteria() const { return criteria_; }
        
        // Set partitioning criteria
        void setCriteria(const PartitionCriteria &criteria) { criteria_ = criteria; }

    private:
        
        // Recursive partitioning with multiple criteria
        void partition_recursive(const concord::Polygon &poly, 
                                std::vector<concord::Polygon> &result, 
                                int depth) {
            
            if (depth >= criteria_.max_recursion_depth) {
                result.push_back(poly);
                return;
            }
            
            // Convert to boost polygon for analysis
            BPolygon boost_poly = utils::to_boost(poly);
            double area = boost::geometry::area(boost_poly);
            
            // Check if polygon meets all criteria
            if (meets_all_criteria(boost_poly, area)) {
                result.push_back(poly);
                return;
            }
            
            // Try different splitting strategies in order of priority
            std::vector<concord::Polygon> split_result;
            
            // 1. Area-based splitting (highest priority)
            if (area > criteria_.max_area) {
                split_result = split_by_area(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }
            
            // 2. Bridge detection and splitting
            if (criteria_.enable_bridge_detection) {
                split_result = split_by_narrow_bridges(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }
            
            // 3. Tooth/extension detection and splitting
            if (criteria_.enable_tooth_detection) {
                split_result = split_by_teeth_and_extensions(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }
            
            // 4. Aspect ratio-based splitting
            if (criteria_.enable_aspect_splitting) {
                split_result = split_by_aspect_ratio(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }
            
            // 5. Convexity-based splitting (last resort)
            split_result = split_by_convexity(poly);
            if (split_result.size() > 1) {
                for (const auto &part : split_result) {
                    partition_recursive(part, result, depth + 1);
                }
                return;
            }
            
            // If no splitting worked, accept the polygon as-is
            result.push_back(poly);
        }
        
        // Check if polygon meets all partitioning criteria
        bool meets_all_criteria(const BPolygon &boost_poly, double area) const {
            // Area check
            if (area > criteria_.max_area) {
                return false;
            }
            
            // Convexity check
            if (calculate_convexity_ratio(boost_poly) < criteria_.min_convexity) {
                return false;
            }
            
            // Aspect ratio check
            if (calculate_aspect_ratio(boost_poly) > criteria_.max_aspect_ratio) {
                return false;
            }
            
            return true;
        }
        
        // Calculate convexity ratio (area / convex_hull_area)
        double calculate_convexity_ratio(const BPolygon &poly) const {
            BPolygon convex_hull;
            boost::geometry::convex_hull(poly, convex_hull);
            
            double poly_area = boost::geometry::area(poly);
            double hull_area = boost::geometry::area(convex_hull);
            
            if (hull_area < 1e-10) return 1.0; // Avoid division by zero
            return poly_area / hull_area;
        }
        
        // Calculate aspect ratio (length / width) from oriented bounding box
        double calculate_aspect_ratio(const BPolygon &poly) const {
            BBox envelope = boost::geometry::return_envelope<BBox>(poly);
            
            double width = envelope.max_corner().x() - envelope.min_corner().x();
            double height = envelope.max_corner().y() - envelope.min_corner().y();
            
            if (std::min(width, height) < 1e-10) return 1.0; // Avoid division by zero
            return std::max(width, height) / std::min(width, height);
        }
        
        // Split polygon by area (simple geometric splitting)
        std::vector<concord::Polygon> split_by_area(const concord::Polygon &poly) const {
            BPolygon boost_poly = utils::to_boost(poly);
            BBox envelope = boost::geometry::return_envelope<BBox>(boost_poly);
            
            double width = envelope.max_corner().x() - envelope.min_corner().x();
            double height = envelope.max_corner().y() - envelope.min_corner().y();
            
            // Split along the longer dimension
            BLineString cutting_line;
            if (width > height) {
                // Split vertically
                double mid_x = (envelope.min_corner().x() + envelope.max_corner().x()) / 2.0;
                cutting_line.push_back(BPoint(mid_x, envelope.min_corner().y() - height));
                cutting_line.push_back(BPoint(mid_x, envelope.max_corner().y() + height));
            } else {
                // Split horizontally  
                double mid_y = (envelope.min_corner().y() + envelope.max_corner().y()) / 2.0;
                cutting_line.push_back(BPoint(envelope.min_corner().x() - width, mid_y));
                cutting_line.push_back(BPoint(envelope.max_corner().x() + width, mid_y));
            }
            
            return split_polygon_with_line(poly, cutting_line);
        }
        
        // Detect and split narrow bridges using erosion
        std::vector<concord::Polygon> split_by_narrow_bridges(const concord::Polygon &poly) const {
            BPolygon boost_poly = utils::to_boost(poly);
            
            // Apply negative buffer (erosion) to detect narrow bridges
            BMultiPolygon eroded;
            double erosion_distance = -criteria_.min_bridge_width / 2.0;
            
            try {
                boost::geometry::strategy::buffer::distance_symmetric<double> dist_strategy(erosion_distance);
                boost::geometry::strategy::buffer::side_straight side_strategy;
                boost::geometry::strategy::buffer::join_miter join_strategy;
                boost::geometry::strategy::buffer::end_flat end_strategy;
                boost::geometry::strategy::buffer::point_square point_strategy;
                
                boost::geometry::buffer(boost_poly, eroded, dist_strategy, side_strategy, 
                                      join_strategy, end_strategy, point_strategy);
            } catch (const std::exception &e) {
                std::cerr << "Buffer operation failed in bridge detection: " << e.what() << std::endl;
                return {poly}; // Return original if erosion fails
            }
            
            // If erosion resulted in multiple disconnected parts, we found bridges
            if (eroded.size() > 1) {
                // Find the narrowest connection points and split there
                return split_at_narrowest_connections(poly, eroded);
            }
            
            return {poly}; // No bridges found
        }
        
        // Split polygon by detecting teeth and extensions
        std::vector<concord::Polygon> split_by_teeth_and_extensions(const concord::Polygon &poly) const {
            BPolygon boost_poly = utils::to_boost(poly);
            
            // Calculate convex hull
            BPolygon convex_hull;
            boost::geometry::convex_hull(boost_poly, convex_hull);
            
            // Find areas that are in convex hull but not in original (concave areas)
            BMultiPolygon difference_result;
            try {
                boost::geometry::difference(convex_hull, boost_poly, difference_result);
            } catch (const std::exception &e) {
                std::cerr << "Difference operation failed in tooth detection: " << e.what() << std::endl;
                return {poly};
            }
            
            // Analyze difference areas to find significant "bites" or concavities
            double original_area = boost::geometry::area(boost_poly);
            
            for (const auto &diff_poly : difference_result) {
                double diff_area = boost::geometry::area(diff_poly);
                
                // If the concave area is significant, try to split across it
                if (diff_area > original_area * criteria_.tooth_threshold) {
                    auto split_result = split_across_concavity(poly, diff_poly);
                    if (split_result.size() > 1) {
                        return split_result;
                    }
                }
            }
            
            return {poly}; // No significant teeth found
        }
        
        // Split elongated polygons by aspect ratio
        std::vector<concord::Polygon> split_by_aspect_ratio(const concord::Polygon &poly) const {
            BPolygon boost_poly = utils::to_boost(poly);
            double aspect_ratio = calculate_aspect_ratio(boost_poly);
            
            if (aspect_ratio > criteria_.max_aspect_ratio) {
                return split_by_area(poly); // Use area splitting for elongated shapes
            }
            
            return {poly}; // Aspect ratio is acceptable
        }
        
        // Split highly concave polygons using convexity analysis
        std::vector<concord::Polygon> split_by_convexity(const concord::Polygon &poly) const {
            BPolygon boost_poly = utils::to_boost(poly);
            double convexity = calculate_convexity_ratio(boost_poly);
            
            if (convexity < criteria_.min_convexity) {
                // Try to find the best cutting line through the concave areas
                return split_by_area(poly); // Fallback to geometric splitting
            }
            
            return {poly}; // Convexity is acceptable
        }
        
        // Helper function to split polygon with a line
        std::vector<concord::Polygon> split_polygon_with_line(const concord::Polygon &poly, 
                                                            const BLineString &cutting_line) const {
            // This is a simplified implementation
            // In practice, you'd need more sophisticated polygon cutting algorithms
            BPolygon boost_poly = utils::to_boost(poly);
            
            // Create a thin rectangle along the cutting line for boolean operations
            BPolygon cutting_rect;
            if (cutting_line.size() >= 2) {
                BPoint p1 = cutting_line[0];
                BPoint p2 = cutting_line[1];
                
                // Create a thin cutting rectangle
                double dx = p2.x() - p1.x();
                double dy = p2.y() - p1.y();
                double length = std::sqrt(dx*dx + dy*dy);
                
                if (length > 1e-10) {
                    // Normalize direction
                    dx /= length;
                    dy /= length;
                    
                    // Perpendicular direction
                    double px = -dy * 0.1; // Very thin cutting line
                    double py = dx * 0.1;
                    
                    cutting_rect.outer().push_back(BPoint(p1.x() + px, p1.y() + py));
                    cutting_rect.outer().push_back(BPoint(p2.x() + px, p2.y() + py));
                    cutting_rect.outer().push_back(BPoint(p2.x() - px, p2.y() - py));
                    cutting_rect.outer().push_back(BPoint(p1.x() - px, p1.y() - py));
                    cutting_rect.outer().push_back(cutting_rect.outer().front());
                    
                    boost::geometry::correct(cutting_rect);
                }
            }
            
            // Subtract cutting rectangle from original polygon
            BMultiPolygon difference_result;
            try {
                boost::geometry::difference(boost_poly, cutting_rect, difference_result);
            } catch (const std::exception &e) {
                std::cerr << "Polygon cutting failed: " << e.what() << std::endl;
                return {poly}; // Return original if cutting fails
            }
            
            // Convert results back to concord polygons
            std::vector<concord::Polygon> result;
            for (const auto &part : difference_result) {
                if (boost::geometry::area(part) > 1.0) { // Filter out tiny fragments
                    result.push_back(utils::from_boost(part, datum_));
                }
            }
            
            // If cutting didn't work well, return original
            if (result.empty()) {
                result.push_back(poly);
            }
            
            return result;
        }
        
        // Split at narrowest connections found by erosion
        std::vector<concord::Polygon> split_at_narrowest_connections(const concord::Polygon &poly, 
                                                                    const BMultiPolygon &eroded_parts) const {
            // This is a simplified implementation
            // Find the geometric center between the eroded parts and split there
            if (eroded_parts.size() < 2) {
                return {poly};
            }
            
            // Calculate centroids of the eroded parts
            std::vector<BPoint> centroids;
            for (const auto &part : eroded_parts) {
                BPoint centroid;
                boost::geometry::centroid(part, centroid);
                centroids.push_back(centroid);
            }
            
            // Create cutting line between the two most distant centroids
            if (centroids.size() >= 2) {
                BLineString cutting_line;
                cutting_line.push_back(centroids[0]);
                cutting_line.push_back(centroids[1]);
                
                // Extend the line to ensure it cuts through the entire polygon
                BBox envelope = boost::geometry::return_envelope<BBox>(utils::to_boost(poly));
                double width = envelope.max_corner().x() - envelope.min_corner().x();
                double height = envelope.max_corner().y() - envelope.min_corner().y();
                double extension = std::max(width, height);
                
                BPoint p1 = centroids[0];
                BPoint p2 = centroids[1];
                double dx = p2.x() - p1.x();
                double dy = p2.y() - p1.y();
                double length = std::sqrt(dx*dx + dy*dy);
                
                if (length > 1e-10) {
                    dx /= length;
                    dy /= length;
                    
                    BLineString extended_line;
                    extended_line.push_back(BPoint(p1.x() - dx * extension, p1.y() - dy * extension));
                    extended_line.push_back(BPoint(p2.x() + dx * extension, p2.y() + dy * extension));
                    
                    return split_polygon_with_line(poly, extended_line);
                }
            }
            
            return {poly}; // Fallback to original
        }
        
        // Split across significant concavity
        std::vector<concord::Polygon> split_across_concavity(const concord::Polygon &poly, 
                                                           const BPolygon &concavity) const {
            // Find the "chord" across the concavity - the shortest line that would close it
            BPoint centroid;
            boost::geometry::centroid(concavity, centroid);
            
            // This is a simplified approach - find the polygon edge closest to the concavity
            // and create a cutting line from there
            BPolygon boost_poly = utils::to_boost(poly);
            BBox envelope = boost::geometry::return_envelope<BBox>(boost_poly);
            
            // Create a cutting line through the concavity centroid
            BLineString cutting_line;
            cutting_line.push_back(BPoint(envelope.min_corner().x(), centroid.y()));
            cutting_line.push_back(BPoint(envelope.max_corner().x(), centroid.y()));
            
            return split_polygon_with_line(poly, cutting_line);
        }
    };

} // namespace farmtrax
