// free_space_regions.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"
#include "gcs_core/convex_region.hpp"
#include "gcs_core/point_index.hpp"

#include <string>

namespace gcs {

/**
 * @brief Sphere flip p -> p * (2R - ||p||)/||p|| about `center`.
 * @remark Zhong et al. sphere-flip transform.
 * @param points Points to flip (rows = points).
 * @param center Sphere center.
 * @param R Sphere radius.
 * @return Flipped points; rows with zero radius are left at the origin offset.
 */
MatrixXd sphere_flip(const MatrixXd& points, const VectorXd& center, double R);

/**
 * @brief Inverse sphere flip (an involution: same functional form as the forward map).
 * @param flipped Previously flipped points.
 * @param center Sphere center used for the original flip.
 * @param R Sphere radius used for the original flip.
 * @return Points mapped back to their original space.
 */
MatrixXd inv_sphere_flip(const MatrixXd& flipped, const VectorXd& center, double R);

/**
 * @brief Star-convex polytope vertices (in original space) for query point `pq`.
 * @remark Zhong et al. sphere-flip transform + Qhull convex hull.
 * @param pq Query point (region center).
 * @param cloud Obstacle point cloud (rows = points).
 * @param R Sphere-flip radius.
 * @param sphere_floor If true, sphere-surface samples (2d cardinal + 2^d diagonal
 * directions) are added before Qhull so the result is always at least as large as the
 * sphere-inscribed polytope, even when obstacles are absent or sparse.
 * @return Star-convex hull vertices in original space.
 */
MatrixXd generate_star_convex(const VectorXd& pq, const MatrixXd& cloud, double R,
                               bool sphere_floor = false);

/**
 * @brief Star-convex polytope vertices using a prebuilt spatial index.
 * @remark Index-backed overload of generate_star_convex(): the in-R obstacle set is
 * gathered from `index` instead of a full-cloud scan; results are identical.
 * @param pq Query point (region center).
 * @param index Spatial index over the obstacle cloud (see PointIndex).
 * @param R Sphere-flip radius.
 * @param sphere_floor If true, adds sphere-surface samples for the minimum-size guarantee.
 * @return Star-convex hull vertices in original space.
 */
MatrixXd generate_star_convex(const VectorXd& pq, const PointIndex& index, double R,
                               bool sphere_floor = false);

/**
 * @brief Trims star-convex vertices to a strict convex H-polytope.
 * @remark Qhull convex hull + barycentric simplex containment test.
 * @param star_verts Star-convex hull vertices (e.g. from generate_star_convex()).
 * @param pq Query point (region center).
 * @param A_out Output halfspace normals.
 * @param b_out Output halfspace offsets.
 */
void star_to_convex(const MatrixXd& star_verts, const VectorXd& pq,
                    MatrixXd& A_out, VectorXd& b_out);

/**
 * @brief One-shot: query point + cloud -> obstacle-free convex region.
 * @remark Composes sphere-flip region generation (generate_star_convex(),
 * star_to_convex()) with a tightening pass against nearby cloud points.
 * @param pq Query point (region center).
 * @param cloud Obstacle point cloud (rows = points).
 * @param R Sphere-flip radius.
 * @param tighten If true, a safety pass guarantees no in-R cloud point lies inside the region.
 * @param name Optional region name.
 * @param sphere_floor If true, enables the minimum-size guarantee (see generate_star_convex()).
 * @return The obstacle-free convex region.
 */
ConvexRegion convex_region_from_pointcloud(const VectorXd& pq,
                                           const MatrixXd& cloud, double R,
                                           bool tighten = true,
                                           const std::string& name = "",
                                           bool sphere_floor = false);

/**
 * @brief One-shot region generation using a prebuilt spatial index.
 * @remark Index-backed overload of convex_region_from_pointcloud(): both the region
 * generation and the tightening pass gather their in-R obstacle sets from `index`
 * instead of scanning the whole cloud. Results are identical to the cloud overload.
 * @param pq Query point (region center).
 * @param index Spatial index over the obstacle cloud (see PointIndex).
 * @param R Sphere-flip radius.
 * @param tighten If true, a safety pass guarantees no in-R cloud point lies inside the region.
 * @param name Optional region name.
 * @param sphere_floor If true, enables the minimum-size guarantee (see generate_star_convex()).
 * @return The obstacle-free convex region.
 */
ConvexRegion convex_region_from_pointcloud(const VectorXd& pq,
                                           const PointIndex& index, double R,
                                           bool tighten = true,
                                           const std::string& name = "",
                                           bool sphere_floor = false);

}  // namespace gcs
