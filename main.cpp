#include "main.h"
#include <thread>
#include <vector>
#include <atomic>
#include <random>

real_type intersect_AABB(const vector_3 min_location, const vector_3 max_location, const vector_3 ray_origin, const vector_3 ray_dir, real_type& tmin, real_type& tmax)
{
	tmin = (min_location.x - ray_origin.x) / ray_dir.x;
	tmax = (max_location.x - ray_origin.x) / ray_dir.x;

	if (tmin > tmax)
		swap(tmin, tmax);

	real_type tymin = (min_location.y - ray_origin.y) / ray_dir.y;
	real_type tymax = (max_location.y - ray_origin.y) / ray_dir.y;

	if (tymin > tymax)
		swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return 0;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	real_type tzmin = (min_location.z - ray_origin.z) / ray_dir.z;
	real_type tzmax = (max_location.z - ray_origin.z) / ray_dir.z;

	if (tzmin > tzmax)
		swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return 0;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	if (tmin < 0 || tmax < 0)
		return 0;

	vector_3 ray_hit_start = ray_origin;
	ray_hit_start.x += ray_dir.x * tmin;
	ray_hit_start.y += ray_dir.y * tmin;
	ray_hit_start.z += ray_dir.z * tmin;

	vector_3 ray_hit_end = ray_origin;
	ray_hit_end.x += ray_dir.x * tmax;
	ray_hit_end.y += ray_dir.y * tmax;
	ray_hit_end.z += ray_dir.z * tmax;

	real_type l = (ray_hit_end - ray_hit_start).length();

	return l;
}

vector_3 random_unit_vector(void)
{
	const real_type z = dis(generator) * 2.0 - 1.0;
	const real_type a = dis(generator) * 2.0 * pi;

	const real_type r = sqrt(1.0f - z * z);
	const real_type x = r * cos(a);
	const real_type y = r * sin(a);

	return vector_3(x, y, z).normalize();
}

std::optional<real_type> intersect(
	const vector_3 location,
	const vector_3 normal,
	const real_type receiver_distance,
	const real_type receiver_radius)
{
	const vector_3 circle_origin(receiver_distance, 0, 0);

	if (normal.dot(circle_origin) <= 0)
		return std::nullopt;

	vector_3 min_location(-receiver_radius + receiver_distance, -receiver_radius, -receiver_radius);
	vector_3 max_location(receiver_radius + receiver_distance, receiver_radius, receiver_radius);

	real_type tmin = 0, tmax = 0;

	real_type AABB_hit = intersect_AABB(min_location, max_location, location, normal, tmin, tmax);

	if (AABB_hit > 0)
		return AABB_hit;

	return std::nullopt;
}

vector_3 random_tangent_vector(const vector_3& point_on_sphere)
{
	// Normalize to ensure it's on unit sphere
	vector_3 normal = point_on_sphere;
	normal.normalize();

	// Choose an arbitrary vector that's not parallel to normal
	vector_3 arbitrary;
	if (fabs(normal.x) > 0.9)
		arbitrary = vector_3(0, 1, 0);  // If normal is mostly along x, use y
	else
		arbitrary = vector_3(1, 0, 0);  // Otherwise use x

	// Get first basis vector perpendicular to normal
	vector_3 tangent1 = normal.cross(arbitrary);
	tangent1.normalize();

	// Get second basis vector perpendicular to both
	vector_3 tangent2 = normal.cross(tangent1);
	tangent2.normalize();

	// Generate random angle for rotation in tangent plane
	real_type angle = dis(generator) * 2.0 * pi;

	// Combine the two tangent vectors with random weights
	vector_3 result = tangent1 * cos(angle) + tangent2 * sin(angle);

	return result.normalize();
}


vector_3 random_cosine_weighted_hemisphere(const vector_3& normal)
{
	// Generate two random numbers
	real_type u1 = dis(generator);
	real_type u2 = dis(generator);

	// Malley's method
	// (cosine-weighted hemisphere sampling)
	// Sample uniformly on a disk, 
	// then project up to hemisphere
	real_type r = sqrt(u1);
	real_type theta = 2.0 * pi * u2;

	// Point on unit disk
	real_type x = r * cos(theta);
	real_type y = r * sin(theta);
	real_type z = sqrt(1.0 - u1); // Height above disk

	// Create orthonormal basis around normal
	vector_3 n = normal;
	n.normalize();

	// Choose an arbitrary vector not parallel to normal
	vector_3 arbitrary;
	if (fabs(n.x) > 0.9)
		arbitrary = vector_3(0, 1, 0);
	else
		arbitrary = vector_3(1, 0, 0);

	// Create tangent and bitangent
	vector_3 tangent = n.cross(arbitrary);
	tangent.normalize();

	vector_3 bitangent = n.cross(tangent);
	bitangent.normalize();

	// Transform from local coordinates
	// to world coordinates
	vector_3 result;
	result.x = tangent.x * x +
		bitangent.x * y +
		n.x * z;

	result.y = tangent.y * x +
		bitangent.y * y +
		n.y * z;

	result.z = tangent.z * x +
		bitangent.z * y +
		n.z * z;

	return result.normalize();
}


real_type get_intersecting_line_density(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type receiver_distance,
	const real_type receiver_distance_plus,
	const real_type receiver_radius)
{
	if (n == 0) return 0.0;

	const unsigned long long batch_size = 100000000ULL;
	size_t num_threads = std::thread::hardware_concurrency();
	real_type total_count = 0.0;
	real_type total_count_plus = 0.0;
	unsigned long long remaining = n;
	unsigned long long completed = 0;

	while (remaining > 0) {
		unsigned long long curr_batch = std::min(batch_size, remaining);

		std::vector<std::thread> threads;
		std::atomic<real_type> batch_count{ 0.0 };
		std::atomic<real_type> batch_count_plus{ 0.0 };

		unsigned long long rays_per_thread = curr_batch / num_threads;
		unsigned long long extra_rays = curr_batch % num_threads;
		unsigned long long th_start = 0;

		for (size_t t = 0; t < num_threads; ++t) {
			unsigned long long th_size = rays_per_thread + (t < extra_rays ? 1ULL : 0ULL);
			if (th_size == 0) continue;

			threads.emplace_back([&](unsigned long long ts, unsigned long long te) {
				std::random_device rd;
				std::mt19937 gen(rd());
				std::uniform_real_distribution<real_type> local_dis(0.0, 1.0);

				real_type lcount = 0.0;
				real_type lcplus = 0.0;

				for (unsigned long long j = ts; j < te; ++j) {
					// Generate random unit vector for location
					real_type z_loc = local_dis(gen) * 2.0 - 1.0;
					real_type a_loc = local_dis(gen) * 2.0 * pi;
					real_type r_loc = sqrt(1.0 - z_loc * z_loc);
					real_type x_loc = r_loc * cos(a_loc);
					real_type y_loc = r_loc * sin(a_loc);
					vector_3 loc(x_loc, y_loc, z_loc);
					loc.x *= emitter_radius;
					loc.y *= emitter_radius;
					loc.z *= emitter_radius;

					vector_3 sn = loc;
					sn.normalize();

					// Generate cosine-weighted hemisphere direction
					real_type u1 = local_dis(gen);
					real_type u2 = local_dis(gen);
					real_type r_hem = sqrt(u1);
					real_type theta_hem = 2.0 * pi * u2;
					real_type x_hem = r_hem * cos(theta_hem);
					real_type y_hem = r_hem * sin(theta_hem);
					real_type z_hem = sqrt(1.0 - u1);

					vector_3 arb;
					if (fabs(sn.x) > 0.9) {
						arb = vector_3(0, 1, 0);
					}
					else {
						arb = vector_3(1, 0, 0);
					}

					vector_3 tan1 = sn.cross(arb);
					tan1.normalize();
					vector_3 tan2 = sn.cross(tan1);
					tan2.normalize();

					vector_3 dir;
					dir.x = tan1.x * x_hem + tan2.x * y_hem + sn.x * z_hem;
					dir.y = tan1.y * x_hem + tan2.y * y_hem + sn.y * z_hem;
					dir.z = tan1.z * x_hem + tan2.z * y_hem + sn.z * z_hem;
					dir.normalize();

					// Intersect
					auto hit = intersect(loc, dir, receiver_distance, receiver_radius);
					if (hit.has_value()) {
						lcount += hit.value() / (2.0 * receiver_radius);
					}

					hit = intersect(loc, dir, receiver_distance_plus, receiver_radius);
					if (hit.has_value()) {
						lcplus += hit.value() / (2.0 * receiver_radius);
					}
				}

				batch_count.fetch_add(lcount);
				batch_count_plus.fetch_add(lcplus);
				}, th_start, th_start + th_size);

			th_start += th_size;
		}

		for (auto& th : threads) {
			th.join();
		}

		total_count += batch_count.load();
		total_count_plus += batch_count_plus.load();

		completed += curr_batch;
		remaining -= curr_batch;

		cout << float(completed) / float(n) << endl;
	}

	return total_count_plus - total_count;
}

int main(int argc, char** argv)
{
	ofstream outfile("ratio");

	const real_type emitter_radius_geometrized =
		sqrt(1e9 * log(2.0) / pi);

	const real_type receiver_radius_geometrized =
		emitter_radius_geometrized * 1; // Minimum one Planck unit

	const real_type emitter_area_geometrized =
		4.0 * pi
		* emitter_radius_geometrized
		* emitter_radius_geometrized;

	// Field line count
	const real_type n_geometrized =
		emitter_area_geometrized
		/ (log(2.0) * 4.0);

	const real_type emitter_mass_geometrized =
		emitter_radius_geometrized
		/ 2.0;

	real_type start_pos =
		emitter_radius_geometrized
		+ receiver_radius_geometrized;

	real_type end_pos = start_pos * 10;

	swap(end_pos, start_pos);

	const size_t pos_res = 10; // Minimum 2 steps

	const real_type pos_step_size =
		(end_pos - start_pos)
		/ (pos_res - 1);

	const real_type epsilon =
		receiver_radius_geometrized;


	for (size_t i = 0; i < pos_res; i++)
	{
		const real_type receiver_distance_geometrized =
			start_pos + i * pos_step_size;

		const real_type receiver_distance_plus_geometrized =
			receiver_distance_geometrized + epsilon;

		// beta function
		const real_type collision_count_plus_minus_collision_count =
			get_intersecting_line_density(
				static_cast<long long unsigned int>(n_geometrized),
				emitter_radius_geometrized,
				receiver_distance_geometrized,
				receiver_distance_plus_geometrized,
				receiver_radius_geometrized);

		// alpha variable
		const real_type gradient_integer =
			collision_count_plus_minus_collision_count
			/ epsilon;

		// g variable
		real_type gradient_strength =
			-gradient_integer
			/
			(receiver_radius_geometrized
				* receiver_radius_geometrized
				);

		//cout << gradient_strength << " " << n_geometrized / (2 * pow(receiver_distance_geometrized, 3.0)) << endl;
		//cout << gradient_strength / (n_geometrized / (2 * pow(receiver_distance_geometrized, 3.0))) << endl;


		const real_type a_Newton_geometrized =
			sqrt(
				n_geometrized * log(2.0)
				/
				(4.0 * pi *
					pow(receiver_distance_geometrized, 4.0))
			);

		const real_type a_flat_geometrized =
			gradient_strength * receiver_distance_geometrized * log(2)
			/ (8.0 * emitter_mass_geometrized);


		//const real_type g_approx = n_geometrized / (2 * pow(receiver_distance_geometrized, 3.0));
		//const real_type a_approx_geometrized =
		//	g_approx * receiver_distance_geometrized * log(2)
		//	/ (8.0 * emitter_mass_geometrized);


		const real_type dt_Schwarzschild = sqrt(1 - emitter_radius_geometrized / receiver_distance_geometrized);

		const real_type a_Schwarzschild_geometrized =
			emitter_radius_geometrized / (pi * pow(receiver_distance_geometrized, 2.0) * dt_Schwarzschild);

		cout << "a_Schwarzschild_geometrized " << a_Schwarzschild_geometrized << endl;
		cout << "a_Newton_geometrized " << a_Newton_geometrized << endl;
		cout << "a_flat_geometrized " << a_flat_geometrized << endl;
		cout << a_Schwarzschild_geometrized / a_flat_geometrized << endl;
		cout << endl;
		cout << a_Newton_geometrized / a_flat_geometrized << endl;
		cout << endl << endl;

		outfile << receiver_distance_geometrized <<
			" " <<
			(a_Schwarzschild_geometrized / a_flat_geometrized) <<
			endl;
	}

}