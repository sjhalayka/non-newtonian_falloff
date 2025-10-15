#include "main.h"

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

real_type get_intersecting_line_density(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type receiver_distance,
	const real_type receiver_radius)
{
	real_type count = 0;

	generator.seed(static_cast<unsigned>(0));

	for (long long unsigned int i = 0; i < n; i++)
	{
		if (i % 100000000 == 0)
			cout << float(i) / float(n) << endl;

		const vector_3 p = random_unit_vector();

		vector_3 normal = p;
		vector_3 location = normal;

		location.x *= emitter_radius;
		location.y *= emitter_radius;
		location.z *= emitter_radius;

		std::optional<real_type> i_hit = intersect(location, normal, receiver_distance, receiver_radius);

		if (i_hit)
			count += *i_hit  / (2.0*receiver_radius);
	}

	return count;
}

real_type metres_to_planck_units(const real_type m)
{
	return m / planck_length;
}


// to do: get cube-rays. opposing facet intersections can cause curvature

int main(int argc, char** argv)
{
	ofstream outfile("ratio"); 

	const real_type emitter_radius_geometrized =
		sqrt(1e9 * log(2.0) / pi);

	const real_type receiver_radius_geometrized =
		max(1.0, emitter_radius_geometrized*0.01); // Minimum one Planck unit

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

	// Random outward, random tangent plane, and quantum graphity connections

	real_type start_pos =
		emitter_radius_geometrized
		+ receiver_radius_geometrized;

	real_type end_pos = start_pos * 10;

	//swap(end_pos, start_pos);

	const size_t pos_res = 25; // Minimum 2 steps

	const real_type pos_step_size =
		(end_pos - start_pos)
		/ (pos_res - 1);

	const real_type epsilon =
		0.01;// *receiver_radius_geometrized;

	for (size_t i = 0; i < pos_res; i++)
	{
		const real_type receiver_distance_geometrized =
			start_pos + i * pos_step_size;

		const real_type receiver_distance_plus_geometrized =
			receiver_distance_geometrized + epsilon;

		// beta function
		const real_type collision_count_plus =
			get_intersecting_line_density(
				static_cast<long long unsigned int>(n_geometrized),
				emitter_radius_geometrized,
				receiver_distance_plus_geometrized,
				receiver_radius_geometrized);

		// beta function
		const real_type collision_count =
			get_intersecting_line_density(
				static_cast<long long unsigned int>(n_geometrized),
				emitter_radius_geometrized,
				receiver_distance_geometrized,
				receiver_radius_geometrized);

		// alpha variable
		const real_type gradient_integer =
			(collision_count_plus - collision_count)
			/ epsilon;

		// g variable
		real_type gradient_strength =
			-gradient_integer
			/
			(receiver_radius_geometrized
			* receiver_radius_geometrized
			);

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

		cout << endl;
		cout << a_Newton_geometrized / a_flat_geometrized << endl;
		cout << endl << endl;

		outfile << receiver_distance_geometrized <<
			" " <<
			(a_Newton_geometrized / a_flat_geometrized) <<
			endl;

		//const real_type a_Newton2_geometrized =
		//	emitter_mass_geometrized / pow(receiver_distance_geometrized, 2.0);
		
		//const real_type g_approx =
		//	n_geometrized
		//	/ (2 * pow(receiver_distance_geometrized, 3.0));

		//const real_type a_approx_geometrized =
		//	g_approx * receiver_distance_geometrized * log(2)
		//	/ (2 * pi * emitter_mass_geometrized);
	}

}




