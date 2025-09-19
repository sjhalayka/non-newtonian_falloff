#include "main.h"


vector_3 random_unit_vector(void)
{
	const real_type z = dis(generator) * 2.0 - 1.0;
	const real_type a = dis(generator) * 2.0 * pi;

	const real_type r = sqrt(1.0f - z * z);
	const real_type x = r * cos(a);
	const real_type y = r * sin(a);

	return vector_3(x, y, z).normalize();
}

vector_3 slerp(vector_3 s0, vector_3 s1, const real_type t)
{
	vector_3 s0_norm = s0;
	s0_norm.normalize();

	vector_3 s1_norm = s1;
	s1_norm.normalize();

	const real_type cos_angle = s0_norm.dot(s1_norm);
	const real_type angle = acos(cos_angle);

	const real_type p0_factor = sin((1 - t) * angle) / sin(angle);
	const real_type p1_factor = sin(t * angle) / sin(angle);

	return s0 * p0_factor + s1 * p1_factor;
}

real_type Lerp(real_type a, real_type b, real_type t)
{
	return a + t * (b - a);
}

bool circle_intersect(
	const vector_3 location,
	const vector_3 normal,
	const real_type unit_circle_distance)
{
	const vector_3 circle_origin(unit_circle_distance, 0, 0);

	if (normal.dot(circle_origin) <= 0)
		return false;

	vector_3 v;
	v.x = location.x + normal.x;
	v.y = location.y + normal.y;
	v.z = location.z + normal.z;

	const real_type ratio = v.x / circle_origin.x;

	v.y = v.y / ratio;
	v.z = v.z / ratio;
	v.x = circle_origin.x;

	vector_3 v2;
	v2.x = circle_origin.x - v.x;
	v2.y = circle_origin.y - v.y;
	v2.z = circle_origin.z - v.z;

	if (v2.length() > 1.0) // is outside unit radius?
		return false;

	return true;
}

long long unsigned int get_intersecting_line_count_integer(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type unit_circle_distance)
{
	long long unsigned int count = 0;

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

		if (circle_intersect(location, normal, unit_circle_distance))
			count++;
	}

	return count;
}


int main(int argc, char** argv)
{
	const real_type emitter_radius = sqrt((1e9 * G * hbar * log(2.0)) / (k * c3 * pi));

	const real_type emitter_area =
		4.0 * pi * emitter_radius * emitter_radius;

	// Field line count
	// re: holographic principle:
	const real_type n =
		(k * c3 * emitter_area)
		/ (log(2.0) * 4.0 * G * hbar);

	const real_type emitter_mass = c2 * emitter_radius / (2.0 * G);

	// constexpr real_type D = 3;
	// constexpr real_type receiver_radius = 1.0;

	// Random outward, random tangent plane, and quantum graphity connections

	const real_type start_pos = /*emitter_radius * */ 100;// 200;
	const real_type end_pos = /*emitter_radius **/ 100.0;
	const size_t pos_res = 2; // Larger than 1
	const real_type pos_step_size = (end_pos - start_pos) / (pos_res - 1);

	for (size_t i = 0; i < pos_res; i++)
	{
		const real_type epsilon = 0.01;

		const real_type unit_circle_distance = start_pos + i * pos_step_size;
		const real_type unit_circle_distance_plus = unit_circle_distance + epsilon;

		// beta function
		const long long unsigned int collision_count_plus_integer =
			get_intersecting_line_count_integer(
				static_cast<long long unsigned int>(n),
				emitter_radius,
				unit_circle_distance_plus);

		// beta function
		const long long unsigned int collision_count_integer =
			get_intersecting_line_count_integer(
				static_cast<long long unsigned int>(n),
				emitter_radius,
				unit_circle_distance);

		// alpha variable
		const real_type gradient_integer =
			(static_cast<real_type>(collision_count_plus_integer) - static_cast<real_type>(collision_count_integer))
			/ epsilon;

		// g variable, doesn't need to be scaled because
		// we are using a unit size receiver
		real_type gradient_strength =
			-gradient_integer;

		// Newtonian acceleration
		const real_type a_Newton =
			sqrt(
				(n * G * c * hbar * log(2.0)) /
				(4 * k * pi * pow(unit_circle_distance, 4.0)));

		// Newtonian speed
//		real_type v_Newton = sqrt(a_Newton * receiver_pos.x);

		// These should match a_Newton
		const real_type a_flat =
			gradient_strength * unit_circle_distance * c * hbar * log(2)
			/ (k * 2 * pi * emitter_mass);

//		real_type v_flat = sqrt(a_flat * receiver_pos.x);

		cout << a_Newton / a_flat << endl;

		cout << endl;
	}

}




