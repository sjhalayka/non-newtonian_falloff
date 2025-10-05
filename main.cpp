#include "main.h"


struct Ray {
	vector_3 origin;
	vector_3 direction; // Should be normalized

	Ray(const vector_3& o, vector_3 d) : origin(o), direction(d)
	{
		direction.normalize();
	}

	vector_3 at(real_type t) const {

		vector_3 d(direction.x * t, direction.y * t, direction.z * t);

		vector_3 o = origin;
		o.x += d.x;
		o.y += d.y;
		o.z += d.z;

		return o;
	}
};

struct Sphere {
	vector_3 center;
	real_type radius;

	Sphere(const vector_3& c, real_type r) : center(c), radius(r) {}
};

struct HitInfo {
	real_type t;           // Distance along ray
	vector_3 point;        // Intersection point
	vector_3 normal;       // Surface normal at hit point
	bool frontFace;    // Did ray hit from outside?
};

// Ray-sphere intersection using geometric method
std::optional<HitInfo> raySphereIntersect(const Ray& ray, const Sphere& sphere) {
	vector_3 oc = ray.origin - sphere.center;

	// Quadratic equation coefficients: at^2 + bt + c = 0
	real_type a = ray.direction.dot(ray.direction);
	real_type b = 2.0f * oc.dot(ray.direction);
	real_type c = oc.dot(oc) - sphere.radius * sphere.radius;

	real_type discriminant = b * b - 4 * a * c;

	// No intersection
	if (discriminant < 0) {
		return std::nullopt;
	}

	// Calculate nearest intersection
	real_type sqrtDisc = std::sqrt(discriminant);
	real_type t = (-b - sqrtDisc) / (2.0f * a);

	// If nearest point is behind ray origin, try far intersection
	if (t < 0.001f) {
		t = (-b + sqrtDisc) / (2.0f * a);
		if (t < 0.001f) {
			return std::nullopt; // Both intersections behind ray
		}
	}

	// Calculate hit information
	HitInfo hit;
	hit.t = t;
	hit.point = ray.at(t);
	hit.normal = (hit.point - sphere.center).normalize();
	hit.frontFace = ray.direction.dot(hit.normal) < 0;

	// Flip normal if hit from inside
	if (!hit.frontFace) {
		hit.normal = hit.normal * -1.0f;
	}

	return hit;
}

// Optimized version using reduced quadratic form
std::optional<HitInfo> raySphereIntersectOptimized(const Ray& ray, const Sphere& sphere) {
	vector_3 oc = ray.origin - sphere.center;

	// Using half-b optimization: t^2 + (2h)t + c = 0
	real_type a = ray.direction.dot(ray.direction);
	real_type h = oc.dot(ray.direction);
	real_type c = oc.dot(oc) - sphere.radius * sphere.radius;

	real_type discriminant = h * h - a * c;

	if (discriminant < 0) {
		return std::nullopt;
	}

	real_type sqrtDisc = std::sqrt(discriminant);
	real_type t = (-h - sqrtDisc) / a;

	if (t < 0.001f) {
		t = (-h + sqrtDisc) / a;
		if (t < 0.001f) {
			return std::nullopt;
		}
	}

	HitInfo hit;
	hit.t = t;
	hit.point = ray.at(t);
	hit.normal = (hit.point - sphere.center).normalize();
	hit.frontFace = ray.direction.dot(hit.normal) < 0;

	if (!hit.frontFace) {
		hit.normal = hit.normal * -1.0f;
	}

	return hit;
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

bool circle_intersect(
	const vector_3 location,
	const vector_3 normal,
	const real_type receiver_distance,
	const real_type receiver_radius)
{
	Ray r(location, normal);

	Sphere s(vector_3(receiver_distance, 0, 0), receiver_radius);
	double t_hit = 0;

	std::optional<HitInfo> hit = raySphereIntersectOptimized(r, s);

	if (hit)
		return true;
	else
		return false;
}

long long unsigned int get_intersecting_line_count_integer(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type receiver_distance,
	const real_type receiver_radius
)
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

		if (circle_intersect(location, normal, receiver_distance, receiver_radius))
			count++;
	}

	return count;
}

real_type metres_to_planck_units(const real_type m)
{
	return m / planck_length;
}


int main(int argc, char** argv)
{
	ofstream outfile("ratio");

	const real_type receiver_radius_geometrized = 
		metres_to_planck_units(1.0); // Minimum one Planck unit

	const real_type emitter_radius_geometrized = 
		sqrt(1e10 * log(2.0) / pi);

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

	const real_type start_pos = 
		emitter_radius_geometrized 
		+ receiver_radius_geometrized;

	const real_type end_pos = metres_to_planck_units(1000.0);

	const size_t pos_res = 100; // Larger than 1

	const real_type pos_step_size = 
		(end_pos - start_pos) 
		/ (pos_res - 1);

	for (size_t i = 0; i < pos_res; i++)
	{
		const real_type epsilon = 
			0.01 * receiver_radius_geometrized;

		const real_type receiver_distance_geometrized = 
			start_pos + i * pos_step_size;

		const real_type receiver_distance_plus_geometrized = 
			receiver_distance_geometrized + epsilon;

		// beta function
		const long long unsigned int collision_count_plus_integer =
			get_intersecting_line_count_integer(
				static_cast<long long unsigned int>(n_geometrized),
				emitter_radius_geometrized,
				receiver_distance_plus_geometrized,
				receiver_radius_geometrized);

		// beta function
		const long long unsigned int collision_count_integer =
			get_intersecting_line_count_integer(
				static_cast<long long unsigned int>(n_geometrized),
				emitter_radius_geometrized,
				receiver_distance_geometrized,
				receiver_radius_geometrized);

		// alpha variable
		const real_type gradient_integer =
			(static_cast<real_type>(collision_count_plus_integer) 
			- static_cast<real_type>(collision_count_integer))
			/ epsilon;

		// g variable
		real_type gradient_strength =
			-gradient_integer 
			/ 
			(receiver_radius_geometrized 
			* receiver_radius_geometrized);

		const real_type a_Newton_geometrized =
			sqrt(
				n_geometrized * log(2.0)
				/ (4 * pi * pow(receiver_distance_geometrized, 4.0))
			);

		const real_type a_flat_geometrized =
			gradient_strength * receiver_distance_geometrized * log(2)
			/ (2 * pi * emitter_mass_geometrized);

		cout << endl;
		cout << a_Newton_geometrized / a_flat_geometrized << endl;
		cout << endl << endl;

		outfile << receiver_radius_geometrized << 
			" " << 
			(a_Newton_geometrized / a_flat_geometrized) << 
			endl;
	}

}




