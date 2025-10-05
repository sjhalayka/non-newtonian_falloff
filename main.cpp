#include "main.h"



// https://chatgpt.com/c/68e2df9e-b1c8-832e-a627-9fac8a379675


struct Ray {
	vector_3 origin;
	vector_3 dir;  // should be normalized
};

struct Sphere {
	vector_3 center;
	real_type radius;
};

// Returns the closest positive intersection distance, if any.
std::optional<real_type> intersectRaySphere(const Ray& ray, const Sphere& sphere) 
{
	vector_3 oc = ray.origin - sphere.center;

	real_type a = ray.dir.dot(ray.dir);
	real_type b = 2.0f * oc.dot(ray.dir);
	real_type c = oc.dot(oc) - sphere.radius * sphere.radius;

	real_type discriminant = b * b - 4 * a * c;
	if (discriminant < 0.0f)
		return std::nullopt; // no intersection

	real_type sqrtD = std::sqrt(discriminant);
	real_type t1 = (-b - sqrtD) / (2.0f * a);
	real_type t2 = (-b + sqrtD) / (2.0f * a);

	// Return the nearest positive t (intersection distance)
	if (t1 > 0.0f && t2 > 0.0f)
		return std::min(t1, t2);
	else if (t1 > 0.0f)
		return t1;
	else if (t2 > 0.0f)
		return t2;
	else
		return std::nullopt;
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
	Ray ray{ location, normal };

	Sphere sphere{ {receiver_distance, 0.0, 0.0}, receiver_radius };

	optional<real_type> t = intersectRaySphere(ray, sphere);
	if (t) {
		//vector_3 hitPoint = ray.origin + ray.dir * (*t);
		//std::cout << "Hit at distance " << *t
		//	<< " at point (" << hitPoint.x << ", "
		//	<< hitPoint.y << ", " << hitPoint.z << ")\n";

		return true;
	}

	return false;




	//const vector_3 circle_origin(receiver_distance, 0, 0);

	//if (normal.dot(circle_origin) <= 0)
	//	return false;

	//vector_3 v;
	//v.x = location.x + normal.x;
	//v.y = location.y + normal.y;
	//v.z = location.z + normal.z;

	//const real_type ratio = v.x / circle_origin.x;

	//v.y = v.y / ratio;
	//v.z = v.z / ratio;
	//v.x = circle_origin.x;

	//vector_3 v2;
	//v2.x = circle_origin.x - v.x;
	//v2.y = circle_origin.y - v.y;
	//v2.z = circle_origin.z - v.z;

	//if (v2.length() > receiver_radius)
	//	return false;

	//return true;
}


long long unsigned int get_intersecting_line_count_integer(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type receiver_distance,
	const real_type receiver_radius
)
{
	
	// Random outward, random tangent plane, and quantum graphity connections


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
	outfile << setprecision(30);



	const real_type emitter_radius_geometrized = 
		sqrt(1e9 * log(2.0) / pi);

	const real_type receiver_radius_geometrized =
		emitter_radius_geometrized;// 1.0;// metres_to_planck_units(1.0); // Minimum one Planck unit




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

	real_type end_pos = start_pos * 100;// metres_to_planck_units(1000.0);


//	swap(end_pos, start_pos);

	const size_t pos_res = 10; // Larger than 1

	const real_type pos_step_size = 
		(end_pos - start_pos) 
		/ (pos_res - 1);

	for (size_t i = 0; i < pos_res; i++)
	{
		const real_type receiver_distance_geometrized = 
			start_pos + i * pos_step_size;

		const real_type epsilon =
			0.01 * receiver_distance_geometrized;

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
		cout << collision_count_plus_integer << " " << collision_count_integer << endl;
		cout << a_Newton_geometrized / a_flat_geometrized << endl;
		cout << endl << endl;

		outfile << receiver_radius_geometrized << 
			" " << 
			(a_Newton_geometrized / a_flat_geometrized) << 
			endl;
	}

}




