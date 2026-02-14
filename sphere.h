#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable {
public:
    // Stationary sphere
    sphere(const point3& static_center, double radius, shared_ptr<material> mat)
        : center(static_center, vec3(0, 0, 0)), radius(std::fmax(0, radius)), mat(mat) {}

    // Moving sphere
    sphere(const point3& start_center, const point3& dest_center, double radius, shared_ptr<material> mat)
        : center(start_center, dest_center - start_center), radius(std::fmax(0, radius)), mat(mat) {}

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        point3 current_center = center.at(r.time());
        vec3 oc = current_center - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius*radius;

        auto discriminant = h*h - a*c;
        if (discriminant < 0)
            return false;

        auto sqrtd = std::sqrt(discriminant);

        // Find nearest root that lies within acceptable range.
        auto root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root; // solved t for ray intersection with surface
        rec.p = r.at(rec.t); // point on the sphere surface, calc from rayline

        // set norm to always be outwards
        vec3 outward_normal = (rec.p - current_center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

private:
    ray center;
    double radius;
    shared_ptr<material> mat;
};

#endif //SPHERE_H
