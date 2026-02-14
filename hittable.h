#ifndef HITTABLE_H
#define HITTABLE_H

class material;

class hit_record {
public:
    point3 p;                 // point hit
    vec3 normal;              // normal from surface
    shared_ptr<material> mat; // class to define how ray scatters and attenuates
    double t;                 // t coef for ray component
    bool front_face;          // true if ray is outside hittable

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        // True when ray is outside (norm "bounces back")
        front_face = dot(r.direction(), outward_normal) < 0;
        // Make norm always point against incident ray
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable {
public:
    virtual ~hittable() = default;

    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};

#endif //HITTABLE_H
