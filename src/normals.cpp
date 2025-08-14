#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h> 
#include <nori/warp.h>
NORI_NAMESPACE_BEGIN

class NormalIntegrator : public Integrator {
public:
    NormalIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const {
        return "NormalIntegrator[]";
    }
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList& props){
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }



    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.f);
        }

        Vector3f lightDir = m_position - its.p;
        float distance2 = lightDir.squaredNorm();
        lightDir.normalize();

        Ray3f shadowRay(its.p, lightDir);

        if (scene->rayIntersect(shadowRay)) {
            return Color3f(0.f);
        } else {
            float cosTheta = std::max(0.0f, lightDir.dot(its.shFrame.n.normalized()));
            return m_energy * cosTheta / (4.0f * M_PI * M_PI * distance2);
        }
    }

    std::string toString() const override { return "SimpleIntegrator[]"; }


protected:
    Point3f m_position;
    Color3f m_energy;
    
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple")



class AoIntegrator : public Integrator {
public:
    AoIntegrator(const PropertyList& props){ }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.f);
        }

        float ao = 0.0f;
        int samples = 10;
        for (int i = 0; i < samples; ++i) {
            // Sample a point on the hemisphere
            Point2f sample = sampler->next2D();
            Vector3f localDir = Warp::squareToCosineHemisphere(sample);
            Vector3f dir = its.shFrame.toWorld(localDir);
            Ray3f aoRay(its.p, dir);

            // Check if the ray intersects with any geometry
            if (!scene->rayIntersect(aoRay)) {
                ao += 1.0f;
            }
        }
    
        return Color3f(ao/samples);
    }

    std::string toString() const override { return "AoIntegrator[]"; }


};

NORI_REGISTER_CLASS(AoIntegrator, "ao")




NORI_NAMESPACE_END