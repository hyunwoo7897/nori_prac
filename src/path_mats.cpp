#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h> 
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>


NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList& props){

    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {

        Color3f total(0.0f);
        Color3f throughput(1.0f);
        Ray3f curRay = ray;
        float eta = 1.0f;
        float prob = 1.0f;
        bool isSpecularRay = true;
        int maxDepth = 50;
        for (int i = 0; i < maxDepth; i++) {
            Color3f emitted(0.0f);
            Color3f reflected(0.0f);
            Intersection its;
            Point2f sample = sampler->next2D();
            Point3f samplePos;
            Normal3f sampleNormal;
            if (!scene->rayIntersect(curRay, its)) {
                break;
            }
                
            // Emission
            if (its.mesh->isEmitter()) {
                emitted =  its.mesh->getEmitter()->eval(its.toLocal(-curRay.d.normalized()),&its);
                total += (throughput * emitted / prob);
                
            }
            

            // Direct lighting sampling

            

            // Indirect lighting sampling
            sample = sampler->next2D();
            BSDFQueryRecord bRec(its.toLocal(-curRay.d),  its.toLocal(Normal3f(0.0f, 0.0f, 1.0f)), ESolidAngle, &its);
            Color3f bsdfValue = its.mesh->getBSDF()->sample(bRec, sample);

            throughput *= bsdfValue / prob;

            curRay = Ray3f(its.p, its.toWorld(bRec.wo));
            curRay.mint = Epsilon;
            curRay.maxt = std::numeric_limits<float>::infinity();

            // Russian roulette
            eta *= bRec.eta;
            prob = i < 3 ? 1.0f : std::min(throughput.maxCoeff() *eta*eta, 0.99f);
            if(sampler->next1D() > prob)
                break;
        }
        return total;
    }
    

    std::string toString() const override { return "PathMatsIntegrator[]"; }


protected:
    
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");




NORI_NAMESPACE_END